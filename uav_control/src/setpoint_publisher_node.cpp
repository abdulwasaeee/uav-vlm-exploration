// setpoint_publisher_node.cpp
// Full lifecycle offboard controller:
//   STARTUP → TAKEOFF → HOVER → AUTONOMOUS → HOVER (on mission complete)
//
// Replaces offboard_controller.py entirely — handles arming, takeoff,
// then hands control to VFH3D velocity commands.
//
// Subscribes:
//   /uav/cmd_vel               (geometry_msgs/TwistStamped)  ← VFH3D (ENU)
//   /uav/mission_complete      (std_msgs/Bool)
//   /fmu/out/vehicle_odometry  (px4_msgs/VehicleOdometry)
//   /fmu/out/vehicle_status_v2 (px4_msgs/VehicleStatus)
//   /fmu/out/vehicle_local_position_v1 (px4_msgs/VehicleLocalPosition)
//
// Publishes:
//   /fmu/in/offboard_control_mode  (px4_msgs/OffboardControlMode)
//   /fmu/in/trajectory_setpoint    (px4_msgs/TrajectorySetpoint)
//   /fmu/in/vehicle_command        (px4_msgs/VehicleCommand)

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <cmath>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

enum class FlightState { STARTUP, TAKEOFF, HOVER, ROTATING, AUTONOMOUS };

class SetpointPublisher : public rclcpp::Node
{
public:
  SetpointPublisher() : Node("setpoint_publisher")
  {
    // ── Parameters ───────────────────────────────────────────────────
    publish_rate_hz_  = declare_parameter("publish_rate_hz",  20.0);
    takeoff_height_   = declare_parameter("takeoff_height",   -2.0); // NED negative=up
    takeoff_threshold_= declare_parameter("takeoff_threshold", 0.8); // fraction of height
    startup_delay_s_  = declare_parameter("startup_delay_s",   5.0);
    cmd_timeout_s_    = declare_parameter("cmd_timeout_s",     1.0);  // increased for SPF phase transitions

    // ── PX4 QoS — matches offboard_controller.py ─────────────────────
    rclcpp::QoS px4_qos(1);
    px4_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    px4_qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    px4_qos.history(rclcpp::HistoryPolicy::KeepLast);

    // ── Publishers ───────────────────────────────────────────────────
    offboard_pub_  = create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", px4_qos);
    setpoint_pub_  = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", px4_qos);
    cmd_pub_       = create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", px4_qos);

    // ── Subscribers ──────────────────────────────────────────────────
    // VFH3D velocity commands (ENU/map frame)
    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        "/uav/cmd_vel", rclcpp::QoS(10),
        [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(cmd_mutex_);
          cmd_vx_ = msg->twist.linear.x;
          cmd_vy_ = msg->twist.linear.y;
          cmd_vz_ = msg->twist.linear.z;
          last_cmd_time_ = get_clock()->now();
          have_cmd_ = true;
        });

    // Yaw tracking from PX4 odometry
    odom_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry",
        rclcpp::QoS(10).best_effort(),
        [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg) {
          float w=msg->q[0], x=msg->q[1], y=msg->q[2], z=msg->q[3];
          current_yaw_ned_ = std::atan2(2.0f*(w*z+x*y), 1.0f-2.0f*(y*y+z*z));
          // Default target yaw = current yaw (updated by SPF orchestrator if needed)
          if (state_ != FlightState::ROTATING)
            target_yaw_ned_ = current_yaw_ned_;
        });

    // Vehicle status for armed + offboard state
    auto status_topic = declare_parameter("vehicle_status_topic",
                                          std::string("/fmu/out/vehicle_status_v2"));
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
        status_topic, px4_qos,
        [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
          armed_        = (msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED);
          in_offboard_  = (msg->nav_state    == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
        });

    // Local position for takeoff detection
    pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
        "/fmu/out/vehicle_local_position_v1", px4_qos,
        [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
          local_z_ = msg->z;  // NED: negative when airborne
        });

    // Mission complete flag
    complete_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/uav/mission_complete", rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
            mission_complete_ = true;
            state_ = FlightState::HOVER;
            RCLCPP_INFO(get_logger(), "Mission complete — holding hover");
          } else {
            // New goal incoming — full reset
            mission_complete_ = false;
            have_cmd_         = false;  // force wait for fresh cmd_vel
            RCLCPP_INFO(get_logger(), "Mission reset — waiting for VFH3D commands");
          }
        });

    // ── Main timer ────────────────────────────────────────────────────
    startup_ticks_needed_ = static_cast<int>(startup_delay_s_ * publish_rate_hz_);
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_hz_),
        std::bind(&SetpointPublisher::tick, this));

    RCLCPP_INFO(get_logger(),
        "SetpointPublisher ready — waiting %.0fs for EKF2...", startup_delay_s_);
  }

private:
  void tick()
  {
    auto now_us = static_cast<uint64_t>(get_clock()->now().nanoseconds() / 1000ULL);

    // ── Always send heartbeat first ───────────────────────────────────
    bool use_velocity = (state_ == FlightState::HOVER ||
                         state_ == FlightState::AUTONOMOUS);
    publishHeartbeat(use_velocity, !use_velocity, now_us);

    // ── State machine ─────────────────────────────────────────────────
    switch (state_) {

      case FlightState::STARTUP: {
        // Send takeoff setpoint during warmup so PX4 accepts offboard mode
        publishPositionSetpoint(0.0f, 0.0f, static_cast<float>(takeoff_height_), now_us);
        ++startup_ticks_;
        if (startup_ticks_ % static_cast<int>(publish_rate_hz_) == 0) {
          double rem = (startup_ticks_needed_ - startup_ticks_) / publish_rate_hz_;
          if (rem > 0)
            RCLCPP_INFO(get_logger(), "EKF2 warmup... %.0fs remaining", rem);
        }
        if (startup_ticks_ >= startup_ticks_needed_) {
          state_ = FlightState::TAKEOFF;
          RCLCPP_INFO(get_logger(), "Warmup done — engaging offboard + arming");
        }
        break;
      }

      case FlightState::TAKEOFF: {
        publishPositionSetpoint(0.0f, 0.0f, static_cast<float>(takeoff_height_), now_us);

        // Engage offboard mode + arm (throttled)
        if (++cmd_counter_ >= cmd_throttle_) {
          cmd_counter_ = 0;
          if (!in_offboard_) engageOffboard();
          else if (!armed_)  arm();
        }

        // Check if takeoff height reached (NED: z < threshold * height)
        if (armed_ && local_z_ < (takeoff_height_ * takeoff_threshold_)) {
          state_ = FlightState::HOVER;
          RCLCPP_INFO(get_logger(), "Takeoff complete at z=%.2fm — ready for goals", local_z_);
        }
        break;
      }

      case FlightState::HOVER: {
        publishVelocitySetpoint(0.0f, 0.0f, 0.0f, now_us);

        // Transition to ROTATING if mission_phase says so
        if (mission_phase_ == "ROTATING") {
          state_ = FlightState::ROTATING;
          RCLCPP_INFO(get_logger(), "Mission phase ROTATING — entering yaw-only mode");
          break;
        }

        // Transition to autonomous when VFH3D sends a fresh command
        if (have_cmd_ && !mission_complete_) {
          bool stale = (get_clock()->now() - last_cmd_time_).seconds() > cmd_timeout_s_;
          if (!stale) {
            state_ = FlightState::AUTONOMOUS;
            RCLCPP_INFO(get_logger(), "Fresh VFH3D cmd received — switching to AUTONOMOUS");
          } else {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "HOVER: waiting for fresh VFH3D cmd (last was %.1fs ago)",
                (get_clock()->now() - last_cmd_time_).seconds());
          }
        }
        break;
      }

      case FlightState::ROTATING: {
        // Yaw-only mode: velocity=0, yaw=target
        // publishHeartbeat with position=true for stable hover
        publishHeartbeat(false, true, now_us);

        // Latch hover position on entry
        if (!rotating_pos_latched_) {
          rotating_x_ = 0.0f;  // use NaN position — velocity hold
          rotating_y_ = 0.0f;
          rotating_z_ = local_z_;
          rotating_pos_latched_ = true;
          RCLCPP_INFO(get_logger(), "ROTATING: latched hover position z=%.2f", rotating_z_);
        }

        // Emit velocity=0 + target yaw — Option A from Shantam's doc
        px4_msgs::msg::TrajectorySetpoint sp;
        sp.position  = {std::nanf(""), std::nanf(""), std::nanf("")};
        sp.velocity  = {0.0f, 0.0f, 0.0f};
        sp.yaw       = target_yaw_ned_;
        sp.yawspeed  = 0.0f;
        sp.timestamp = now_us;
        setpoint_pub_->publish(sp);

        // Transition out
        if (mission_phase_ == "TRANSLATING" || mission_phase_ == "COMPLETE") {
          rotating_pos_latched_ = false;
          state_ = FlightState::AUTONOMOUS;
          RCLCPP_INFO(get_logger(), "ROTATING done — switching to AUTONOMOUS");
        } else if (mission_phase_ == "IDLE") {
          rotating_pos_latched_ = false;
          state_ = FlightState::HOVER;
        }
        break;
      }

      case FlightState::AUTONOMOUS: {
        std::lock_guard<std::mutex> lk(cmd_mutex_);

        // Fall back to hover if cmd is stale
        bool stale = !have_cmd_ ||
            (get_clock()->now() - last_cmd_time_).seconds() > cmd_timeout_s_;
        if (stale) {
          state_ = FlightState::HOVER;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
              "VFH3D cmd stale — hovering");
          publishVelocitySetpoint(0.0f, 0.0f, 0.0f, now_us);
          return;
        }

        // ENU → NED:  ned_x =  enu_y (North)
        //             ned_y =  enu_x (East)
        //             ned_z = -enu_z (Down)
        float ned_vx = static_cast<float>( cmd_vy_);
        float ned_vy = static_cast<float>( cmd_vx_);
        float ned_vz = static_cast<float>(-cmd_vz_);
        publishVelocitySetpoint(ned_vx, ned_vy, ned_vz, now_us);
        break;
      }
    }
  }

  // ── Publish helpers ───────────────────────────────────────────────────
  void publishHeartbeat(bool velocity, bool position, uint64_t ts)
  {
    px4_msgs::msg::OffboardControlMode hb;
    hb.position     = position;
    hb.velocity     = velocity;
    hb.acceleration = false;
    hb.attitude     = false;
    hb.body_rate    = false;
    hb.timestamp    = ts;
    offboard_pub_->publish(hb);
  }

  void publishPositionSetpoint(float x, float y, float z, uint64_t ts)
  {
    px4_msgs::msg::TrajectorySetpoint sp;
    sp.position  = {x, y, z};
    sp.yaw       = 0.0f;
    sp.timestamp = ts;
    setpoint_pub_->publish(sp);
  }

  void publishVelocitySetpoint(float vx, float vy, float vz, uint64_t ts)
  {
    px4_msgs::msg::TrajectorySetpoint sp;
    sp.position  = {std::nanf(""), std::nanf(""), std::nanf("")};
    sp.velocity  = {vx, vy, vz};
    sp.yaw       = current_yaw_ned_;
    sp.yawspeed  = 0.0f;
    sp.timestamp = ts;
    setpoint_pub_->publish(sp);
  }

  void arm()
  {
    px4_msgs::msg::VehicleCommand msg;
    msg.command          = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
    msg.param1           = 1.0f;
    msg.target_system    = 1;
    msg.target_component = 1;
    msg.source_system    = 1;
    msg.source_component = 1;
    msg.from_external    = true;
    msg.timestamp        = static_cast<uint64_t>(get_clock()->now().nanoseconds()/1000ULL);
    cmd_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Arm command sent");
  }

  void engageOffboard()
  {
    px4_msgs::msg::VehicleCommand msg;
    msg.command          = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
    msg.param1           = 1.0f;
    msg.param2           = 6.0f;
    msg.target_system    = 1;
    msg.target_component = 1;
    msg.source_system    = 1;
    msg.source_component = 1;
    msg.from_external    = true;
    msg.timestamp        = static_cast<uint64_t>(get_clock()->now().nanoseconds()/1000ULL);
    cmd_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Offboard mode command sent");
  }

  // ── State ─────────────────────────────────────────────────────────────
  FlightState state_{FlightState::STARTUP};
  std::string mission_phase_{"IDLE"};
  float  target_yaw_ned_{0.0f};
  bool   rotating_pos_latched_{false};
  float  rotating_x_{0.0f}, rotating_y_{0.0f}, rotating_z_{0.0f};

  double cmd_vx_{0}, cmd_vy_{0}, cmd_vz_{0};
  float  current_yaw_ned_{0.0f};
  float  local_z_{0.0f};
  bool   armed_{false}, in_offboard_{false};
  bool   have_cmd_{false}, mission_complete_{false};
  rclcpp::Time last_cmd_time_;
  std::mutex   cmd_mutex_;

  int startup_ticks_{0}, startup_ticks_needed_{100};
  int cmd_counter_{0};
  const int cmd_throttle_{10};  // send arm/offboard every N ticks

  // ── Params ────────────────────────────────────────────────────────────
  double publish_rate_hz_, takeoff_height_, takeoff_threshold_;
  double startup_delay_s_, cmd_timeout_s_;

  // ── ROS handles ───────────────────────────────────────────────────────
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr  setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr      cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr            phase_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr   odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr     status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              complete_sub_;
  rclcpp::TimerBase::SharedPtr                                      timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetpointPublisher>());
  rclcpp::shutdown();
}