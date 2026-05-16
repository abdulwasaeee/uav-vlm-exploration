// waypoint_manager_node.cpp
// Upgraded with SPF rotate-then-translate FSM.
// ROTATING phase: drone rotates to face target yaw
// TRANSLATING phase: drone moves to target XY+Z
//
// Subscribes:
//   /uav/global_path   (nav_msgs/Path)       ← from planner server
//   /drone/odom        (nav_msgs/Odometry)   ← drone pose
//
// Publishes:
//   /uav/current_waypoint  (geometry_msgs/PointStamped) → mp_node
//   /uav/mission_complete  (std_msgs/Bool)
//   /uav/mission_phase     (std_msgs/String) → IDLE|ROTATING|TRANSLATING|COMPLETE

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <uav_planner_interface/action/navigate_to_goal.hpp>
#include <cmath>

enum class SPFPhase { IDLE, ROTATING, TRANSLATING, COMPLETE };

static double normalizeAngle(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

static double quatToYaw(double x, double y, double z, double w) {
  return std::atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z));
}

class WaypointManagerNode : public rclcpp::Node
{
public:
  WaypointManagerNode() : Node("waypoint_manager")
  {
    acceptance_radius_       = declare_parameter("acceptance_radius",        0.4);
    final_acceptance_radius_ = declare_parameter("final_acceptance_radius",  0.25);
    replan_deviation_        = declare_parameter("replan_deviation",         2.0);
    enable_replan_           = declare_parameter("enable_replan",            false);
    publish_rate_hz_         = declare_parameter("publish_rate_hz",          20.0);
    yaw_threshold_deg_       = declare_parameter("yaw_alignment_threshold_deg", 15.0);
    yaw_hold_cycles_         = declare_parameter("yaw_alignment_hold_cycles",   10);

    yaw_threshold_rad_ = yaw_threshold_deg_ * M_PI / 180.0;

    // ── Subscribers ──────────────────────────────────────────────────
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/uav/global_path", rclcpp::QoS(1).reliable(),
        std::bind(&WaypointManagerNode::path_cb, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/drone/odom", rclcpp::QoS(10),
        std::bind(&WaypointManagerNode::odom_cb, this, std::placeholders::_1));

    // ── Publishers ───────────────────────────────────────────────────
    wp_pub_       = create_publisher<geometry_msgs::msg::PointStamped>(
        "/uav/current_waypoint", 10);
    complete_pub_ = create_publisher<std_msgs::msg::Bool>(
        "/uav/mission_complete", 10);
    phase_pub_    = create_publisher<std_msgs::msg::String>(
        "/uav/mission_phase", 10);

    // ── Action client for replanning ─────────────────────────────────
    action_client_ = rclcpp_action::create_client<uav_planner_interface::action::NavigateToGoal>(
        this->get_node_base_interface(),
        this->get_node_graph_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "/uav/navigate_to_goal");

    // ── Timer ────────────────────────────────────────────────────────
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_hz_),
        std::bind(&WaypointManagerNode::update, this));

    RCLCPP_INFO(get_logger(),
        "WaypointManager ready (SPF FSM) — yaw_threshold=%.1f° hold_cycles=%d",
        yaw_threshold_deg_, yaw_hold_cycles_);
  }

private:
  void path_cb(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) return;

    std_msgs::msg::Bool reset_msg;
    reset_msg.data = false;
    complete_pub_->publish(reset_msg);

    path_         = msg->poses;
    wp_idx_       = 0;
    mission_done_ = false;
    final_goal_   = path_.back().pose;

    // Initialise FSM for first waypoint
    initFSMForWaypoint(0);

    RCLCPP_INFO(get_logger(), "New path received — %zu waypoints", path_.size());
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    drone_x_   = msg->pose.pose.position.x;
    drone_y_   = msg->pose.pose.position.y;
    drone_z_   = msg->pose.pose.position.z;
    auto& q    = msg->pose.pose.orientation;
    drone_yaw_ = quatToYaw(q.x, q.y, q.z, q.w);
    have_odom_ = true;
  }

  void initFSMForWaypoint(size_t idx)
  {
    if (idx >= path_.size()) return;

    // Extract target yaw from path pose orientation
    auto& q       = path_[idx].pose.orientation;
    target_yaw_   = quatToYaw(q.x, q.y, q.z, q.w);
    yaw_tol_cnt_  = 0;

    double yaw_err = std::abs(normalizeAngle(target_yaw_ - drone_yaw_));
    if (yaw_err > yaw_threshold_rad_) {
      spf_phase_ = SPFPhase::ROTATING;
      RCLCPP_INFO(get_logger(),
          "WP %zu: yaw error %.1f° > threshold %.1f° — ROTATING first",
          idx, yaw_err * 180.0 / M_PI, yaw_threshold_deg_);
    } else {
      spf_phase_ = SPFPhase::TRANSLATING;
      RCLCPP_INFO(get_logger(),
          "WP %zu: yaw already aligned — TRANSLATING directly", idx);
    }
  }

  void publishPhase(const std::string& phase)
  {
    std_msgs::msg::String msg;
    msg.data = phase;
    phase_pub_->publish(msg);
  }

  void update()
  {
    if (path_.empty() || !have_odom_ || mission_done_) {
      publishPhase("IDLE");
      return;
    }

    auto& wp = path_[wp_idx_].pose.position;

    // ── SPF FSM ───────────────────────────────────────────────────────
    switch (spf_phase_) {

      case SPFPhase::ROTATING: {
        publishPhase("ROTATING");

        // Publish hover waypoint (current XY, target yaw encoded in orientation)
        // mp_node will hover — setpoint_publisher handles yaw via mission_phase
        geometry_msgs::msg::PointStamped pt;
        pt.header.stamp    = get_clock()->now();
        pt.header.frame_id = "map";
        pt.point.x = drone_x_;
        pt.point.y = drone_y_;
        pt.point.z = drone_z_;
        wp_pub_->publish(pt);

        // Check yaw alignment
        double yaw_err = std::abs(normalizeAngle(target_yaw_ - drone_yaw_));
        if (yaw_err < yaw_threshold_rad_) {
          yaw_tol_cnt_++;
          if (yaw_tol_cnt_ >= yaw_hold_cycles_) {
            spf_phase_ = SPFPhase::TRANSLATING;
            RCLCPP_INFO(get_logger(),
                "Yaw aligned (err=%.1f°) — switching to TRANSLATING",
                yaw_err * 180.0 / M_PI);
          }
        } else {
          yaw_tol_cnt_ = 0;
        }
        break;
      }

      case SPFPhase::TRANSLATING: {
        publishPhase("TRANSLATING");

        // Publish target waypoint
        geometry_msgs::msg::PointStamped pt;
        pt.header.stamp    = get_clock()->now();
        pt.header.frame_id = "map";
        pt.point.x = wp.x;
        pt.point.y = wp.y;
        pt.point.z = wp.z;
        wp_pub_->publish(pt);

        // Check if reached
        double dx   = drone_x_ - wp.x;
        double dy   = drone_y_ - wp.y;
        double dz   = drone_z_ - wp.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        bool is_final = (wp_idx_ + 1 >= path_.size());
        double radius = is_final ? final_acceptance_radius_ : acceptance_radius_;

        if (dist < radius) {
          if (wp_idx_ + 1 < path_.size()) {
            ++wp_idx_;
            RCLCPP_INFO(get_logger(), "Waypoint %zu/%zu reached — next",
                wp_idx_, path_.size());
            initFSMForWaypoint(wp_idx_);
          } else {
            spf_phase_    = SPFPhase::COMPLETE;
            mission_done_ = true;
            std_msgs::msg::Bool done;
            done.data = true;
            complete_pub_->publish(done);
            publishPhase("COMPLETE");
            RCLCPP_INFO(get_logger(), "Mission complete!");
          }
        }

        // Replan check
        if (enable_replan_) {
          double min_dist = std::numeric_limits<double>::max();
          for (auto& pose : path_) {
            double d = std::hypot(
                drone_x_ - pose.pose.position.x,
                drone_y_ - pose.pose.position.y);
            min_dist = std::min(min_dist, d);
          }
          if (min_dist > replan_deviation_) {
            RCLCPP_WARN(get_logger(),
                "Deviation %.2fm > %.2fm — requesting replan",
                min_dist, replan_deviation_);
            requestReplan();
          }
        }
        break;
      }

      case SPFPhase::COMPLETE:
        publishPhase("COMPLETE");
        break;

      case SPFPhase::IDLE:
      default:
        publishPhase("IDLE");
        break;
    }
  }

  void requestReplan()
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(1))) {
      RCLCPP_WARN(get_logger(), "Planner server not available for replan");
      return;
    }
    auto goal = uav_planner_interface::action::NavigateToGoal::Goal();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp    = get_clock()->now();
    goal.target_pose.pose            = final_goal_;
    goal.planning_timeout_sec        = 5.0f;
    action_client_->async_send_goal(goal);
  }

  // Path tracking
  std::vector<geometry_msgs::msg::PoseStamped> path_;
  geometry_msgs::msg::Pose final_goal_;
  size_t wp_idx_{0};
  bool   mission_done_{false};

  // SPF FSM
  SPFPhase spf_phase_{SPFPhase::IDLE};
  double   target_yaw_{0.0};
  int      yaw_tol_cnt_{0};

  // Drone pose
  double drone_x_{0}, drone_y_{0}, drone_z_{0}, drone_yaw_{0};
  bool   have_odom_{false};

  // Params
  double acceptance_radius_, final_acceptance_radius_;
  double replan_deviation_, publish_rate_hz_;
  double yaw_threshold_deg_, yaw_threshold_rad_;
  int    yaw_hold_cycles_;
  bool   enable_replan_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr        path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr wp_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              complete_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr            phase_pub_;
  rclcpp_action::Client<uav_planner_interface::action::NavigateToGoal>::SharedPtr
      action_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointManagerNode>());
  rclcpp::shutdown();
}
