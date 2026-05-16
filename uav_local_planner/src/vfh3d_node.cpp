// vfh3d_node.cpp
// ROS 2 node wrapping the VFH3D algorithm.
//
// Subscribes:
//   /octomap_binary           (octomap_msgs/Octomap)
//   /drone/odom               (nav_msgs/Odometry)
//   /uav/current_waypoint     (geometry_msgs/PointStamped)  ← from waypoint_manager
//
// Publishes:
//   /uav/cmd_vel              (geometry_msgs/TwistStamped)  → setpoint_publisher
//   /uav/vfh_status           (std_msgs/String)             → debug

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

#include <uav_local_planner/vfh3d.hpp>

class VFH3DNode : public rclcpp::Node
{
public:
  VFH3DNode() : Node("vfh3d_node")
  {
    // ── Parameters ───────────────────────────────────────────────────
    uav_local_planner::VFH3DConfig cfg;
    cfg.az_sectors     = declare_parameter("az_sectors",     72);
    cfg.el_sectors     = declare_parameter("el_sectors",     36);
    cfg.bbox_radius    = declare_parameter("bbox_radius",    3.0);
    cfg.robot_radius   = declare_parameter("robot_radius",   0.30);
    cfg.safety_radius  = declare_parameter("safety_radius",  0.15);
    cfg.h_high         = declare_parameter("h_high",         0.5);
    cfg.h_low          = declare_parameter("h_low",          0.2);
    cfg.w_goal         = declare_parameter("w_goal",         3.0);
    cfg.w_current      = declare_parameter("w_current",      1.0);
    cfg.w_prev         = declare_parameter("w_prev",         2.0);
    cfg.max_speed      = declare_parameter("max_speed",      1.0);
    cfg.min_speed      = declare_parameter("min_speed",      0.1);
    cfg.max_vz         = declare_parameter("max_vz",         0.5);
    cfg.min_clearance  = declare_parameter("min_clearance",  0.5);
    double rate_hz     = declare_parameter("update_rate_hz", 20.0);

    vfh_ = std::make_unique<uav_local_planner::VFH3D>(cfg);

    // ── Subscribers ──────────────────────────────────────────────────
    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
        "/octomap_binary",
        rclcpp::QoS(1).reliable().durability_volatile(),
        [this](const octomap_msgs::msg::Octomap::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(map_mutex_);
          auto* tree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
          if (tree) octree_.reset(tree);
        });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/drone/odom", rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(odom_mutex_);
          pos_ = Eigen::Vector3d(
              msg->pose.pose.position.x,
              msg->pose.pose.position.y,
              msg->pose.pose.position.z);
          vel_ = Eigen::Vector3d(
              msg->twist.twist.linear.x,
              msg->twist.twist.linear.y,
              msg->twist.twist.linear.z);
          have_odom_ = true;
        });

    waypoint_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/uav/current_waypoint", rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(wp_mutex_);
          waypoint_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
          have_waypoint_ = true;
          vfh_->reset();
        });

    // Listen for mission complete to reset IDLE state
    mission_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/uav/mission_complete", rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
            std::lock_guard<std::mutex> lk(wp_mutex_);
            have_waypoint_ = false;
            vfh_->reset();
          } else {
            // New goal incoming — re-enable
            std::lock_guard<std::mutex> lk(wp_mutex_);
            have_waypoint_ = false;  // will be set true when first wp arrives
          }
        });

    // ── Publishers ───────────────────────────────────────────────────
    cmd_pub_    = create_publisher<geometry_msgs::msg::TwistStamped>(
        "/uav/cmd_vel", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/uav/vfh_status", 10);

    // ── Main timer ───────────────────────────────────────────────────
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz),
        std::bind(&VFH3DNode::update, this));

    RCLCPP_INFO(get_logger(), "VFH3DNode ready at %.0f Hz", rate_hz);
  }

private:
  void update()
  {
    // Always publish status even when no waypoint active
    if (!have_odom_) return;

    std_msgs::msg::String status;

    if (!have_waypoint_) {
      status.data = "IDLE";
      status_pub_->publish(status);
      // Publish zero velocity to keep setpoint publisher happy
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.stamp    = get_clock()->now();
      cmd.header.frame_id = "map";
      cmd_pub_->publish(cmd);
      return;
    }

    std::shared_ptr<octomap::OcTree> tree;
    {
      std::lock_guard<std::mutex> lk(map_mutex_);
      tree = octree_;
    }
    if (!tree) return;

    Eigen::Vector3d pos, vel, wp;
    {
      std::lock_guard<std::mutex> lk(odom_mutex_);
      pos = pos_; vel = vel_;
    }
    {
      std::lock_guard<std::mutex> lk(wp_mutex_);
      wp = waypoint_;
    }

    // ── Run VFH3D ────────────────────────────────────────────────────
    auto result = vfh_->update(*tree, pos, wp, vel);

    // ── Publish cmd_vel ──────────────────────────────────────────────
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp    = get_clock()->now();
    cmd.header.frame_id = "map";  // velocity in map/ENU frame

    if (result.estop) {
      // Zero velocity — safety stop
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "E-STOP: obstacle within %.2fm",
          result.closest_obstacle_dist);
    } else {
      cmd.twist.linear.x = result.velocity.x();
      cmd.twist.linear.y = result.velocity.y();
      cmd.twist.linear.z = result.velocity.z();
    }
    cmd_pub_->publish(cmd);

    // ── Publish status ───────────────────────────────────────────────
    status.data = result.estop ? "ESTOP" :
                  result.obstacle_detected ? "AVOIDING" : "NOMINAL";
    status_pub_->publish(status);
  }

  std::unique_ptr<uav_local_planner::VFH3D> vfh_;

  std::shared_ptr<octomap::OcTree> octree_;
  Eigen::Vector3d pos_{0,0,0}, vel_{0,0,0}, waypoint_{0,0,0};
  bool have_odom_{false}, have_waypoint_{false};

  std::mutex map_mutex_, odom_mutex_, wp_mutex_;

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr  octomap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr           mission_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr    cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                status_pub_;
  rclcpp::TimerBase::SharedPtr                                       timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VFH3DNode>());
  rclcpp::shutdown();
}