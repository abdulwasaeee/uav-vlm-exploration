// waypoint_manager_node.cpp
// Tracks the global path from the planner server and feeds the next
// waypoint to the VFH3D local planner one at a time.
// Advances to the next waypoint when the drone is within acceptance_radius.
// Requests a replan if the drone deviates too far from the global path.
//
// Subscribes:
//   /uav/global_path   (nav_msgs/Path)       ← from planner server
//   /drone/odom        (nav_msgs/Odometry)   ← drone pose
//
// Publishes:
//   /uav/current_waypoint  (geometry_msgs/PointStamped) → vfh3d_node
//   /uav/mission_complete  (std_msgs/Bool)

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <uav_planner_interface/action/navigate_to_goal.hpp>

class WaypointManagerNode : public rclcpp::Node
{
public:
  WaypointManagerNode() : Node("waypoint_manager")
  {
    acceptance_radius_ = declare_parameter("acceptance_radius", 0.4);
    final_acceptance_radius_ = declare_parameter("final_acceptance_radius", 0.25);
    replan_deviation_  = declare_parameter("replan_deviation",  2.0);
    enable_replan_     = declare_parameter("enable_replan",     false);
    publish_rate_hz_   = declare_parameter("publish_rate_hz",   20.0);

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

    // ── Action client for replanning ─────────────────────────────────
    action_client_ = rclcpp_action::create_client<
        uav_planner_interface::action::NavigateToGoal>(
        this, "/uav/navigate_to_goal");

    // ── Timer ────────────────────────────────────────────────────────
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publish_rate_hz_),
        std::bind(&WaypointManagerNode::update, this));

    RCLCPP_INFO(get_logger(), "WaypointManager ready");
  }

private:
  void path_cb(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (msg->poses.empty()) return;

    // Publish reset first, then wait one cycle before activating path
    std_msgs::msg::Bool reset_msg;
    reset_msg.data = false;
    complete_pub_->publish(reset_msg);

    path_         = msg->poses;
    wp_idx_       = 0;
    mission_done_ = false;
    final_goal_   = path_.back().pose;

    RCLCPP_INFO(get_logger(), "New path received — %zu waypoints", path_.size());
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    drone_x_ = msg->pose.pose.position.x;
    drone_y_ = msg->pose.pose.position.y;
    drone_z_ = msg->pose.pose.position.z;
    have_odom_ = true;
  }

  void update()
  {
    if (path_.empty() || !have_odom_ || mission_done_) return;

    auto& wp = path_[wp_idx_].pose.position;
    double dx = drone_x_ - wp.x;
    double dy = drone_y_ - wp.y;
    double dz = drone_z_ - wp.z;
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

    // Advance waypoint index when within acceptance radius
    // Use tighter radius for final waypoint
    bool is_final = (wp_idx_ + 1 >= path_.size());
    double radius = is_final ? final_acceptance_radius_ : acceptance_radius_;

    if (dist < radius) {
      if (wp_idx_ + 1 < path_.size()) {
        ++wp_idx_;
        RCLCPP_INFO(get_logger(), "Waypoint %zu/%zu reached — next",
            wp_idx_, path_.size());
      } else {
        // Final waypoint reached
        mission_done_ = true;
        std_msgs::msg::Bool done;
        done.data = true;
        complete_pub_->publish(done);
        RCLCPP_INFO(get_logger(), "Mission complete!");
        return;
      }
    }

    // Check deviation from path — replan if too far off
    double min_dist = std::numeric_limits<double>::max();
    for (auto& pose : path_) {
      double d = std::hypot(
          drone_x_ - pose.pose.position.x,
          drone_y_ - pose.pose.position.y);
      min_dist = std::min(min_dist, d);
    }
    if (enable_replan_ && min_dist > replan_deviation_) {
      RCLCPP_WARN(get_logger(),
          "Deviation %.2fm > %.2fm — requesting replan", min_dist, replan_deviation_);
      requestReplan();
    }

    // Publish current waypoint
    auto& cur_wp = path_[wp_idx_].pose.position;
    geometry_msgs::msg::PointStamped pt;
    pt.header.stamp    = get_clock()->now();
    pt.header.frame_id = "map";
    pt.point.x = cur_wp.x;
    pt.point.y = cur_wp.y;
    pt.point.z = cur_wp.z;
    wp_pub_->publish(pt);
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

  // Drone pose
  double drone_x_{0}, drone_y_{0}, drone_z_{0};
  bool   have_odom_{false};

  // Params
  double acceptance_radius_, final_acceptance_radius_, replan_deviation_, publish_rate_hz_;
  bool   enable_replan_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr        path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr wp_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr              complete_pub_;
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