// planner_server_node.cpp
// ROS 2 action server that:
//   1. Loads a GlobalPlannerBase plugin (default: AStarPlanner)
//   2. Maintains a local copy of the Octomap
//   3. Serves NavigateToGoal action requests
//
// Action  : /uav/navigate_to_goal  (uav_planner_interface/action/NavigateToGoal)
// Subscribes: /octomap_binary      (octomap_msgs/Octomap)
// Publishes:  /uav/global_path     (nav_msgs/Path)  — for RViz2 visualisation

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/path.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <pluginlib/class_loader.hpp>

#include "uav_planner_interface/action/navigate_to_goal.hpp"
#include <uav_planner_interface/global_planner_base.hpp>

using NavigateToGoal = uav_planner_interface::action::NavigateToGoal;
using GoalHandle     = rclcpp_action::ServerGoalHandle<NavigateToGoal>;

class PlannerServerNode : public rclcpp::Node
{
public:
  PlannerServerNode()
  : Node("planner_server"),
    plugin_loader_("uav_planner_interface", "uav_planning::GlobalPlannerBase")
  {
    // ── Parameters ───────────────────────────────────────────────────
    auto pname = declare_parameter<std::string>("planner_plugin",
                   "uav_planning::AStarPlanner");
    uav_planning::PlannerConfig cfg;
    cfg.robot_radius      = declare_parameter<double>("robot_radius",      0.30);
    cfg.inflation_radius  = declare_parameter<double>("inflation_radius",  0.15);
    cfg.goal_tolerance    = declare_parameter<double>("goal_tolerance",    0.25);
    cfg.max_altitude      = declare_parameter<double>("max_altitude",      5.0);
    cfg.min_altitude      = declare_parameter<double>("min_altitude",      0.3);
    cfg.max_planning_time = declare_parameter<double>("max_planning_time", 10.0);
    // SPF direct mode — skip A*, emit one-pose path directly
    spf_direct_mode_ = declare_parameter<bool>("spf_direct_mode", false);

    // ── Load plugin ──────────────────────────────────────────────────
    // Defer initialize() call — shared_from_this() is invalid in constructor.
    // A one-shot timer fires immediately after the node is fully constructed.
    planner_ = plugin_loader_.createSharedInstance(pname);
    plugin_name_ = pname;
    plugin_cfg_  = cfg;
    init_timer_ = create_wall_timer(
        std::chrono::milliseconds(0),
        [this]() {
          planner_->initialize(shared_from_this(), plugin_cfg_);
          RCLCPP_INFO(get_logger(), "Loaded planner plugin: %s", plugin_name_.c_str());
          init_timer_->cancel();
        });

    // ── TF ───────────────────────────────────────────────────────────
    tf_buf_    = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listen_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_);

    // ── Octomap subscriber ───────────────────────────────────────────
    octomap_sub_ = create_subscription<octomap_msgs::msg::Octomap>(
        "/octomap_binary", rclcpp::QoS(1).reliable().durability_volatile(),
        std::bind(&PlannerServerNode::octomap_cb, this, std::placeholders::_1));

    // ── Path publisher (for RViz2) ────────────────────────────────────
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/uav/global_path", 10);

    // ── Action server ────────────────────────────────────────────────
    action_server_ = rclcpp_action::create_server<NavigateToGoal>(
        this, "/uav/navigate_to_goal",
        std::bind(&PlannerServerNode::handle_goal,   this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PlannerServerNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&PlannerServerNode::handle_accept, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "PlannerServer ready — action: /uav/navigate_to_goal");
  }

private:
  // ── Octomap callback ─────────────────────────────────────────────────
  void octomap_cb(const octomap_msgs::msg::Octomap::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(map_mutex_);
    auto* tree = dynamic_cast<octomap::OcTree*>(
        octomap_msgs::msgToMap(*msg));
    if (tree) {
      octree_.reset(tree);
      map_frame_ = msg->header.frame_id;
    }
  }

  // ── Action handlers ──────────────────────────────────────────────────
  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const NavigateToGoal::Goal> goal)
  {
    RCLCPP_INFO(get_logger(), "Received goal: (%.2f, %.2f, %.2f)",
        goal->target_pose.pose.position.x,
        goal->target_pose.pose.position.y,
        goal->target_pose.pose.position.z);
    if (!octree_) {
      RCLCPP_WARN(get_logger(), "No octomap yet — rejecting goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(get_logger(), "Goal cancelled");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accept(const std::shared_ptr<GoalHandle> gh)
  {
    // Run planning in a separate thread — never block the action server thread
    std::thread([this, gh]() { execute(gh); }).detach();
  }

  void execute(const std::shared_ptr<GoalHandle> gh)
  {
    auto result = std::make_shared<NavigateToGoal::Result>();
    auto fb     = std::make_shared<NavigateToGoal::Feedback>();

    // ── Get current drone pose as start ──────────────────────────────
    geometry_msgs::msg::PoseStamped start;
    try {
      auto tf = tf_buf_->lookupTransform(
          map_frame_, "base_link",
          rclcpp::Time(0, 0, RCL_ROS_TIME),
          rclcpp::Duration::from_seconds(1.0));
      start.header.frame_id = map_frame_;
      start.header.stamp    = get_clock()->now();
      start.pose.position.x = tf.transform.translation.x;
      start.pose.position.y = tf.transform.translation.y;
      start.pose.position.z = tf.transform.translation.z;
      start.pose.orientation = tf.transform.rotation;
    } catch (const tf2::TransformException& ex) {
      RCLCPP_ERROR(get_logger(), "Could not get drone pose: %s", ex.what());
      result->result_code = "INVALID_GOAL";
      gh->abort(result);
      return;
    }

    // ── Copy octree for thread safety ─────────────────────────────────
    std::shared_ptr<octomap::OcTree> tree_copy;
    {
      std::lock_guard<std::mutex> lk(map_mutex_);
      tree_copy = octree_;
    }

    // ── SPF direct mode — skip A*, emit one-pose path ────────────────
    if (spf_direct_mode_) {
      RCLCPP_INFO(get_logger(), "[SPF] Direct mode — skipping A*, emitting one-pose path");
      nav_msgs::msg::Path path;
      path.header.frame_id = map_frame_;
      path.header.stamp    = get_clock()->now();
      path.poses.push_back(gh->get_goal()->target_pose);
      path_pub_->publish(path);
      result->global_path  = path;
      result->result_code  = "SUCCESS";
      gh->succeed(result);
      RCLCPP_INFO(get_logger(), "[SPF] Direct path published — 1 pose");
      return;
    }

    // ── Call planner plugin ───────────────────────────────────────────
    auto feedback_cb = [&](float pct) {
      if (gh->is_canceling()) return;
      fb->percent_complete = pct;
      fb->status_msg = "Planning...";
      gh->publish_feedback(fb);
    };

    auto path = planner_->computePath(
        start, gh->get_goal()->target_pose, *tree_copy, feedback_cb);

    if (gh->is_canceling()) {
      result->result_code = "NO_PATH";
      gh->canceled(result);
      return;
    }

    if (path.poses.empty()) {
      result->result_code = "NO_PATH";
      gh->abort(result);
      RCLCPP_WARN(get_logger(), "Planning failed — no path found");
      return;
    }

    // ── Success ───────────────────────────────────────────────────────
    // ── Downsample path — keep one waypoint every ~1.0m ─────────────
    // Dense 0.2m voxel waypoints cause waypoint manager to race through
    // the path without accurately tracking position
    if (path.poses.size() > 2) {
      const double wp_spacing = 1.0;  // metres between waypoints
      std::vector<geometry_msgs::msg::PoseStamped> sparse;
      sparse.push_back(path.poses.front());
      for (size_t i = 1; i + 1 < path.poses.size(); ++i) {
        auto& prev = sparse.back().pose.position;
        auto& cur  = path.poses[i].pose.position;
        double d = std::sqrt(
            std::pow(cur.x-prev.x,2) +
            std::pow(cur.y-prev.y,2) +
            std::pow(cur.z-prev.z,2));
        if (d >= wp_spacing) sparse.push_back(path.poses[i]);
      }
      sparse.push_back(path.poses.back());  // always include goal
      RCLCPP_INFO(get_logger(), "Path downsampled: %zu → %zu waypoints",
          path.poses.size(), sparse.size());
      path.poses = sparse;
    }

    path_pub_->publish(path);
    result->global_path = path;
    result->result_code = "SUCCESS";
    gh->succeed(result);
    RCLCPP_INFO(get_logger(), "Path published — %zu waypoints",
                path.poses.size());
  }

  // Members
  pluginlib::ClassLoader<uav_planning::GlobalPlannerBase> plugin_loader_;
  uav_planning::GlobalPlannerBase::Ptr                    planner_;
  std::string                                             plugin_name_;
  uav_planning::PlannerConfig                             plugin_cfg_;
  rclcpp::TimerBase::SharedPtr                            init_timer_;

  std::shared_ptr<tf2_ros::Buffer>            tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listen_;

  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr           path_pub_;
  rclcpp_action::Server<NavigateToGoal>::SharedPtr            action_server_;

  bool spf_direct_mode_{false};
  std::shared_ptr<octomap::OcTree> octree_;
  std::string                      map_frame_{"map"};
  std::mutex                       map_mutex_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlannerServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}