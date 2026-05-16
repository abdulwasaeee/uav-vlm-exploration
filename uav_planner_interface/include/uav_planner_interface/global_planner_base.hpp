#pragma once
// global_planner_base.hpp
// Pure abstract interface for all global planner plugins.
// To add a new planner (RRT*, ML, etc.), inherit this class,
// implement computePath(), and register via pluginlib.

#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <octomap/octomap.h>
#include <rclcpp/rclcpp.hpp>
#include <functional>

namespace uav_planning {

struct PlannerConfig {
  double robot_radius      = 0.30;
  double inflation_radius  = 0.15;
  double goal_tolerance    = 0.25;
  double max_altitude      = 5.0;
  double min_altitude      = 0.3;
  double max_planning_time = 10.0;  // seconds — A* timeout
};

class GlobalPlannerBase
{
public:
  using Ptr = std::shared_ptr<GlobalPlannerBase>;

  // Feedback callback: called periodically during planning with 0.0–1.0 progress
  using FeedbackCb = std::function<void(float)>;

  virtual ~GlobalPlannerBase() = default;

  // Called once by the planner server after construction
  virtual void initialize(rclcpp::Node::SharedPtr node, const PlannerConfig& cfg) = 0;

  // Main planning call — blocking, runs in action server thread
  // Returns an empty path on failure
  virtual nav_msgs::msg::Path computePath(
      const geometry_msgs::msg::PoseStamped& start,
      const geometry_msgs::msg::PoseStamped& goal,
      const octomap::OcTree& octree,
      const FeedbackCb& feedback_cb) = 0;

  // Human-readable name for logging
  virtual std::string name() const = 0;
};

}  // namespace uav_planning