// frontier_explorer_node.cpp
// Frontier-based autonomous exploration for UAV.
//
// Algorithm:
//   1. Scan OctoMap for frontier voxels (free adjacent to unknown)
//   2. Cluster frontiers spatially (simple radius clustering)
//   3. Score clusters by size and distance — hooks for VLM weights later
//   4. Send best frontier as goal to /uav/navigate_to_goal
//   5. On success/failure → repeat. Stop when no frontiers remain.
//
// Subscribes:
//   /octomap_binary        (octomap_msgs/Octomap)
//   /drone/odom            (nav_msgs/Odometry)
//   /uav/mission_complete  (std_msgs/Bool)
//
// Publishes:
//   /uav/exploration_frontiers  (visualization_msgs/MarkerArray) — RViz2
//   /uav/exploration_status     (std_msgs/String)

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <uav_planner_interface/action/navigate_to_goal.hpp>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <random>

using NavigateToGoal = uav_planner_interface::action::NavigateToGoal;
using GoalHandleNav  = rclcpp_action::ClientGoalHandle<NavigateToGoal>;

// ── Frontier cluster ──────────────────────────────────────────────────
struct FrontierCluster {
  Eigen::Vector3d centroid;
  int             size;        // number of frontier voxels
  double          score;       // higher = better to visit next
};

class FrontierExplorer : public rclcpp::Node
{
public:
  FrontierExplorer() : Node("frontier_explorer")
  {
    // ── Parameters ───────────────────────────────────────────────────
    // Map bounds — from your SDF
    map_xmin_ = declare_parameter("map_xmin", -10.0);
    map_xmax_ = declare_parameter("map_xmax",  10.0);
    map_ymin_ = declare_parameter("map_ymin", -10.0);
    map_ymax_ = declare_parameter("map_ymax",  10.0);
    map_zmin_ = declare_parameter("map_zmin",   0.3);  // min fly height
    map_zmax_ = declare_parameter("map_zmax",   4.0);  // below hanging beam

    // Exploration altitudes — two layers
    explore_altitudes_ = declare_parameter(
        "explore_altitudes", std::vector<double>{1.5, 3.5});
    current_alt_idx_   = 0;

    // Frontier params
    min_frontier_size_   = declare_parameter("min_frontier_size",   5);
    cluster_radius_      = declare_parameter("cluster_radius",       1.0);
    scan_interval_s_     = declare_parameter("scan_interval_s",      3.0);
    goal_timeout_s_      = declare_parameter("goal_timeout_s",      20.0);
    min_goal_distance_   = declare_parameter("min_goal_distance",    1.0);

    // Score weights — these are the VLM extension points
    w_size_     = declare_parameter("w_size",     1.0);   // reward large frontiers
    w_distance_ = declare_parameter("w_distance", 0.5);   // penalise far frontiers
    w_altitude_ = declare_parameter("w_altitude", 0.3);   // reward current altitude layer

    // ── Action client ─────────────────────────────────────────────────
    action_client_ = rclcpp_action::create_client<NavigateToGoal>(
        this, "/uav/navigate_to_goal");

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
          std::lock_guard<std::mutex> lk(pose_mutex_);
          drone_pos_ = Eigen::Vector3d(
              msg->pose.pose.position.x,
              msg->pose.pose.position.y,
              msg->pose.pose.position.z);
          have_pose_ = true;
        });

    complete_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/uav/mission_complete", rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg->data) {
            goal_active_ = false;
            RCLCPP_INFO(get_logger(), "Goal reached — scanning for next frontier");
          }
        });

    // ── Publishers ───────────────────────────────────────────────────
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/uav/exploration_frontiers", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(
        "/uav/exploration_status", 10);

    // ── Trajectory publisher ─────────────────────────────────────────
    trajectory_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/uav/trajectory", 10);
    trajectory_.header.frame_id = "map";

    // Record pose at 2 Hz
    traj_timer_ = create_wall_timer(
        std::chrono::milliseconds(500),
        [this]() {
          if (!have_pose_) return;
          std::lock_guard<std::mutex> lk(pose_mutex_);
          geometry_msgs::msg::PoseStamped ps;
          ps.header.stamp    = get_clock()->now();
          ps.header.frame_id = "map";
          ps.pose.position.x = drone_pos_.x();
          ps.pose.position.y = drone_pos_.y();
          ps.pose.position.z = drone_pos_.z();
          ps.pose.orientation.w = 1.0;
          trajectory_.poses.push_back(ps);
          trajectory_.header.stamp = ps.header.stamp;
          trajectory_pub_->publish(trajectory_);
        });
        timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(scan_interval_s_ * 1000)),
        std::bind(&FrontierExplorer::explorationStep, this));

    RCLCPP_INFO(get_logger(),
        "FrontierExplorer ready — altitudes: %.1fm, %.1fm",
        explore_altitudes_[0], explore_altitudes_[1]);
  }

private:
  // ── Main exploration step ─────────────────────────────────────────────
  void explorationStep()
  {
    if (!have_pose_ || goal_active_) return;

    // ── Bootstrap phase — seed map before frontier search ─────────────
    if (bootstrapping_) {
      if (bootstrap_idx_ < static_cast<int>(bootstrap_waypoints_.size())) {
        if (!goal_active_) {
          auto& wp = bootstrap_waypoints_[bootstrap_idx_];
          RCLCPP_INFO(get_logger(),
              "Bootstrap %d/%zu — flying to (%.1f, %.1f, %.1f)",
              bootstrap_idx_+1, bootstrap_waypoints_.size(),
              wp.x(), wp.y(), wp.z());
          sendGoal(wp);
          ++bootstrap_idx_;
        }
        return;
      }
      bootstrapping_ = false;
      failed_goals_.clear();  // clear any bootstrap failures
      RCLCPP_INFO(get_logger(), "Bootstrap complete — starting frontier exploration");
    }

    std::shared_ptr<octomap::OcTree> tree;
    {
      std::lock_guard<std::mutex> lk(map_mutex_);
      tree = octree_;
    }
    if (!tree) {
      publishStatus("WAITING_FOR_MAP");
      return;
    }

    Eigen::Vector3d pos;
    {
      std::lock_guard<std::mutex> lk(pose_mutex_);
      pos = drone_pos_;
    }

    // ── 1. Extract frontier voxels ─────────────────────────────────────
    auto frontiers = extractFrontiers(*tree);
    RCLCPP_INFO(get_logger(), "Found %zu frontier voxels", frontiers.size());

    if (frontiers.empty()) {
      // Try next altitude layer before declaring done
      if (current_alt_idx_ + 1 < static_cast<int>(explore_altitudes_.size())) {
        ++current_alt_idx_;
        RCLCPP_INFO(get_logger(),
            "Layer %d complete — switching to altitude %.1fm",
            current_alt_idx_, explore_altitudes_[current_alt_idx_]);
        publishStatus("SWITCHING_LAYER");
        return;
      }
      RCLCPP_INFO(get_logger(), "Exploration complete — no more frontiers");
      publishStatus("COMPLETE");
      timer_->cancel();
      return;
    }

    // ── 2. Cluster frontiers ───────────────────────────────────────────
    auto clusters = clusterFrontiers(frontiers, tree->getResolution());
    RCLCPP_INFO(get_logger(), "Clustered into %zu frontier regions", clusters.size());

    // ── 3. Score clusters ──────────────────────────────────────────────
    double target_z = explore_altitudes_[current_alt_idx_];
    for (auto& c : clusters) {
      double dist       = (c.centroid - pos).norm();
      double alt_penalty = std::abs(c.centroid.z() - target_z);
      // Reward far frontiers — drives the drone across the room in big jumps
      // rather than creeping outward incrementally.
      // VLM extension: replace w_distance * dist with a semantic term
      c.score = w_size_     * static_cast<double>(c.size) / 10.0
              + w_distance_ * dist          // + = reward distance
              - w_altitude_ * alt_penalty;
    }

    // Sort descending by score
    std::sort(clusters.begin(), clusters.end(),
        [](const FrontierCluster& a, const FrontierCluster& b) {
          return a.score > b.score;
        });

    // ── 4. Publish frontier markers ────────────────────────────────────
    publishFrontierMarkers(clusters);

    // ── 5. Pick best reachable frontier ───────────────────────────────
    Eigen::Vector3d goal_pos;
    bool found = false;
    for (auto& c : clusters) {
      double dist = (c.centroid - pos).norm();
      if (dist < min_goal_distance_) continue;
      if (isRecentlyFailed(c.centroid)) continue;

      // Clamp to safe map bounds
      goal_pos = Eigen::Vector3d(
          std::clamp(c.centroid.x(), map_xmin_ + 1.0, map_xmax_ - 1.0),
          std::clamp(c.centroid.y(), map_ymin_ + 1.0, map_ymax_ - 1.0),
          target_z);
      found = true;
      break;
    }

    if (!found) {
      RCLCPP_WARN(get_logger(), "No reachable frontier found — waiting");
      publishStatus("WAITING");
      return;
    }

    // ── 6. Send goal ───────────────────────────────────────────────────
    sendGoal(goal_pos);
  }

  // ── Extract frontier voxels from OctoMap ─────────────────────────────
  // A frontier voxel is: FREE and has at least one UNKNOWN neighbour
  std::vector<Eigen::Vector3d> extractFrontiers(const octomap::OcTree& tree)
  {
    std::vector<Eigen::Vector3d> frontiers;
    double res = tree.getResolution();
    double target_z = explore_altitudes_[current_alt_idx_];
    double z_band   = 1.5;  // metres above/below target altitude to search

    // Iterate over known free leaf nodes within altitude band
    for (auto it = tree.begin_leafs(); it != tree.end_leafs(); ++it) {
      // Only consider free voxels
      if (tree.isNodeOccupied(*it)) continue;

      double z = it.getZ();
      if (z < target_z - z_band || z > target_z + z_band) continue;
      if (z < map_zmin_ || z > map_zmax_) continue;

      double x = it.getX(), y = it.getY();
      if (x < map_xmin_ || x > map_xmax_) continue;
      if (y < map_ymin_ || y > map_ymax_) continue;

      // Check full 26-connected neighbourhood
      // Frontier = free + has unknown neighbour + no occupied neighbour
      bool is_frontier = false;
      bool too_close_to_obstacle = false;

      const std::array<std::array<int,3>, 26> nbrs26 = {{
        {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1},
        {1,1,0},{1,-1,0},{-1,1,0},{-1,-1,0},
        {1,0,1},{1,0,-1},{-1,0,1},{-1,0,-1},
        {0,1,1},{0,1,-1},{0,-1,1},{0,-1,-1},
        {1,1,1},{1,1,-1},{1,-1,1},{1,-1,-1},
        {-1,1,1},{-1,1,-1},{-1,-1,1},{-1,-1,-1}
      }};

      for (auto& d : nbrs26) {
        octomap::point3d nb(x+d[0]*res, y+d[1]*res, z+d[2]*res);
        auto* nb_node = tree.search(nb);
        if (!nb_node) {
          is_frontier = true;
        } else if (tree.isNodeOccupied(*nb_node)) {
          too_close_to_obstacle = true;
          break;
        }
      }

      if (is_frontier && !too_close_to_obstacle)
        frontiers.emplace_back(x, y, z);
    }
    return frontiers;
  }

  // ── Cluster frontier voxels by proximity ─────────────────────────────
  // Simple greedy radius clustering — O(n²) but fine for typical frontier counts
  std::vector<FrontierCluster> clusterFrontiers(
      const std::vector<Eigen::Vector3d>& pts, double /*res*/)
  {
    std::vector<bool> assigned(pts.size(), false);
    std::vector<FrontierCluster> clusters;

    for (size_t i = 0; i < pts.size(); ++i) {
      if (assigned[i]) continue;

      FrontierCluster c;
      c.centroid = pts[i];
      c.size     = 1;
      assigned[i] = true;

      // Grow cluster
      for (size_t j = i+1; j < pts.size(); ++j) {
        if (assigned[j]) continue;
        if ((pts[j] - c.centroid).norm() < cluster_radius_) {
          // Update centroid incrementally
          c.centroid = (c.centroid * c.size + pts[j]) / (c.size + 1);
          ++c.size;
          assigned[j] = true;
        }
      }

      if (c.size >= min_frontier_size_)
        clusters.push_back(c);
    }
    return clusters;
  }

  // ── Send navigation goal ──────────────────────────────────────────────
  void sendGoal(const Eigen::Vector3d& pos)
  {
    if (!action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "Action server not available");
      return;
    }

    auto goal = NavigateToGoal::Goal();
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp    = get_clock()->now();
    goal.target_pose.pose.position.x = pos.x();
    goal.target_pose.pose.position.y = pos.y();
    goal.target_pose.pose.position.z = pos.z();
    goal.target_pose.pose.orientation.w = 1.0;
    goal.planning_timeout_sec = static_cast<float>(goal_timeout_s_);

    auto opts = rclcpp_action::Client<NavigateToGoal>::SendGoalOptions();
    opts.result_callback = [this, pos](const GoalHandleNav::WrappedResult& res) {
      goal_active_ = false;
      if (res.result->result_code == "SUCCESS") {
        RCLCPP_INFO(get_logger(), "Frontier reached (%.2f, %.2f, %.2f)",
            pos.x(), pos.y(), pos.z());
      } else {
        RCLCPP_WARN(get_logger(), "Goal failed: %s — blacklisting",
            res.result->result_code.c_str());
        addFailedGoal(pos);
      }
    };

    goal_active_ = true;
    current_goal_ = pos;
    action_client_->async_send_goal(goal, opts);

    RCLCPP_INFO(get_logger(),
        "Exploring frontier at (%.2f, %.2f, %.2f) — layer %d/2",
        pos.x(), pos.y(), pos.z(), current_alt_idx_+1);
    publishStatus("EXPLORING");
  }

  // ── Failed goal blacklist ─────────────────────────────────────────────
  void addFailedGoal(const Eigen::Vector3d& pos)
  {
    failed_goals_.push_back(pos);
    // Keep list bounded
    if (failed_goals_.size() > 20)
      failed_goals_.erase(failed_goals_.begin());
  }

  bool isRecentlyFailed(const Eigen::Vector3d& pos) const
  {
    for (auto& f : failed_goals_)
      if ((f - pos).norm() < cluster_radius_ * 1.5) return true;
    return false;
  }

  // ── Publish frontier markers for RViz2 ───────────────────────────────
  void publishFrontierMarkers(const std::vector<FrontierCluster>& clusters)
  {
    visualization_msgs::msg::MarkerArray arr;

    // Clear previous markers
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    for (size_t i = 0; i < clusters.size(); ++i) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "map";
      m.header.stamp    = get_clock()->now();
      m.ns              = "frontiers";
      m.id              = static_cast<int>(i);
      m.type            = visualization_msgs::msg::Marker::SPHERE;
      m.action          = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = clusters[i].centroid.x();
      m.pose.position.y = clusters[i].centroid.y();
      m.pose.position.z = clusters[i].centroid.z();
      m.pose.orientation.w = 1.0;

      // Size proportional to cluster size
      double s = std::clamp(clusters[i].size / 20.0, 0.2, 1.0);
      m.scale.x = m.scale.y = m.scale.z = s;

      // Best frontier = green, others = cyan
      if (i == 0) {
        m.color.r = 0.0f; m.color.g = 1.0f;
        m.color.b = 0.0f; m.color.a = 0.9f;
      } else {
        m.color.r = 0.0f; m.color.g = 0.8f;
        m.color.b = 0.8f; m.color.a = 0.6f;
      }
      m.lifetime = rclcpp::Duration::from_seconds(scan_interval_s_ * 2);
      arr.markers.push_back(m);
    }
    marker_pub_->publish(arr);
  }

  void publishStatus(const std::string& s)
  {
    std_msgs::msg::String msg;
    msg.data = s;
    status_pub_->publish(msg);
  }

  // ── Members ───────────────────────────────────────────────────────────
  std::shared_ptr<octomap::OcTree> octree_;
  Eigen::Vector3d drone_pos_{0,0,0}, current_goal_{0,0,0};
  bool have_pose_{false}, goal_active_{false};
  std::mutex map_mutex_, pose_mutex_;

  std::vector<Eigen::Vector3d> failed_goals_;
  std::vector<double> explore_altitudes_;
  int current_alt_idx_{0};

  // Bootstrap
  std::vector<Eigen::Vector3d> bootstrap_waypoints_;
  int  bootstrap_idx_{0};
  bool bootstrapping_{true};

  // Params
  double map_xmin_, map_xmax_, map_ymin_, map_ymax_, map_zmin_, map_zmax_;
  int    min_frontier_size_;
  double cluster_radius_, scan_interval_s_, goal_timeout_s_;
  double min_goal_distance_;
  double w_size_, w_distance_, w_altitude_;

  rclcpp_action::Client<NavigateToGoal>::SharedPtr action_client_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr    odom_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr        complete_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                status_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                  trajectory_pub_;
  nav_msgs::msg::Path                                                trajectory_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr traj_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierExplorer>());
  rclcpp::shutdown();
}