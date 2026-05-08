// mp_node.cpp
// ROS 2 node wrapping the Motion Primitive Library local planner.
// Drop-in replacement for vfh3d_node — identical I/O topics.
//
// Subscribes:
//   /drone/tof_merged/points   (sensor_msgs/PointCloud2)
//   /drone/odom                (nav_msgs/Odometry)
//   /uav/current_waypoint      (geometry_msgs/PointStamped)
//   /uav/mission_complete      (std_msgs/Bool)
//
// Publishes:
//   /uav/cmd_vel               (geometry_msgs/TwistStamped)
//   /uav/vfh_status            (std_msgs/String)

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <uav_local_planner/motion_primitives.hpp>

#include <Eigen/Dense>
#include <deque>
#include <mutex>
#include <cmath>

// ── Orbit detector ──────────────────────────────────────────────────────────
// Detects circular motion without goal progress by accumulating yaw change.
// Complements the stall detector: stall catches lack of progress, orbit
// catches spinning-in-place specifically.  Both trigger a safety hover.

namespace {
double normalizeAngle(double a) {
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace

struct OrbitDetector {
  double yaw_threshold  = 1.5 * M_PI;  // fire after 270° of accumulated yaw
  double progress_min   = 0.3;         // must get this much closer than best-ever to reset
  double ignore_radius  = 1.2;         // skip check if already this close to goal
  double yaw_accumulator{0.0};
  double min_dist{1e9};                // best-ever distance — resets only on true progress
  double prev_yaw_{0.0};
  bool   prev_valid_{false};
  bool   orbiting{false};

  void reset() {
    yaw_accumulator = 0.0; min_dist = 1e9; prev_valid_ = false; orbiting = false;
  }

  void update(double dist_to_goal, double yaw)
  {
    orbiting = false;
    if (dist_to_goal < ignore_radius) { reset(); return; }
    if (!prev_valid_) { prev_yaw_ = yaw; prev_valid_ = true; return; }

    double dyaw = std::abs(normalizeAngle(yaw - prev_yaw_));
    prev_yaw_ = yaw;
    yaw_accumulator += dyaw;

    // Compute progress BEFORE updating min_dist so the delta is non-zero.
    // (Bug fix V3: previous code updated min_dist first, making progress always 0.)
    double progress = min_dist - dist_to_goal;
    if (dist_to_goal < min_dist) min_dist = dist_to_goal;

    if (progress > progress_min)
      yaw_accumulator = 0.0;

    orbiting = (yaw_accumulator > yaw_threshold);
  }
};

// ── Goal-progress stall detector ──────────────────────────────────────────
// Tracks the best-ever (minimum) distance to goal since the last reset.
// STALLED fires when that best distance has not improved by progress_min
// within no_improve_window seconds.
//
// Why the old sliding-window design failed:
//   When the drone oscillates near an obstacle (e.g. reaching 0.12 m then
//   bouncing to 1.75 m repeatedly), each 3-second window sees a fresh approach
//   that looks like 1.2 m of "progress".  initial_dist_in_window - min = 1.2 m
//   > 0.3 m → never stalled, even after 60 s of bouncing.
//
//   Additionally, stall_ignore_radius = 1.0 m suppressed the detector for 92%
//   of all samples when the drone oscillated below 1.0 m.
//
// New design: track global best_dist since last reset.  The clock resets only
// when the drone beats best_dist by progress_min.  Bouncing between 0.12 m and
// 1.75 m never beats 0.12 m − 0.15 m = −0.03 m, so the clock runs to timeout.
struct StallDetector {
  double no_improve_window = 20.0;  // fire after this many seconds without improvement
  double progress_min      = 0.15;  // dist reduction required to reset the clock (m)
  double ignore_radius     = 0.25;  // suppress when already this close to goal (m)
  bool   stalled           = false;

  double bestDist()        const { return best_dist_;      }
  double sinceImprove()    const { return since_improve_;  }

  void reset() { best_dist_ = 1e9; last_improve_t_ = -1.0; since_improve_ = 0.0; stalled = false; }

  void update(double dist_to_goal, rclcpp::Time now)
  {
    if (dist_to_goal < ignore_radius) { reset(); return; }

    const double t = now.seconds();
    if (last_improve_t_ < 0.0) {
      best_dist_      = dist_to_goal;
      last_improve_t_ = t;
      stalled         = false;
      return;
    }

    if (dist_to_goal < best_dist_ - progress_min) {
      best_dist_      = dist_to_goal;
      last_improve_t_ = t;
    }

    since_improve_ = t - last_improve_t_;
    stalled        = (since_improve_ > no_improve_window);
  }

private:
  double best_dist_     {1e9};
  double last_improve_t_{-1.0};
  double since_improve_ {0.0};
};

// ─────────────────────────────────────────────────────────────────────────
class MPNode : public rclcpp::Node
{
public:
  MPNode() : Node("mp_node")
  {
    // ── Parameters ──────────────────────────────────────────────────────
    uav_local_planner::MPConfig cfg;

    // Arc primitives
    cfg.num_az_horizontal = declare_parameter("num_az_horizontal", 18);
    cfg.num_curvatures    = declare_parameter("num_curvatures",     5);
    cfg.max_curvature     = declare_parameter("max_curvature",      0.4);
    cfg.arc_length        = declare_parameter("arc_length",         2.0);

    // Pitched layers
    cfg.elevation_angles_deg = declare_parameter(
        "elevation_angles_deg", std::vector<double>{30.0, 15.0, -15.0, -30.0});
    cfg.num_az_pitched    = declare_parameter("num_az_pitched", 18);

    // Collision
    cfg.collision_radius  = declare_parameter("collision_radius", 0.45);
    cfg.min_clearance     = declare_parameter("min_clearance",    0.6);

    // Speed
    cfg.max_speed         = declare_parameter("max_speed",  2.5);
    cfg.min_speed         = declare_parameter("min_speed",  0.2);
    cfg.max_vz            = declare_parameter("max_vz",     1.0);
    cfg.alt_kp            = declare_parameter("alt_kp",     0.8);

    // Scoring
    cfg.w_goal            = declare_parameter("w_goal", 3.0);
    cfg.w_prev            = declare_parameter("w_prev", 2.0);
    cfg.w_pos             = declare_parameter("w_pos",  2.0);

    // Obstacle detection persistence
    cfg.obstacle_hysteresis_factor = declare_parameter("obstacle_hysteresis_factor", 1.3);
    cfg.bypass_min_hold_cycles     = declare_parameter("bypass_min_hold_cycles",     60);

    // History buffer
    cfg.history_capacity  = declare_parameter("history_capacity",  2048);
    cfg.history_subsample = declare_parameter("history_subsample", 4);

    // Stall detection
    stall_.no_improve_window = declare_parameter("stall_no_improve_window", 20.0);
    stall_.progress_min      = declare_parameter("stall_progress_min",       0.15);
    stall_.ignore_radius     = declare_parameter("stall_ignore_radius",      0.25);

    // Orbit detection
    orbit_.yaw_threshold  = declare_parameter("orbit_yaw_threshold",  1.5 * M_PI);
    orbit_.progress_min   = declare_parameter("orbit_progress_min",   0.3);
    orbit_.ignore_radius  = declare_parameter("orbit_ignore_radius",  0.3);

    double rate_hz = declare_parameter("update_rate_hz", 20.0);

    planner_ = std::make_unique<uav_local_planner::MotionPrimitives>(cfg);

    RCLCPP_INFO(get_logger(),
        "MPNode: %d horiz arcs (%d az × %d κ) + %d pitched segments, %.0f Hz | "
        "stall: %.0fs/%.2fm/%.2fm | orbit: %.0f°/%.2fm/%.2fm",
        planner_->numHorizPrims(),
        cfg.num_az_horizontal, cfg.num_curvatures,
        planner_->numPitchedPrims(),
        rate_hz,
        stall_.no_improve_window, stall_.progress_min, stall_.ignore_radius,
        orbit_.yaw_threshold * 180.0 / M_PI, orbit_.progress_min, orbit_.ignore_radius);

    // ── Subscribers ──────────────────────────────────────────────────────
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/drone/tof_merged/points",
        rclcpp::QoS(1).reliable().durability_volatile(),
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
          auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
          pcl::fromROSMsg(*msg, *cloud);
          std::lock_guard<std::mutex> lk(cloud_mutex_);
          cloud_ = cloud;
        });

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/drone/odom", rclcpp::QoS(10),
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(odom_mutex_);
          pos_ = Eigen::Vector3d(
              msg->pose.pose.position.x,
              msg->pose.pose.position.y,
              msg->pose.pose.position.z);
          auto& q = msg->pose.pose.orientation;
          Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
          yaw_ = std::atan2(
              2.0 * (quat.w() * quat.z() + quat.x() * quat.y()),
              1.0 - 2.0 * (quat.y() * quat.y() + quat.z() * quat.z()));
          have_odom_ = true;
        });

    waypoint_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
        "/uav/current_waypoint", rclcpp::QoS(10),
        [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(wp_mutex_);
          waypoint_ = Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z);
          have_waypoint_ = true;
          planner_->reset();
          stall_.reset();   // new waypoint clears stall state
          orbit_.reset();   // new waypoint clears orbit state
        });

    mission_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/uav/mission_complete", rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          std::lock_guard<std::mutex> lk(wp_mutex_);
          have_waypoint_ = false;
          if (msg->data) { planner_->reset(); stall_.reset(); orbit_.reset(); }
        });

    // ── Publishers ───────────────────────────────────────────────────────
    cmd_pub_    = create_publisher<geometry_msgs::msg::TwistStamped>("/uav/cmd_vel", 10);
    status_pub_ = create_publisher<std_msgs::msg::String>("/uav/vfh_status", 10);
    diag_pub_   = create_publisher<std_msgs::msg::Float64MultiArray>("/uav/mp_diag", 10);

    // ── Timer ────────────────────────────────────────────────────────────
    timer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / rate_hz),
        std::bind(&MPNode::update, this));
  }

private:
  void update()
  {
    if (!have_odom_) return;

    std_msgs::msg::String status;

    if (!have_waypoint_) {
      status.data = "IDLE";
      status_pub_->publish(status);
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.stamp    = get_clock()->now();
      cmd.header.frame_id = "map";
      cmd_pub_->publish(cmd);
      return;
    }

    // ── Stall check ──────────────────────────────────────────────────────
    Eigen::Vector3d pos, wp;
    double yaw;
    {
      std::lock_guard<std::mutex> lk(odom_mutex_);
      pos = pos_; yaw = yaw_;
    }
    {
      std::lock_guard<std::mutex> lk(wp_mutex_);
      wp = waypoint_;
    }

    const double dist_to_goal = (pos.head<2>() - wp.head<2>()).norm();
    stall_.update(dist_to_goal, get_clock()->now());

    if (stall_.stalled) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "STALLED: best approach %.2fm, no %.2fm improvement for %.0fs — hovering. "
          "Send a new waypoint to resume.",
          stall_.bestDist(), stall_.progress_min, stall_.sinceImprove());
      status.data = "STALLED";
      status_pub_->publish(status);
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.stamp    = get_clock()->now();
      cmd.header.frame_id = "map";
      cmd_pub_->publish(cmd);
      std_msgs::msg::Float64MultiArray diag;
      diag.data = {dist_to_goal, yaw, orbit_.yaw_accumulator,
                   0.0, 1.0, 0.0, stall_.bestDist(), stall_.sinceImprove(), 0.0, 0.0, 0.0, 0.0};
      diag_pub_->publish(diag);
      return;
    }

    // ── Orbit check ──────────────────────────────────────────────────────
    orbit_.update(dist_to_goal, yaw);
    if (orbit_.orbiting) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "ORBIT DETECTED: accumulated %.0f° yaw change with < %.2fm goal progress — hovering. "
          "Send a new waypoint to resume.",
          orbit_.yaw_accumulator * 180.0 / M_PI, orbit_.progress_min);
      status.data = "ORBITING";
      status_pub_->publish(status);
      geometry_msgs::msg::TwistStamped cmd;
      cmd.header.stamp    = get_clock()->now();
      cmd.header.frame_id = "map";
      cmd_pub_->publish(cmd);
      std_msgs::msg::Float64MultiArray diag;
      diag.data = {dist_to_goal, yaw, orbit_.yaw_accumulator,
                   1.0, 0.0, 0.0, stall_.bestDist(), stall_.sinceImprove(), 0.0, 0.0, 0.0, 0.0};
      diag_pub_->publish(diag);
      return;
    }

    // ── Cloud check ──────────────────────────────────────────────────────
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud;
    {
      std::lock_guard<std::mutex> lk(cloud_mutex_);
      cloud = cloud_;
    }
    if (!cloud) return;

    // ── Run planner ──────────────────────────────────────────────────────
    auto result = planner_->update(*cloud, pos, yaw, wp);

    // ── Publish cmd_vel ──────────────────────────────────────────────────
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp    = get_clock()->now();
    cmd.header.frame_id = "map";

    if (result.estop) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "E-STOP: obstacle within %.2fm", result.closest_obstacle_dist);
    } else {
      cmd.twist.linear.x = result.velocity.x();
      cmd.twist.linear.y = result.velocity.y();
      cmd.twist.linear.z = result.velocity.z();
    }
    cmd_pub_->publish(cmd);

    // ── Publish status ───────────────────────────────────────────────────
    auto bs = planner_->bypassState();
    if (result.estop) {
      status.data = "ESTOP";
    } else if (result.obstacle_detected) {
      if (bs == uav_local_planner::BypassState::LEFT)       status.data = "AVOIDING_L";
      else if (bs == uav_local_planner::BypassState::RIGHT) status.data = "AVOIDING_R";
      else                                                  status.data = "AVOIDING";
    } else {
      status.data = "NOMINAL";
    }
    status_pub_->publish(status);

    // ── Publish diagnostics ──────────────────────────────────────────────
    // Layout: [dist_to_goal, yaw, yaw_accum, orbiting, stalled, bypass_state,
    //          stall_best_dist, stall_since_improve, closest_obstacle,
    //          best_prim_idx, estop, obstacle_detected]
    {
      const double bs_val = (bs == uav_local_planner::BypassState::LEFT)  ? 1.0 :
                            (bs == uav_local_planner::BypassState::RIGHT) ? 2.0 : 0.0;
      std_msgs::msg::Float64MultiArray diag;
      diag.data = {
          dist_to_goal, yaw,
          orbit_.yaw_accumulator,
          orbit_.orbiting          ? 1.0 : 0.0,
          stall_.stalled           ? 1.0 : 0.0,
          bs_val,
          stall_.bestDist(),
          stall_.sinceImprove(),
          result.closest_obstacle_dist,
          static_cast<double>(result.best_primitive_idx),
          result.estop             ? 1.0 : 0.0,
          result.obstacle_detected ? 1.0 : 0.0
      };
      diag_pub_->publish(diag);
    }
  }

  std::unique_ptr<uav_local_planner::MotionPrimitives> planner_;
  StallDetector stall_;
  OrbitDetector orbit_;

  std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloud_;
  Eigen::Vector3d pos_{0, 0, 0}, waypoint_{0, 0, 0};
  double yaw_{0.0};
  bool have_odom_{false}, have_waypoint_{false};

  std::mutex cloud_mutex_, odom_mutex_, wp_mutex_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr   cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr          odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr              mission_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr       cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr                  status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr       diag_pub_;
  rclcpp::TimerBase::SharedPtr                                          timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPNode>());
  rclcpp::shutdown();
}
