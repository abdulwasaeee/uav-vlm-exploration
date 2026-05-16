#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <cmath>

namespace uav_local_planner {

struct MPConfig {
  // ── Arc primitive geometry ──────────────────────────────────────────
  int    num_az_horizontal = 18;    // azimuth directions (20° spacing)
  int    num_curvatures    = 5;     // curvature samples per direction
  double max_curvature     = 0.4;   // 1/m → min turning radius = 2.5 m
  double arc_length        = 2.0;   // metres, matches A010 max range

  // ── Pitched layers (ceiling / floor detection) ──────────────────────
  std::vector<double> elevation_angles_deg = {30.0, 15.0, -15.0, -30.0};
  int    num_az_pitched    = 18;    // azimuth directions per pitched layer

  // ── Collision geometry ───────────────────────────────────────────────
  double collision_radius  = 0.75;  // m — robot radius + safety margin
  double min_clearance     = 1.0;   // m — e-stop distance

  // ── Speed limits ─────────────────────────────────────────────────────
  double max_speed         = 2.5;   // m/s horizontal
  double min_speed         = 0.2;   // m/s floor near obstacles
  double max_vz            = 1.0;   // m/s vertical
  double alt_kp            = 0.8;   // altitude P-gain

  // ── Scoring weights ──────────────────────────────────────────────────
  double w_goal            = 3.0;   // angular cost toward goal direction
  double w_prev            = 2.0;   // angular cost toward previous primitive
  double w_pos             = 2.0;   // positional cost — reward closing distance to goal

  // ── Obstacle detection persistence ──────────────────────────────────
  // Once obstacle_detected goes true, it stays true until obs_dist exceeds
  // arc_length * obstacle_hysteresis_factor.  Prevents AVOIDING→NOMINAL
  // flicker when the obstacle sits right at the detection boundary.
  double obstacle_hysteresis_factor = 1.3;  // disengage at arc_length * 1.3

  // ── Bypass commitment ─────────────────────────────────────────────────
  // Minimum planner cycles to hold bypass_state before checking the exit
  // condition.  At 20 Hz, 60 cycles = 3 s.  The safety fallback (bypass
  // side fully blocked) always overrides this timer.
  int    bypass_min_hold_cycles = 60;

  // ── Slew-rate limiting ───────────────────────────────────────────────
  double max_accel = 0.75;          // m/s² — max velocity change per second

  // ── Pessimistic obstacle distance filter ─────────────────────────────
  int    pessimistic_window = 10;   // cycles — temporal-min window (0.5 s at 20 Hz)

  // ── Recovery cooldown ────────────────────────────────────────────────
  double recovery_duration_sec = 2.0;  // seconds of cooldown after ESTOP
  int    recovery_duration_cycles = 40; // cycles = duration_sec × 20 Hz (set by node)
  double recovery_max_speed    = 0.5;  // m/s cap during RECOVERING state

  // ── Temporal point buffer ────────────────────────────────────────────
  // Points stored in base_link frame. Primary benefit: recovering blind spots
  // during yaw manoeuvres (translation drift is acceptable for short windows).
  int    history_capacity  = 16384; // max buffered points from past frames
  int    history_subsample = 2;     // keep 1-in-N points from each new cloud
};

struct MPResult {
  Eigen::Vector3d velocity;         // map (ENU) frame
  bool   estop;
  bool   obstacle_detected;
  double closest_obstacle_dist;
  int    best_primitive_idx;        // index into horiz_prims_, -1 if none
};

enum class BypassState { NONE, LEFT, RIGHT };

class MotionPrimitives {
public:
  explicit MotionPrimitives(const MPConfig& cfg);

  // Primary update — call at 20 Hz.
  // cloud     : fused ToF cloud in base_link frame
  // drone_pos : position in map frame (ENU)
  // drone_yaw : yaw in map frame (radians)
  // waypoint  : target in map frame (ENU)
  MPResult update(
      const pcl::PointCloud<pcl::PointXYZ>& cloud,
      const Eigen::Vector3d& drone_pos,
      double drone_yaw,
      const Eigen::Vector3d& waypoint);

  void reset();

  // Read-only access for diagnostics
  int numHorizPrims() const { return static_cast<int>(horiz_prims_.size()); }
  int numPitchedPrims() const { return static_cast<int>(pitched_prims_.size()); }
  BypassState bypassState() const { return bypass_state_; }
  bool inRecovery() const { return recovery_cycles_remaining_ > 0; }

private:
  // ── Pre-computed arc geometry ────────────────────────────────────────
  struct ArcGeom {
    Eigen::Vector2d center;   // arc center in XY (base_link)
    double R;                 // signed radius = 1/κ
    double alpha_start;       // angle from center to arc start (origin)
    double delta_theta;       // signed arc span = κ * arc_length
    Eigen::Vector2d end_xy;   // arc endpoint in XY
  };

  struct Primitive {
    Eigen::Vector3d end_point;   // 3D terminal point in base_link
    double az_angle;             // initial heading azimuth (radians)
    double elevation_rad;        // elevation angle (0 for horizontal)
    double curvature;            // κ (0 = straight)
    double terminal_az;          // az_angle + κ·arc_length (for scoring)
    bool   is_curved;
    ArcGeom arc;                 // valid only when is_curved == true
  };

  void buildPrimitives();
  void updateBuffer(const pcl::PointCloud<pcl::PointXYZ>& cloud);

  // Analytic 2D distance from (px,py) to a constant-curvature arc
  double pointToArcDist2D(double px, double py, const ArcGeom& a) const;

  // 3D point-to-segment distance; segment runs from origin to seg_end
  double pointToSegmentDist(const Eigen::Vector3d& p,
                            const Eigen::Vector3d& seg_end) const;

  // Combined speed: min of distance-ramp and valid-fraction-density
  double adaptiveSpeed(double closest_dist, double valid_fraction) const;

  static double normalizeAngle(double a);

  MPConfig cfg_;

  std::vector<Primitive> horiz_prims_;    // curved arcs at elevation = 0
  std::vector<Primitive> pitched_prims_;  // straight rays at ±15°, ±30°

  // Circular FIFO of cloud points in base_link frame (Eigen float saves memory)
  std::vector<Eigen::Vector3f> point_buf_;
  int write_head_{0};
  int buf_fill_{0};

  int prev_best_{-1};
  BypassState bypass_state_{BypassState::NONE};
  int  bypass_hold_cycles_{0};       // cycles elapsed since bypass engaged
  bool prev_obstacle_detected_{false};

  // Pessimistic obstacle distance filter
  std::vector<double> recent_obs_dists_;
  int obs_dist_buf_idx_{0};

  // Recovery cooldown
  int recovery_cycles_remaining_{0};  // decremented each non-ESTOP cycle
};

}  // namespace uav_local_planner
