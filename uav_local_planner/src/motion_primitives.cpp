// motion_primitives.cpp
// Motion Primitive Library local planner — V2
//
// Improvements over V1:
//   - Constant-curvature arc primitives (analytic point-to-arc distance)
//   - Pitched straight-ray layers (±15°, ±30°) with Z-band pre-filter
//   - Circular FIFO history buffer (covers sensor blind spots during yaw)
//   - Adaptive speed from both closest-obstacle distance AND valid-arc fraction

#include <uav_local_planner/motion_primitives.hpp>
#include <algorithm>
#include <limits>
#include <cmath>

namespace uav_local_planner {

// ── Construction ──────────────────────────────────────────────────────────
MotionPrimitives::MotionPrimitives(const MPConfig& cfg) : cfg_(cfg)
{
  point_buf_.resize(cfg_.history_capacity);
  recent_obs_dists_.assign(cfg_.pessimistic_window, std::numeric_limits<double>::max());
  buildPrimitives();
}

void MotionPrimitives::reset()
{
  prev_best_               = -1;
  write_head_              = 0;
  buf_fill_                = 0;
  bypass_state_            = BypassState::NONE;
  bypass_hold_cycles_      = 0;
  prev_obstacle_detected_  = false;
  obs_dist_buf_idx_        = 0;
  recovery_cycles_remaining_ = 0;
  recent_obs_dists_.assign(cfg_.pessimistic_window,
                           std::numeric_limits<double>::max());
}

// ── Primitive generation ──────────────────────────────────────────────────
void MotionPrimitives::buildPrimitives()
{
  horiz_prims_.clear();
  pitched_prims_.clear();

  const double L = cfg_.arc_length;

  // Curvature set: linearly spaced in [-max_κ, +max_κ]
  std::vector<double> curvatures(cfg_.num_curvatures);
  if (cfg_.num_curvatures == 1) {
    curvatures[0] = 0.0;
  } else {
    for (int j = 0; j < cfg_.num_curvatures; ++j) {
      curvatures[j] = -cfg_.max_curvature +
          j * 2.0 * cfg_.max_curvature / (cfg_.num_curvatures - 1);
    }
  }

  // ── Horizontal curved arcs ────────────────────────────────────────────
  for (int i = 0; i < cfg_.num_az_horizontal; ++i) {
    const double az = i * 2.0 * M_PI / cfg_.num_az_horizontal;

    for (double kappa : curvatures) {
      Primitive p;
      p.az_angle     = az;
      p.elevation_rad = 0.0;
      p.curvature    = kappa;

      if (std::abs(kappa) < 1e-4) {
        // Straight line
        p.is_curved    = false;
        p.terminal_az  = az;
        p.end_point    = Eigen::Vector3d(std::cos(az) * L, std::sin(az) * L, 0.0);
      } else {
        p.is_curved = true;
        const double R = 1.0 / kappa;  // signed radius

        // Arc center: R * left-perpendicular to heading az
        // left-perp = (-sin(az), cos(az))  →  C = R*(-sin(az), cos(az))
        p.arc.R      = R;
        p.arc.center = Eigen::Vector2d(-R * std::sin(az), R * std::cos(az));

        // Angle from C to arc start (origin):
        //   vec = (0 - C.x, 0 - C.y) = (R·sin(az), -R·cos(az))
        //   → atan2(-R·cos(az), R·sin(az))  — handles both signs of R
        p.arc.alpha_start = std::atan2(-R * std::cos(az), R * std::sin(az));

        // Total heading change along arc
        p.arc.delta_theta = kappa * L;

        // Arc endpoint via parametric integral
        //   x = (sin(az + κL) - sin(az)) / κ
        //   y = (cos(az)       - cos(az + κL)) / κ
        const double az1 = az + kappa * L;
        p.arc.end_xy = Eigen::Vector2d(
            (std::sin(az1) - std::sin(az)) / kappa,
            (std::cos(az)  - std::cos(az1)) / kappa);
        p.end_point    = Eigen::Vector3d(p.arc.end_xy.x(), p.arc.end_xy.y(), 0.0);
        p.terminal_az  = az1;
      }
      horiz_prims_.push_back(p);
    }
  }

  // ── Pitched straight-line segments ────────────────────────────────────
  for (double el_deg : cfg_.elevation_angles_deg) {
    const double el = el_deg * M_PI / 180.0;
    for (int i = 0; i < cfg_.num_az_pitched; ++i) {
      const double az = i * 2.0 * M_PI / cfg_.num_az_pitched;
      Primitive p;
      p.az_angle      = az;
      p.elevation_rad = el;
      p.curvature     = 0.0;
      p.terminal_az   = az;
      p.is_curved     = false;
      p.end_point = Eigen::Vector3d(
          std::cos(az) * std::cos(el) * L,
          std::sin(az) * std::cos(el) * L,
          std::sin(el) * L);
      pitched_prims_.push_back(p);
    }
  }
}

// ── History buffer ────────────────────────────────────────────────────────
void MotionPrimitives::updateBuffer(const pcl::PointCloud<pcl::PointXYZ>& cloud)
{
  const int step = std::max(1, cfg_.history_subsample);
  const int cap  = cfg_.history_capacity;

  for (int i = 0; i < static_cast<int>(cloud.size()); i += step) {
    const auto& pt = cloud.points[i];
    if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
    const double d = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
    if (d < 0.1 || d > cfg_.arc_length + cfg_.collision_radius) continue;

    point_buf_[write_head_] = Eigen::Vector3f(pt.x, pt.y, pt.z);
    write_head_ = (write_head_ + 1) % cap;
    if (buf_fill_ < cap) ++buf_fill_;
  }
}

// ── Main update ───────────────────────────────────────────────────────────
MPResult MotionPrimitives::update(
    const pcl::PointCloud<pcl::PointXYZ>& cloud,
    const Eigen::Vector3d& drone_pos,
    double drone_yaw,
    const Eigen::Vector3d& waypoint)
{
  MPResult result;
  result.estop                 = false;
  result.obstacle_detected     = false;
  result.closest_obstacle_dist = std::numeric_limits<double>::max();
  result.best_primitive_idx    = -1;
  result.velocity              = Eigen::Vector3d::Zero();

  // ── 1. Update history buffer ──────────────────────────────────────────
  updateBuffer(cloud);

  // ── 2. Pre-filter buffer into per-layer point vectors ─────────────────
  // This avoids repeating the Z-filter for every primitive.
  const double cr = cfg_.collision_radius;
  const double L  = cfg_.arc_length;

  // Horizontal layer: only points within ±collision_radius in Z
  std::vector<Eigen::Vector3f> horiz_pts;
  horiz_pts.reserve(buf_fill_);

  for (int i = 0; i < buf_fill_; ++i) {
    const float z = point_buf_[i].z();
    if (std::abs(z) <= static_cast<float>(cr))
      horiz_pts.push_back(point_buf_[i]);

    // Track closest obstacle across ALL points (for e-stop and speed)
    const double d = point_buf_[i].cast<double>().norm();
    if (d < result.closest_obstacle_dist) result.closest_obstacle_dist = d;
  }

  // Pessimistic temporal-min filter: smooths out high-frequency obstacle
  // distance flicker when the drone yaws between thin obstacles.
  {
    recent_obs_dists_[obs_dist_buf_idx_] = result.closest_obstacle_dist;
    obs_dist_buf_idx_ = (obs_dist_buf_idx_ + 1) % cfg_.pessimistic_window;
    result.closest_obstacle_dist = *std::min_element(
        recent_obs_dists_.begin(), recent_obs_dists_.end());
  }

  // Hysteresis: engage at arc_length, disengage at arc_length × factor.
  // Prevents AVOIDING→NOMINAL flicker when obs_dist oscillates at the boundary.
  {
    const bool  raw_det   = result.closest_obstacle_dist < L;
    const double disengage = L * cfg_.obstacle_hysteresis_factor;
    result.obstacle_detected = raw_det ||
        (prev_obstacle_detected_ && result.closest_obstacle_dist < disengage);
    prev_obstacle_detected_ = result.obstacle_detected;
  }

  // Pitched layers: one filtered list per elevation angle
  const int num_layers = static_cast<int>(cfg_.elevation_angles_deg.size());
  std::vector<std::vector<Eigen::Vector3f>> layer_pts(num_layers);
  for (int li = 0; li < num_layers; ++li) {
    const double el    = cfg_.elevation_angles_deg[li] * M_PI / 180.0;
    const double z_min = std::min(0.0, std::sin(el) * L) - cr;
    const double z_max = std::max(0.0, std::sin(el) * L) + cr;
    layer_pts[li].reserve(buf_fill_ / 4);
    for (int i = 0; i < buf_fill_; ++i) {
      const float z = point_buf_[i].z();
      if (z >= static_cast<float>(z_min) && z <= static_cast<float>(z_max))
        layer_pts[li].push_back(point_buf_[i]);
    }
  }

  // ── 3. E-stop ──────────────────────────────────────────────────────────
  if (result.closest_obstacle_dist < cfg_.min_clearance) {
    result.estop = true;
    recovery_cycles_remaining_ = cfg_.recovery_duration_cycles;
    return result;
  }

  // Decrement recovery timer — after ESTOP has cleared
  if (recovery_cycles_remaining_ > 0) --recovery_cycles_remaining_;

  // ── 4. Collision check — horizontal primitives ────────────────────────
  const int nh = static_cast<int>(horiz_prims_.size());
  std::vector<bool> valid_h(nh, true);

  for (const auto& pt : horiz_pts) {
    const double px = pt.x(), py = pt.y(), pz = pt.z();
    for (int i = 0; i < nh; ++i) {
      if (!valid_h[i]) continue;
      const Primitive& p = horiz_prims_[i];
      double dist;
      if (p.is_curved) {
        const double d2 = pointToArcDist2D(px, py, p.arc);
        dist = std::sqrt(d2 * d2 + pz * pz);
      } else {
        dist = pointToSegmentDist(Eigen::Vector3d(px, py, pz), p.end_point);
      }
      if (dist < cr) valid_h[i] = false;
    }
  }

  int valid_h_count = 0;
  for (bool v : valid_h) if (v) ++valid_h_count;

  // ── 5. Collision check — pitched primitives ───────────────────────────
  const int np = static_cast<int>(pitched_prims_.size());
  std::vector<bool> valid_p(np, true);

  // pitched_prims_ is ordered: all prims for el[0], then el[1], etc.
  const int per_layer = cfg_.num_az_pitched;
  for (int li = 0; li < num_layers; ++li) {
    const int base = li * per_layer;
    for (const auto& pt : layer_pts[li]) {
      const Eigen::Vector3d p3(pt.x(), pt.y(), pt.z());
      for (int j = 0; j < per_layer; ++j) {
        const int idx = base + j;
        if (!valid_p[idx]) continue;
        if (pointToSegmentDist(p3, pitched_prims_[idx].end_point) < cr)
          valid_p[idx] = false;
      }
    }
  }

  // Compute upper/lower valid fractions for vz scaling
  int upper_valid = 0, upper_total = 0;
  int lower_valid = 0, lower_total = 0;
  for (int li = 0; li < num_layers; ++li) {
    const int base = li * per_layer;
    const bool is_upper = cfg_.elevation_angles_deg[li] > 0;
    for (int j = 0; j < per_layer; ++j) {
      const int idx = base + j;
      if (is_upper) { ++upper_total; if (valid_p[idx]) ++upper_valid; }
      else           { ++lower_total; if (valid_p[idx]) ++lower_valid; }
    }
  }
  const double upper_frac = upper_total > 0 ? static_cast<double>(upper_valid) / upper_total : 1.0;
  const double lower_frac = lower_total > 0 ? static_cast<double>(lower_valid) / lower_total : 1.0;

  // ── 6. Goal direction in base_link frame ──────────────────────────────
  const double cy = std::cos(drone_yaw), sy = std::sin(drone_yaw);
  Eigen::Vector3d goal_map(waypoint.x() - drone_pos.x(),
                            waypoint.y() - drone_pos.y(), 0.0);
  if (goal_map.norm() < 1e-3) goal_map = Eigen::Vector3d(1, 0, 0);
  goal_map.normalize();

  const double gx_base =  cy * goal_map.x() + sy * goal_map.y();
  const double gy_base = -sy * goal_map.x() + cy * goal_map.y();
  const double goal_az_base = std::atan2(gy_base, gx_base);

  // ── 7. Bypass state machine ───────────────────────────────────────────
  // Trigger: fewer than 2 valid primitives within ±25° of goal direction.
  //
  // Side selection and filtering use goal-relative azimuth (terminal_az - goal_az_base)
  // rather than body-frame Y, making the committed bypass direction invariant to drone
  // yaw.  (Bug fix V3: previous body-frame check rotated with the drone during an orbit.)
  //
  // The dead-code second trigger `best_d_goal > marginal_angle` has been removed;
  // it could only fire when near_goal_valid == 0 (first condition already sufficient).
  const double block_angle_rad = 25.0 * M_PI / 180.0;
  int near_goal_valid = 0;
  for (int i = 0; i < nh; ++i) {
    if (!valid_h[i]) continue;
    if (std::abs(normalizeAngle(horiz_prims_[i].terminal_az - goal_az_base)) < block_angle_rad)
      ++near_goal_valid;
  }

  bool goal_blocked = (near_goal_valid < 2);

  // Enter bypass: commit to whichever side (left/right of the goal direction) has more
  // valid primitives.  "left" = positive goal-relative azimuth (CCW of goal in world frame).
  if (goal_blocked && bypass_state_ == BypassState::NONE) {
    int left_valid = 0, right_valid = 0;
    for (int i = 0; i < nh; ++i) {
      if (!valid_h[i]) continue;
      double rel = normalizeAngle(horiz_prims_[i].terminal_az - goal_az_base);
      if (rel > 0.05) left_valid++;
      else if (rel < -0.05) right_valid++;
    }
    if (left_valid > 0 || right_valid > 0) {
      bypass_state_       = (left_valid >= right_valid) ? BypassState::LEFT : BypassState::RIGHT;
      bypass_hold_cycles_ = 0;  // start commitment timer
    }
  }

  // Build filtered valid set for scoring
  std::vector<bool> valid_score = valid_h;
  if (bypass_state_ != BypassState::NONE) {
    ++bypass_hold_cycles_;

    bool keep_left = (bypass_state_ == BypassState::LEFT);
    for (int i = 0; i < nh; ++i) {
      if (!valid_score[i]) continue;
      // Goal-relative sign: positive = left of goal (CCW), negative = right (CW)
      bool prim_left = normalizeAngle(horiz_prims_[i].terminal_az - goal_az_base) > 0.05;
      if (prim_left != keep_left) valid_score[i] = false;
    }

    // Exit bypass when a near-straight primitive toward the goal becomes valid.
    // Only checked after the minimum commitment window has elapsed so the drone
    // cannot abort bypass the moment the path transiently clears.
    if (bypass_hold_cycles_ >= cfg_.bypass_min_hold_cycles) {
      for (int i = 0; i < nh; ++i) {
        if (!valid_h[i]) continue;
        if (std::abs(horiz_prims_[i].curvature) > 0.08) continue;
        double daz = std::abs(normalizeAngle(
            horiz_prims_[i].terminal_az - goal_az_base));
        if (daz < block_angle_rad) {
          bypass_state_ = BypassState::NONE;
          valid_score = valid_h;
          break;
        }
      }
    }

    // Safety fallback: if the committed bypass side has no valid primitives at
    // all, revert immediately regardless of the hold timer.
    int filtered_count = 0;
    for (bool v : valid_score) if (v) ++filtered_count;
    if (filtered_count == 0) {
      bypass_state_ = BypassState::NONE;
      valid_score = valid_h;
    }
  }

  // ── 8. Score valid horizontal primitives ─────────────────────────────
  // Pre-compute endpoint-to-goal distances for positional cost term.
  std::vector<double> end_dist(nh, 1e9);
  double best_end_dist = 1e9;
  for (int i = 0; i < nh; ++i) {
    if (!valid_score[i]) continue;
    const auto& ep = horiz_prims_[i].end_point;
    const double ex = drone_pos.x() + cy * ep.x() - sy * ep.y();
    const double ey = drone_pos.y() + sy * ep.x() + cy * ep.y();
    const double d = std::hypot(waypoint.x() - ex, waypoint.y() - ey);
    end_dist[i] = d;
    if (d < best_end_dist) best_end_dist = d;
  }

  // Guard against degenerate case (all blocked or best_end_dist ≈ 0)
  if (best_end_dist < 1e-6) best_end_dist = 1e-6;

  double best_cost = std::numeric_limits<double>::max();
  int    best_idx  = -1;

  for (int i = 0; i < nh; ++i) {
    if (!valid_score[i]) continue;

    const double d_goal = std::abs(normalizeAngle(
        horiz_prims_[i].terminal_az - goal_az_base));

    double d_prev = 0.0;
    if (prev_best_ >= 0) {
      d_prev = std::abs(normalizeAngle(
          horiz_prims_[i].terminal_az - horiz_prims_[prev_best_].terminal_az));
    }

    // Positional cost: normalise against best end-distance so the primitive
    // that ends closest to the goal gets cost=1.0×w_pos, others pay a penalty
    // proportional to how much farther they end from the goal.
    const double pos_norm = end_dist[i] / best_end_dist;

    const double cost = cfg_.w_goal * d_goal
                      + cfg_.w_prev * d_prev
                      + cfg_.w_pos  * pos_norm;
    if (cost < best_cost) {
      best_cost = cost;
      best_idx  = i;
    }
  }

  if (best_idx < 0) {
    // All horizontal primitives blocked — hover
    return result;
  }

  prev_best_ = best_idx;
  result.best_primitive_idx = best_idx;

  // ── 9. Adaptive speed — min of distance-ramp and density-ramp ─────────
  const double valid_frac = static_cast<double>(valid_h_count) / nh;
  const double speed = adaptiveSpeed(result.closest_obstacle_dist, valid_frac);

  // ── 10. Velocity in map frame ─────────────────────────────────────────
  // Use INITIAL heading of selected arc (velocity direction at t=0)
  const double vx_base = std::cos(horiz_prims_[best_idx].az_angle) * speed;
  const double vy_base = std::sin(horiz_prims_[best_idx].az_angle) * speed;

  const double vx_map = cy * vx_base - sy * vy_base;
  const double vy_map = sy * vx_base + cy * vy_base;

  // Altitude: P-controller, scaled by pitched-layer clearance
  double vz = std::clamp(cfg_.alt_kp * (waypoint.z() - drone_pos.z()),
                          -cfg_.max_vz, cfg_.max_vz);
  if (vz > 0) vz *= upper_frac;   // reduce upward speed if ceiling blocked
  if (vz < 0) vz *= lower_frac;   // reduce downward speed if floor blocked

  result.velocity = Eigen::Vector3d(vx_map, vy_map, vz);
  return result;
}

// ── Analytic 2D point-to-arc distance ────────────────────────────────────
double MotionPrimitives::pointToArcDist2D(
    double px, double py, const ArcGeom& a) const
{
  const double dx   = px - a.center.x();
  const double dy   = py - a.center.y();
  const double d_pc = std::sqrt(dx * dx + dy * dy);

  if (d_pc < 1e-8) return std::abs(a.R);

  // Angle from center to point
  const double alpha_p   = std::atan2(dy, dx);
  const double alpha_rel = normalizeAngle(alpha_p - a.alpha_start);

  // Check containment within the arc's angular span
  const bool in_range = (a.delta_theta >= 0.0)
      ? (alpha_rel >= 0.0 && alpha_rel <= a.delta_theta)
      : (alpha_rel <= 0.0 && alpha_rel >= a.delta_theta);

  if (in_range) {
    return std::abs(d_pc - std::abs(a.R));
  }

  // Beyond arc endpoints: distance to nearer endpoint
  const double d_start = std::sqrt(px * px + py * py);  // start = origin
  const double d_end   = std::sqrt(
      (px - a.end_xy.x()) * (px - a.end_xy.x()) +
      (py - a.end_xy.y()) * (py - a.end_xy.y()));
  return std::min(d_start, d_end);
}

// ── 3D point-to-segment distance ─────────────────────────────────────────
double MotionPrimitives::pointToSegmentDist(
    const Eigen::Vector3d& p, const Eigen::Vector3d& seg_end) const
{
  const double len2 = seg_end.squaredNorm();
  if (len2 < 1e-10) return p.norm();
  const double t = std::clamp(p.dot(seg_end) / len2, 0.0, 1.0);
  return (p - t * seg_end).norm();
}

// ── Adaptive speed ────────────────────────────────────────────────────────
double MotionPrimitives::adaptiveSpeed(double closest_dist, double valid_frac) const
{
  // Quadratic distance ramp: biases speed toward the low end in the
  // "caution zone" between min_clearance and arc_length.
  // Starting the ramp at the full detection range (not arc_length/2) means
  // the drone is already slowing when it first sees an obstacle at 2 m.
  const double d_high = cfg_.arc_length;
  const double d_low  = cfg_.min_clearance;
  const double t_dist = std::clamp((closest_dist - d_low) / (d_high - d_low), 0.0, 1.0);

  // During recovery after ESTOP, cap the effective max speed
  const double eff_max_speed = (recovery_cycles_remaining_ > 0)
      ? cfg_.recovery_max_speed : cfg_.max_speed;

  const double speed_dist    = cfg_.min_speed + (t_dist * t_dist) * (eff_max_speed - cfg_.min_speed);

  // Density-based ramp: full speed when all primitives clear, min when all blocked
  const double speed_density = cfg_.min_speed + valid_frac * (cfg_.max_speed - cfg_.min_speed);

  // Take the more conservative of the two
  return std::min(speed_dist, speed_density);
}

// ── Utility ───────────────────────────────────────────────────────────────
double MotionPrimitives::normalizeAngle(double a)
{
  while (a >  M_PI) a -= 2.0 * M_PI;
  while (a <= -M_PI) a += 2.0 * M_PI;
  return a;
}

}  // namespace uav_local_planner
