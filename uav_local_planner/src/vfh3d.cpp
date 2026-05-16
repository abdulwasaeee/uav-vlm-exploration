// vfh3d.cpp — 3DVFH+ implementation

#include <uav_local_planner/vfh3d.hpp>
#include <algorithm>
#include <numeric>
#include <limits>

namespace uav_local_planner {

VFH3D::VFH3D(const VFH3DConfig& cfg) : cfg_(cfg)
{
  // Allocate histograms
  hist_.assign(cfg_.az_sectors,
      std::vector<float>(cfg_.el_sectors, 0.0f));
  hist_smooth_.assign(cfg_.az_sectors,
      std::vector<float>(cfg_.el_sectors, 0.0f));

  // Build 1D Gaussian kernel for histogram smoothing
  kernel_size_ = 3;
  kernel_.resize(2 * kernel_size_ + 1);
  float sigma = 1.5f, sum = 0.0f;
  for (int i = -kernel_size_; i <= kernel_size_; ++i) {
    kernel_[i + kernel_size_] = std::exp(-0.5f * (i*i) / (sigma*sigma));
    sum += kernel_[i + kernel_size_];
  }
  for (auto& v : kernel_) v /= sum;
}

void VFH3D::reset()
{
  has_prev_ = false;
  for (auto& row : hist_)        std::fill(row.begin(), row.end(), 0.0f);
  for (auto& row : hist_smooth_) std::fill(row.begin(), row.end(), 0.0f);
}

// ── Main update ───────────────────────────────────────────────────────
VFH3DResult VFH3D::update(
    const octomap::OcTree& octree,
    const Eigen::Vector3d& drone_pos,
    const Eigen::Vector3d& waypoint,
    const Eigen::Vector3d& current_vel)
{
  (void)current_vel;  // reserved for velocity smoothing in future
  VFH3DResult result;
  result.obstacle_detected    = false;
  result.estop                = false;
  result.closest_obstacle_dist = std::numeric_limits<double>::max();
  result.velocity             = Eigen::Vector3d::Zero();

  // ── 1. Build polar histogram ────────────────────────────────────────
  buildHistogram(octree, drone_pos);
  smoothHistogram();

  // ── 2. Find closest obstacle for speed scaling + e-stop ─────────────
  double res = octree.getResolution();
  double r   = cfg_.bbox_radius;
  int steps  = static_cast<int>(r / res);

  for (int dx = -steps; dx <= steps; ++dx)
  for (int dy = -steps; dy <= steps; ++dy)
  for (int dz = -steps; dz <= steps; ++dz) {
    octomap::point3d query(
        drone_pos.x() + dx*res,
        drone_pos.y() + dy*res,
        drone_pos.z() + dz*res);
    auto* node = octree.search(query);
    if (node && octree.isNodeOccupied(node)) {
      double dist = std::sqrt(dx*dx + dy*dy + dz*dz) * res;
      result.closest_obstacle_dist = std::min(result.closest_obstacle_dist, dist);
      result.obstacle_detected = true;
    }
  }

  // E-stop
  if (result.closest_obstacle_dist < cfg_.min_clearance) {
    result.estop = true;
    result.velocity = Eigen::Vector3d::Zero();
    return result;
  }

  // ── 3. Find best free direction toward waypoint ──────────────────────
  auto candidates = findCandidates(drone_pos, waypoint);

  if (candidates.empty()) {
    // All directions blocked — hover in place
    result.velocity = Eigen::Vector3d::Zero();
    return result;
  }

  // Pick lowest-cost candidate
  auto best = std::min_element(candidates.begin(), candidates.end(),
      [](const Candidate& a, const Candidate& b){ return a.cost < b.cost; });

  result.best_az_sector = best->az;
  result.best_el_sector = best->el;

  // Update previous direction
  prev_az_  = best->az;
  prev_el_  = best->el;
  has_prev_ = true;

  // ── 4. Compute velocity vector ───────────────────────────────────────
  Eigen::Vector3d dir = sectorToDirection(best->az, best->el);
  double speed = adaptiveSpeed(result.closest_obstacle_dist);

  // Scale vertical component separately
  Eigen::Vector3d vel = dir * speed;
  vel.z() = std::clamp(vel.z(), -cfg_.max_vz, cfg_.max_vz);

  result.velocity = vel;
  return result;
}

// ── Build histogram ───────────────────────────────────────────────────
void VFH3D::buildHistogram(
    const octomap::OcTree& octree,
    const Eigen::Vector3d& drone_pos)
{
  // Reset
  for (auto& row : hist_) std::fill(row.begin(), row.end(), 0.0f);

  double res   = octree.getResolution();
  double r     = cfg_.bbox_radius;
  double r_rob = cfg_.robot_radius + cfg_.safety_radius;
  int    steps = static_cast<int>(r / res);

  for (int dx = -steps; dx <= steps; ++dx)
  for (int dy = -steps; dy <= steps; ++dy)
  for (int dz = -steps; dz <= steps; ++dz) {
    double dist = std::sqrt(dx*dx + dy*dy + dz*dz) * res;
    if (dist < 0.01 || dist > r) continue;

    octomap::point3d query(
        drone_pos.x() + dx*res,
        drone_pos.y() + dy*res,
        drone_pos.z() + dz*res);
    auto* node = octree.search(query);
    if (!node || !octree.isNodeOccupied(node)) continue;

    // Direction from drone to voxel
    Eigen::Vector3d dir(dx*res, dy*res, dz*res);
    dir.normalize();

    // Magnitude: closer + more certain = higher contribution
    // Enlarged by robot radius (VFH+ compensation)
    double enlarged_dist = std::max(dist - r_rob, 0.01);
    float  mag = static_cast<float>(
        node->getOccupancy() / (enlarged_dist * enlarged_dist));

    // Map to histogram sector
    int az, el;
    directionToSector(dir, az, el);
    hist_[az][el] += mag;
  }
}

// ── Gaussian smoothing (wrap-around on azimuth) ───────────────────────
void VFH3D::smoothHistogram()
{
  for (int az = 0; az < cfg_.az_sectors; ++az)
  for (int el = 0; el < cfg_.el_sectors; ++el) {
    float val = 0.0f;
    for (int k = -kernel_size_; k <= kernel_size_; ++k) {
      int naz = (az + k + cfg_.az_sectors) % cfg_.az_sectors;
      val += kernel_[k + kernel_size_] * hist_[naz][el];
    }
    hist_smooth_[az][el] = val;
  }
}

// ── Find candidate free directions ───────────────────────────────────
std::vector<VFH3D::Candidate> VFH3D::findCandidates(
    const Eigen::Vector3d& drone_pos,
    const Eigen::Vector3d& waypoint)
{
  // Goal direction
  Eigen::Vector3d goal_dir = (waypoint - drone_pos);
  if (goal_dir.norm() < 1e-3) goal_dir = Eigen::Vector3d(1,0,0);
  goal_dir.normalize();

  int goal_az, goal_el;
  directionToSector(goal_dir, goal_az, goal_el);

  // Current heading direction (from prev selected or goal)
  int cur_az = has_prev_ ? prev_az_ : goal_az;
  int cur_el = has_prev_ ? prev_el_ : goal_el;

  std::vector<Candidate> candidates;

  for (int az = 0; az < cfg_.az_sectors; ++az)
  for (int el = 0; el < cfg_.el_sectors; ++el) {
    // Only consider free sectors (below low threshold)
    if (hist_smooth_[az][el] > cfg_.h_low) continue;

    // Angular distances (wrap-around for azimuth)
    int daz_goal = std::abs(az - goal_az);
    daz_goal = std::min(daz_goal, cfg_.az_sectors - daz_goal);
    int del_goal = std::abs(el - goal_el);

    int daz_cur = std::abs(az - cur_az);
    daz_cur = std::min(daz_cur, cfg_.az_sectors - daz_cur);
    int del_cur = std::abs(el - cur_el);

    int daz_prev = has_prev_ ? std::abs(az - prev_az_) : daz_goal;
    daz_prev = std::min(daz_prev, cfg_.az_sectors - daz_prev);
    int del_prev = has_prev_ ? std::abs(el - prev_el_) : del_goal;

    // Cost function: weighted sum of angular distances
    double cost =
        cfg_.w_goal    * std::sqrt(daz_goal*daz_goal + del_goal*del_goal) +
        cfg_.w_current * std::sqrt(daz_cur*daz_cur   + del_cur*del_cur)   +
        cfg_.w_prev    * std::sqrt(daz_prev*daz_prev + del_prev*del_prev);

    candidates.push_back({az, el, cost});
  }

  return candidates;
}

// ── Sector ↔ direction conversions ───────────────────────────────────
Eigen::Vector3d VFH3D::sectorToDirection(int az, int el) const
{
  // az: 0..az_sectors → 0..2π
  // el: 0..el_sectors → -π/2..π/2  (el=0 → straight down, el=el_sectors-1 → straight up)
  double azimuth   = (az + 0.5) * 2.0 * M_PI / cfg_.az_sectors;
  double elevation = (el + 0.5) * M_PI / cfg_.el_sectors - M_PI_2;

  return Eigen::Vector3d(
      std::cos(elevation) * std::cos(azimuth),
      std::cos(elevation) * std::sin(azimuth),
      std::sin(elevation));
}

void VFH3D::directionToSector(const Eigen::Vector3d& dir, int& az, int& el) const
{
  double azimuth   = std::atan2(dir.y(), dir.x());
  if (azimuth < 0) azimuth += 2.0 * M_PI;
  double elevation = std::asin(std::clamp(dir.z(), -1.0, 1.0));

  az = static_cast<int>(azimuth   / (2.0 * M_PI) * cfg_.az_sectors);
  el = static_cast<int>((elevation + M_PI_2) / M_PI * cfg_.el_sectors);

  az = std::clamp(az, 0, cfg_.az_sectors - 1);
  el = std::clamp(el, 0, cfg_.el_sectors - 1);
}

// ── Adaptive speed ────────────────────────────────────────────────────
double VFH3D::adaptiveSpeed(double closest_dist) const
{
  // Linear ramp: full speed beyond bbox_radius/2, min speed at min_clearance
  double d_high = cfg_.bbox_radius / 2.0;
  double d_low  = cfg_.min_clearance;
  double t = std::clamp((closest_dist - d_low) / (d_high - d_low), 0.0, 1.0);
  return cfg_.min_speed + t * (cfg_.max_speed - cfg_.min_speed);
}

}  // namespace uav_local_planner