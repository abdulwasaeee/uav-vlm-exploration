#pragma once
// vfh3d.hpp
// 3DVFH+ algorithm — builds a 2D polar histogram from a local Octomap
// bounding box, identifies free directions, and selects the best velocity
// setpoint toward the current waypoint while avoiding obstacles.
//
// Reference: Vanneste et al., "3DVFH+: Local Obstacle Avoidance in 3D"

#include <octomap/octomap.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

namespace uav_local_planner {

// ── Configuration ─────────────────────────────────────────────────────
struct VFH3DConfig {
  // Histogram resolution
  int   az_sectors  = 72;    // azimuth bins   (360° / 5° = 72)
  int   el_sectors  = 36;    // elevation bins (180° / 5° = 36)

  // Local bounding box around drone (metres)
  double bbox_radius = 3.0;  // matches A010 max range

  // Robot size
  double robot_radius    = 0.30;
  double safety_radius   = 0.15;  // extra inflation

  // Histogram thresholds
  float  h_high = 0.5f;  // above → occupied sector
  float  h_low  = 0.2f;  // below → free sector (hysteresis)

  // Candidate direction selection weights
  double w_goal    = 3.0;  // weight toward waypoint direction
  double w_current = 1.0;  // weight toward current heading (smoothness)
  double w_prev    = 2.0;  // weight toward previous selected direction

  // Speed limits
  double max_speed     = 1.0;   // m/s
  double min_speed     = 0.1;   // m/s — minimum when close to obstacle
  double max_vz        = 0.5;   // m/s vertical

  // Safety
  double min_clearance = 0.6;    // metres — e-stop threshold (increased from 0.5)
};

// ── Result from one VFH+ update cycle ────────────────────────────────
struct VFH3DResult {
  Eigen::Vector3d velocity;     // recommended velocity in base_link frame
  bool   obstacle_detected;
  bool   estop;                 // true if within min_clearance
  double closest_obstacle_dist;
  int    best_az_sector;        // for debugging
  int    best_el_sector;
};

// ── Main algorithm class ──────────────────────────────────────────────
class VFH3D {
public:
  explicit VFH3D(const VFH3DConfig& cfg);

  // Primary update — call at 20-50 Hz
  // octree    : local octomap (full map, algorithm extracts bbox internally)
  // drone_pos : current drone position in map frame
  // waypoint  : next target waypoint in map frame
  // current_vel: current drone velocity in map frame (for smoothing)
  VFH3DResult update(
      const octomap::OcTree& octree,
      const Eigen::Vector3d& drone_pos,
      const Eigen::Vector3d& waypoint,
      const Eigen::Vector3d& current_vel);

  // Reset internal state (call when switching waypoints)
  void reset();

private:
  // Build 2D polar histogram from voxels in bbox
  void buildHistogram(
      const octomap::OcTree& octree,
      const Eigen::Vector3d& drone_pos);

  // Smooth histogram with Gaussian kernel
  void smoothHistogram();

  // Find all candidate free directions (valleys in histogram)
  struct Candidate {
    int az, el;
    double cost;
  };
  std::vector<Candidate> findCandidates(
      const Eigen::Vector3d& drone_pos,
      const Eigen::Vector3d& waypoint);

  // Convert sector indices to unit direction vector
  Eigen::Vector3d sectorToDirection(int az, int el) const;

  // Convert direction vector to sector indices
  void directionToSector(const Eigen::Vector3d& dir, int& az, int& el) const;

  // Compute adaptive speed based on closest obstacle
  double adaptiveSpeed(double closest_dist) const;

  VFH3DConfig cfg_;

  // 2D histogram: [az_sectors][el_sectors]
  std::vector<std::vector<float>> hist_;
  std::vector<std::vector<float>> hist_smooth_;

  // Previous selected direction for smoothing
  int prev_az_{0}, prev_el_{0};
  bool has_prev_{false};

  // Gaussian kernel for smoothing
  std::vector<float> kernel_;
  int kernel_size_{3};
};

}  // namespace uav_local_planner