# UAV Obstacle Avoidance Stack

A ROS 2 (Humble) obstacle avoidance system for UAVs with two interchangeable local planners, a plugin-based global planner (A* default), and full PX4 SITL integration. Validated on an F450 frame with a 5× ToF depth sensor array (Arducam A010, horizontal ring mount).

---

## Architecture

```
Sensing          5× A010 ToF cameras (100×100, 20 Hz, 72° horizontal ring)
                 + RGBD front camera + bottom depth camera
      ↓
Fusion           uav_depth_fusion — merges point clouds into base_link frame,
                 bridges PX4 odometry NED→ENU, broadcasts static sensor TFs
      ↓
Mapping          uav_mapping — OctoMap 3D occupancy (0.2 m voxels)
   ↙    ↘         [required for global planner; optional for local planner]
Global           uav_planner_interface — A* on OctoMap (plugin-based)
Planner   ↓
      Waypoints  uav_local_planner/waypoint_manager
          ↓
Local            uav_local_planner — reactive avoidance at 20 Hz
Planner          (Motion Primitive Library — default)
   [or]          (3DVFH+ on OctoMap — legacy, set use_mp:=false)
          ↓
Control          uav_control — velocity setpoints → PX4 TrajectorySetpoint (NED)
```

---

## Packages

| Package | Description |
|---|---|
| `uav_depth_fusion` | Point cloud merge node, static TF broadcaster, PX4 odometry bridge (NED→ENU) |
| `uav_mapping` | OctoMap server configuration and launch |
| `uav_planner_interface` | `GlobalPlannerBase` plugin interface, A* planner plugin, action server |
| `uav_local_planner` | Motion Primitive Library (default) + legacy 3DVFH+, waypoint manager |
| `uav_control` | Full lifecycle setpoint publisher (STARTUP → TAKEOFF → HOVER → AUTONOMOUS) |

---

## Local Planner Variants

### Motion Primitive Library (`mp_node`) — default

Bypasses OctoMap entirely. Operates directly on the raw fused ToF point cloud.

**How it works:**
1. Pre-computes 72 horizontal flight primitives (straight-line rays, 5° spacing, 2 m length) in the drone body frame at startup.
2. Each control cycle, iterates over cloud points and computes point-to-segment distance for each primitive. Primitives with any point within `collision_radius` are flagged invalid.
3. Scores valid primitives by a weighted cost: angular distance to goal direction (`w_goal`) and angular distance to previous selected primitive (`w_prev`) for smoothness.
4. Selects lowest-cost primitive, scales speed adaptively by closest obstacle distance, and applies a proportional altitude controller for vertical motion.
5. Outputs velocity in the map (ENU) frame to `/uav/cmd_vel`.

**Advantages over VFH3D:**
- No OctoMap dependency — lower latency (no map update lag), no `uav_mapping` node required
- Deterministic — no convergence risk, constant per-cycle compute cost
- Simpler to tune — fewer parameters than histogram thresholds

**Limitation:** Purely reactive, no map memory. Sensor coverage is horizontal only (matches the A010 ring mount); vertical obstacle detection relies on altitude hold.

### 3DVFH+ (`vfh3d_node`) — legacy

Builds a 2D polar histogram from OctoMap voxels in a local bounding box, smooths with a Gaussian kernel, and selects a free direction using a weighted cost function. Requires `uav_mapping` to be running.

---

## Requirements

### System
- ROS 2 Humble
- Gazebo Harmonic
- PX4 Autopilot (SITL)
- Python 3.10+

### ROS 2 packages
```bash
sudo apt install \
  ros-humble-pcl-ros \
  ros-humble-pcl-conversions \
  ros-humble-tf2-geometry-msgs \
  ros-humble-message-filters \
  ros-humble-octomap-server \
  ros-humble-octomap-ros \
  ros-humble-octomap-msgs \
  ros-humble-octomap \
  ros-humble-pluginlib \
  ros-humble-rclcpp-action \
  ros-humble-eigen3-cmake-module \
  libeigen3-dev
```

### Workspace dependencies
`px4_msgs` must be present in your workspace:
```bash
git clone https://github.com/PX4/px4_msgs.git src/px4_msgs
```

---

## Setup

### 1. Clone and build
```bash
git clone https://github.com/shantam98/px4_sitl_planner.git
cd px4_sitl_planner
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### 2. Configure sensor transforms
Edit `uav_depth_fusion/src/tf_static_broadcaster.cpp` if your sensor mounting positions differ from the default F450 layout:

| Sensor | Position (x, y, z) | Yaw |
|---|---|---|
| tof_0 | (0.25, 0.00, 0) | 0° (front) |
| tof_1 | (0.077, 0.236, 0) | 72° (front-left) |
| tof_2 | (-0.202, 0.147, 0) | 144° (rear-left) |
| tof_3 | (-0.202, -0.147, 0) | -144° (rear-right) |
| tof_4 | (0.077, -0.236, 0) | -72° (front-right) |

### 3. Configure ROS-Gazebo bridge
The included `sensor_bridge.yaml` maps Gazebo sensor topics to ROS 2:
```bash
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:=<path>/sensor_bridge.yaml
```

---

## Launch

### With Motion Primitive planner (default)

`uav_mapping` is not required for local avoidance when using `mp_node`.

```bash
# Terminal 1 — sensor fusion + TF + odometry bridge
ros2 launch uav_depth_fusion fusion.launch.py

# Terminal 2 — global planner (requires OctoMap)
ros2 launch uav_mapping mapping.launch.py
ros2 launch uav_planner_interface planner.launch.py

# Terminal 3 — local planner + waypoint manager
ros2 launch uav_local_planner local_planner.launch.py

# Terminal 4 — flight controller
ros2 launch uav_control control.launch.py
```

> If you are not using the global A* planner (e.g. sending direct waypoints), terminals for `uav_mapping` and `uav_planner_interface` can be skipped entirely.

### With legacy VFH3D planner

```bash
ros2 launch uav_local_planner local_planner.launch.py use_mp:=false
```

All other terminals remain the same; `uav_mapping` must be running.

The control node handles the full flight lifecycle automatically:
- **STARTUP** — 5 s EKF2 warmup
- **TAKEOFF** — arms, engages offboard, climbs to 5 m
- **HOVER** — holds position, waits for a navigation goal
- **AUTONOMOUS** — executes local planner velocity commands
- **HOVER** — returns to hover on mission complete

---

## Sending Goals

Once the drone is in HOVER state, send a navigation goal via the action interface:

```bash
ros2 action send_goal /uav/navigate_to_goal \
  uav_planner_interface/action/NavigateToGoal \
  "{target_pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 2.0, z: 3.0}, orientation: {w: 1.0}}}, planning_timeout_sec: 15.0}"
```

Goals are in the `map` frame (ENU). The planner will return `SUCCESS`, `NO_PATH`, `TIMEOUT`, or `INVALID_GOAL`.

---

## Key Topics

| Topic | Type | Description |
|---|---|---|
| `/drone/tof_merged/points` | `PointCloud2` | Fused 5× ToF cloud in `base_link` frame |
| `/octomap_binary` | `Octomap` | Live 3D occupancy map (VFH3D and global planner) |
| `/octomap_occupied_cells_vis_array` | `MarkerArray` | RViz2 visualisation of OctoMap |
| `/uav/global_path` | `Path` | A* planned path in `map` frame |
| `/uav/current_waypoint` | `PointStamped` | Active waypoint fed to local planner |
| `/uav/cmd_vel` | `TwistStamped` | Local planner velocity output (ENU) |
| `/uav/vfh_status` | `String` | `IDLE` / `NOMINAL` / `AVOIDING` / `ESTOP` |
| `/uav/mission_complete` | `Bool` | Fires `true` when final waypoint reached |
| `/drone/odom` | `Odometry` | Drone pose in `map` frame (ENU) |

---

## RViz2 Setup

Set **Fixed Frame** to `map`, then add:

- `PointCloud2` → `/drone/tof_merged/points` — colour by Z, Reliability: Reliable
- `MarkerArray` → `/octomap_occupied_cells_vis_array` (VFH3D mode only)
- `Path` → `/uav/global_path`
- `Odometry` → `/drone/odom`
- `TF` — visualise sensor frames on drone body

---

## Parameters

### uav_depth_fusion
| Parameter | Default | Description |
|---|---|---|
| `target_frame` | `base_link` | Frame to merge clouds into |
| `tf_timeout_sec` | `0.1` | TF lookup timeout |

### uav_mapping
| Parameter | Default | Description |
|---|---|---|
| `resolution` | `0.2` | OctoMap voxel size (metres) |
| `sensor_model_max_range` | `3.0` | Max sensor range (A010 = 3 m) |
| `transform_tolerance` | `0.2` | TF timestamp tolerance |

### uav_planner_interface
| Parameter | Default | Description |
|---|---|---|
| `planner_plugin` | `uav_planning::AStarPlanner` | Plugin class name |
| `robot_radius` | `0.30` | F450 radius with props (metres) |
| `inflation_radius` | `0.15` | Extra obstacle clearance |
| `max_planning_time` | `10.0` | A* timeout (seconds) |

### uav_local_planner — Motion Primitive (`mp_node`)
| Parameter | Default | Description |
|---|---|---|
| `num_primitives` | `72` | Number of horizontal directions (360° / 5°) |
| `primitive_length` | `2.0` | Ray length in metres (matches A010 range) |
| `collision_radius` | `0.45` | Point-to-ray clearance threshold (robot + safety margin) |
| `max_speed` | `2.5` | Max horizontal speed (m/s) |
| `min_speed` | `0.2` | Min speed near obstacles (m/s) |
| `max_vz` | `1.0` | Max vertical speed (m/s) |
| `min_clearance` | `0.6` | E-stop distance (metres) |
| `w_goal` | `3.0` | Cost weight — angular distance to goal direction |
| `w_prev` | `2.0` | Cost weight — angular distance to previous primitive |
| `alt_kp` | `0.8` | Altitude P-gain for vertical velocity |
| `update_rate_hz` | `20.0` | Control loop rate |

### uav_local_planner — VFH3D (`vfh3d_node`, legacy)
| Parameter | Default | Description |
|---|---|---|
| `az_sectors` / `el_sectors` | `72` / `36` | Histogram azimuth and elevation bins |
| `bbox_radius` | `3.0` | Local OctoMap bounding box radius (metres) |
| `h_high` / `h_low` | `0.5` / `0.2` | Histogram occupancy thresholds (hysteresis) |
| `w_goal` / `w_current` / `w_prev` | `3.0` / `1.0` / `2.0` | Direction cost weights |
| `max_speed` | `2.5` | Max horizontal speed (m/s) |
| `min_clearance` | `0.6` | E-stop distance (metres) |

### waypoint_manager
| Parameter | Default | Description |
|---|---|---|
| `acceptance_radius` | `1.0` | Waypoint acceptance radius (metres) |
| `final_acceptance_radius` | `0.8` | Final waypoint acceptance radius |
| `replan_deviation` | `2.0` | Distance from path before requesting replan |

### uav_control
| Parameter | Default | Description |
|---|---|---|
| `takeoff_height` | `-2.0` | Takeoff altitude NED (negative = up) |
| `startup_delay_s` | `5.0` | EKF2 warmup delay (seconds) |
| `cmd_timeout_s` | `0.5` | Stale `cmd_vel` fallback to hover |

---

## Adding a New Global Planner Plugin

The global planner is plugin-based via `pluginlib`. To add a custom planner (e.g. RRT*, ML policy):

1. Create a class inheriting `uav_planning::GlobalPlannerBase`
2. Implement `initialize()`, `computePath()`, and `name()`
3. Register with `PLUGINLIB_EXPORT_CLASS`
4. Add a `planner_plugins.xml` entry
5. Switch at launch time:

```bash
ros2 launch uav_planner_interface planner.launch.py \
  planner_plugin:=my_package::MyPlanner
```

No changes to any other node required.

---

## TF Tree

```
map
└── odom                          (static identity — replace with SLAM)
    └── base_link                 (PX4 odometry, NED→ENU converted)
        ├── tof_array_link        (identity joint)
        │   ├── tof_0_link        (front, 0°)
        │   ├── tof_1_link        (front-left, 72°)
        │   ├── tof_2_link        (rear-left, 144°)
        │   ├── tof_3_link        (rear-right, -144°)
        │   └── tof_4_link        (front-right, -72°)
        ├── rgbd_cam_link         (front RGBD, 0.2 m forward)
        └── bottom_cam_link       (downward, pitched 90°)
```

---

## Known Limitations & Future Work

- `map → odom` is a static identity transform. Replace with a SLAM or VIO solution (e.g. RTAB-Map, ORB-SLAM3) for drift-free long-range navigation.
- The Motion Primitive planner uses horizontal-only primitives matching the A010 sensor ring. Overhead and underfoot obstacles are not detected; altitude hold is the only vertical protection.
- The A* planner treats unknown voxels as free. In highly unstructured environments consider switching to a conservative unknown=occupied policy once the map is sufficiently explored.
- The global planner plugin interface is ready for ML-based planners — the `computePath()` signature accepts the full OctoMap and a feedback callback.

---

## License

MIT
