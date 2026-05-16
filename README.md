# UAV VLM Exploration

VLM-enriched UAV exploration and navigation built using:

* PX4 SITL
* ROS2 Humble
* Gazebo Harmonic

This project implements:

* **See Point Fly (SPF)** — CoRL 2025
* **Frontier-Based Exploration (FBE)**
* Vision-Language-Model-guided autonomous navigation



---

# Quick Start

## 0. Kill Existing Processes

```bash
pkill -f "px4|gz|MicroXRCE|ros2|singularity|xterm" 2>/dev/null
sleep 8
```

---

# 1. Launch Simulation

```bash
cd ~/irobot/px4_sim
bash run_server.sh e1547056
```

Wait approximately **90 seconds** for all xterms to initialize.

---

# 2. Launch Core VLM Navigation Stack

## Terminal A — VLM Interface + User Interaction

```bash
singularity exec --nv ~/irobot/uav_stack.sif bash -c "
source /opt/ros/humble/setup.bash &&
source ~/irobot/px4_ros2_ws/install/setup.bash &&
source ~/irobot/planner_ws/install/setup.bash &&

export OPENAI_API_KEY=$OPENAI_API_KEY &&
export OLLAMA_API_KEY=$OLLAMA_API_KEY &&

ros2 launch uav_vlm vlm.launch.py
"
```

---

## Terminal B — SPF Orchestrator

```bash
singularity exec --nv ~/irobot/uav_stack.sif bash -c "
source /opt/ros/humble/setup.bash &&
source ~/irobot/px4_ros2_ws/install/setup.bash &&
source ~/irobot/planner_ws/install/setup.bash &&

ros2 launch uav_global_planner spf_orchestrator.launch.py
"
```

---

## Terminal C — RViz Visualization

```bash
singularity exec --nv ~/irobot/uav_stack.sif bash -c "
source /opt/ros/humble/setup.bash &&
source ~/irobot/px4_ros2_ws/install/setup.bash &&
source ~/irobot/planner_ws/install/setup.bash &&

export DISPLAY=$DISPLAY &&
rviz2
"
```

---

# 3. Launch Autonomous Exploration Modules

## Terminal D — Frontier Explorer

```bash
singularity exec --nv ~/irobot/uav_stack.sif bash -c "
source /opt/ros/humble/setup.bash &&
source ~/irobot/px4_ros2_ws/install/setup.bash &&
source ~/irobot/planner_ws/install/setup.bash &&

ros2 launch uav_exploration explorer.launch.py
"
```

---

## Terminal E — VLM Frontier Selector

```bash
singularity exec --nv ~/irobot/uav_stack.sif bash -c "
source /opt/ros/humble/setup.bash &&
source ~/irobot/px4_ros2_ws/install/setup.bash &&
source ~/irobot/planner_ws/install/setup.bash &&

export OPENAI_API_KEY=$OPENAI_API_KEY &&

python3 ~/irobot/planner_ws/uav_exploration/src/vlm_frontier_selector.py
"
```

---

## Terminal F — Metrics Logger

```bash
singularity exec --nv ~/irobot/uav_stack.sif bash -c "
source /opt/ros/humble/setup.bash &&
source ~/irobot/px4_ros2_ws/install/setup.bash &&
source ~/irobot/planner_ws/install/setup.bash &&

python3 ~/irobot/planner_ws/uav_exploration/src/exploration_metrics.py \
  --ros-args -p run_name:=vlm_run
"
```

---

# VLM User Interface

After launching Terminal A:

```text
Hi! I am your inspection drone.
How can I help you today?
```

## Example Commands

```text
fly to the yellow cylinder
find the red box
what do you see
go forward
move left 2
```

---

# System Architecture

```text
User Instruction
      │
      ▼
user_instruction_node
      │
Publishes: /user/instruction
      │
      ▼
vlm_spatial_grounding
      │
Captures RGB + depth image
Queries GPT-4o
Returns 2D pixel + depth
Projects target into 3D world pose
Publishes: /spf/target_pose
      │
      ▼
spf_orchestrator
      │
Calls navigation action server
Publishes: /uav/global_planner_status
      │
      ▼
planner_server_node
      │
SPF direct mode skips A*
Publishes single-pose path
Publishes: /uav/global_path
      │
      ▼
waypoint_manager_node
      │
FSM States:
- ROTATING
- TRANSLATING
- COMPLETE
      │
      ▼
mp_node
      │
ROTATING:
    publishes zero cmd_vel

TRANSLATING:
    obstacle avoidance +
    motion primitives
      │
Publishes: /uav/cmd_vel
      │
      ▼
setpoint_publisher_node
      │
Publishes:
    /fmu/in/trajectory_setpoint
      │
      ▼
MicroXRCE-DDS Agent
      │
      ▼
PX4 SITL
      │
      ▼
Gazebo Harmonic
```

---

# Benchmark Results

| Configuration | Distance | VLM Calls | Time  |
| ------------- | -------- | --------- | ----- |
| Pure FBE      | 102.7 m  | 0         | 500 s |
| FBE + VLM     | 87.46 m  | 36        | 525 s |

### Result

* **14.8% improvement in exploration efficiency**
* Slight increase in runtime due to VLM inference overhead

---

# API Speed Comparison

| Model               | Latency           |
| ------------------- | ----------------- |
| Ollama gemma3:4b    | 0.69 s            |
| OpenAI gpt-4o-mini  | 0.95 s            |
| OpenAI gpt-4o       | 1.09 s            |
| Ollama gemma4:31b   | 1.31 s            |
| Ollama gpt-oss:120b | No vision support |

---

# Core C++ Modifications

## `waypoint_manager_node.cpp`

Added:

* `SPFPhase` FSM

  * `IDLE`
  * `ROTATING`
  * `TRANSLATING`
  * `COMPLETE`

New features:

* `/uav/mission_phase` publisher
* Yaw alignment threshold parameter
* Rotation-first behavior before translation

---

## `mp_node.cpp`

Added:

* `/uav/mission_phase` subscriber

Behavior:

* `ROTATING` → publishes zero velocity
* `TRANSLATING` → normal obstacle avoidance

---

## `setpoint_publisher_node.cpp`

Added:

* `FlightState::ROTATING`

Changes:

* Velocity forced to zero during yaw alignment
* Increased `cmd_timeout_s`

  * `0.5 → 1.0`

---

## `planner_server_node.cpp`

Added:

```cpp
spf_direct_mode = true
```

Effect:

* Skips A*
* Publishes direct single-pose path

---

# Known Issues

## ESTOP Prevents Movement

**Cause**

```text
mp_node triggers stop when obstacle distance < 0.47 m
```

**Workaround**

* Restart simulation
* Move UAV away from obstacle

---

## Drone Stuck After Mission Completion

**Cause**

```text
waypoint_manager continues publishing last waypoint
```

**Workaround**

* Send a new instruction

---

## VLM Hallucination

**Cause**

```text
Gazebo lighting affects object recognition
```

**Workaround**

* Retry from another viewpoint

---

## cmd_vel Timeout

**Cause**

```text
setpoint_publisher switches back to HOVER after 1 second
```

**Workaround**

* Continuously publish waypoints

---

# References

## SPF Paper

[See Point Fly (arXiv)](https://arxiv.org/abs/2509.22653?utm_source=chatgpt.com)

## Frontier-Based Exploration

```text
Yamauchi, B. (1997)
A Frontier-Based Approach for Autonomous Exploration
```

---

# Authors

### Ahmed (e1547056)

* VLM integration
* SPF implementation
* Exploration metrics

### Shantam

* System architecture
* cuVSLAM semantic layer
* System design
