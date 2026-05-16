
# UAV VLM Exploration



VLM-enriched UAV exploration and navigation built on PX4 SITL + ROS2 Humble + Gazebo Harmonic.



Implements See Point Fly (SPF) (CoRL 2025) with Frontier-Based Exploration (FBE) enriched by Vision-Language Models.



## Kill Everything



pkill -f "px4|gz|MicroXRCE|ros2|singularity|xterm" 2>/dev/null

sleep 8



## Step 1 - Launch Simulation



cd ~/irobot/px4_sim && bash run_server.sh e1547056



Wait 90 seconds for all 5 xterms to boot.



## Step 2 - Launch VLM Navigation



Terminal A - VLM + User Interface:



singularity exec --nv ~/irobot/uav_stack.sif bash -c "

source /opt/ros/humble/setup.bash &&

source ~/irobot/px4_ros2_ws/install/setup.bash &&

source ~/irobot/planner_ws/install/setup.bash &&

export OPENAI_API_KEY=$OPENAI_API_KEY &&

export OLLAMA_API_KEY=$OLLAMA_API_KEY &&

ros2 launch uav_vlm vlm.launch.py

"



Terminal B - SPF Orchestrator:



singularity exec --nv ~/irobot/uav_stack.sif bash -c "

source /opt/ros/humble/setup.bash &&

source ~/irobot/px4_ros2_ws/install/setup.bash &&

source ~/irobot/planner_ws/install/setup.bash &&

ros2 launch uav_global_planner spf_orchestrator.launch.py

"



Terminal C - RViz:



singularity exec --nv ~/irobot/uav_stack.sif bash -c "

source /opt/ros/humble/setup.bash &&

source ~/irobot/px4_ros2_ws/install/setup.bash &&

source ~/irobot/planner_ws/install/setup.bash &&

export DISPLAY=$DISPLAY &&

rviz2

"



## Step 3 - Autonomous Exploration



Terminal D - Frontier Explorer:



singularity exec --nv ~/irobot/uav_stack.sif bash -c "

source /opt/ros/humble/setup.bash &&

source ~/irobot/px4_ros2_ws/install/setup.bash &&

source ~/irobot/planner_ws/install/setup.bash &&

ros2 launch uav_exploration explorer.launch.py

"



Terminal E - VLM Frontier Selector:



singularity exec --nv ~/irobot/uav_stack.sif bash -c "

source /opt/ros/humble/setup.bash &&

source ~/irobot/px4_ros2_ws/install/setup.bash &&

source ~/irobot/planner_ws/install/setup.bash &&

export OPENAI_API_KEY=$OPENAI_API_KEY &&

python3 ~/irobot/planner_ws/uav_exploration/src/vlm_frontier_selector.py

"



Terminal F - Metrics Logger:



singularity exec --nv ~/irobot/uav_stack.sif bash -c "

source /opt/ros/humble/setup.bash &&

source ~/irobot/px4_ros2_ws/install/setup.bash &&

source ~/irobot/planner_ws/install/setup.bash &&

python3 ~/irobot/planner_ws/uav_exploration/src/exploration_metrics.py \

  --ros-args -p run_name:=vlm_run

"



## VLM Interface Usage



After Terminal A starts you will see:



Hi! I am your inspection drone.

How can I help you today?



You:



Example commands:

You: fly to the yellow cylinder

You: find the red box

You: what do you see

You: go forward

You: move left 2



## System Architecture



User types instruction

        |

        v

user_instruction_node

publishes /user/instruction

        |

        v

vlm_spatial_grounding

captures camera + depth image

sends to GPT-4o with instruction

GPT-4o returns 2D pixel (u,v) + depth estimate

pinhole camera model projects to 3D world pose

publishes /spf/target_pose

        |

        v

spf_orchestrator

receives /spf/target_pose

calls /uav/navigate_to_goal action

publishes /uav/global_planner_status

        |

        v

planner_server_node

spf_direct_mode=true skips A* emits one-pose path

publishes /uav/global_path

        |

        v

waypoint_manager_node SPF-D FSM

ROTATING phase: publishes hover wp until yaw aligned

TRANSLATING phase: publishes target wp until reached

COMPLETE: publishes mission_complete

        |

        v

mp_node

ROTATING: publishes zero cmd_vel

TRANSLATING: runs motion primitives + obstacle avoidance

publishes /uav/cmd_vel

        |

        v

setpoint_publisher_node SPF-D ROTATING state

ROTATING: velocity=0 yaw=target_yaw

AUTONOMOUS: forwards cmd_vel as velocity

publishes /fmu/in/trajectory_setpoint

        |

        v

MicroXRCE-DDS Agent

        |

        v

PX4 SITL

        |

        v

Gazebo Harmonic - drone moves



## Benchmark Results



Baseline pure FBE:

Distance: 102.7m

VLM calls: 0

Time: 500s



FBE + VLM:

Distance: 87.46m

VLM calls: 36

Time: 525s

Improvement: 14.8% more efficient



## API Speed Results



Ollama gemma3:4b  - 0.69s  FASTEST used for scans

OpenAI gpt-4o-mini - 0.95s

OpenAI gpt-4o    - 1.09s

Ollama gemma4:31b - 1.31s

Ollama gpt-oss:120b - NO VISION SUPPORT



## C++ Changes Made



waypoint_manager_node.cpp

Added SPFPhase enum IDLE ROTATING TRANSLATING COMPLETE

Added /uav/mission_phase publisher

Added yaw_alignment_threshold_deg parameter default 15 degrees

Added yaw_alignment_hold_cycles parameter default 10 cycles

FSM rotates first if yaw error exceeds threshold



mp_node.cpp

Added /uav/mission_phase subscriber

During ROTATING publishes zero cmd_vel

During TRANSLATING normal obstacle avoidance



setpoint_publisher_node.cpp

Added FlightState ROTATING

During ROTATING velocity=0 position=NaN yaw=target_yaw

Increased cmd_timeout_s from 0.5 to 1.0



planner_server_node.cpp

Added spf_direct_mode parameter default true

Skips A* and emits single pose path directly



## Known Issues



ESTOP blocks movement

Cause: mp_node stops when obstacle within 0.47m

Workaround: restart sim or send drone away



Drone stuck after task complete

Cause: waypoint_manager keeps publishing last waypoint

Workaround: send new instruction



VLM hallucination

Cause: Gazebo lighting causes misidentification

Workaround: retry from different position



cmd_vel timeout

Cause: setpoint_publisher reverts to HOVER after 1s

Workaround: continuous waypoint publishing



## References



See Point Fly: https://arxiv.org/abs/2509.22653

FBE: Yamauchi 1997



## Authors



Ahmed e1547056 - VLM integration SPF implementation exploration metrics

Shantam - System architecture cuVSLAM semantic layer design

