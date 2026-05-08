import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('uav_local_planner')
    mp_config = os.path.join(pkg, 'config', 'mp_params.yaml')

    use_mp = LaunchConfiguration('use_mp', default='true')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_mp',
            default_value='true',
            description='Use Motion Primitive planner (true) or legacy VFH3D (false)'),

        Node(
            package='uav_local_planner',
            executable='waypoint_manager_node',
            name='waypoint_manager',
            output='screen',
            parameters=[{
                'acceptance_radius':       1.0,
                'final_acceptance_radius': 0.25,
                'replan_deviation':        2.0,
                'publish_rate_hz':         20.0,
            }],
        ),

        # ── Motion Primitive planner (default) ─────────���──────────────────
        # All tuning lives in config/mp_params.yaml
        Node(
            condition=IfCondition(use_mp),
            package='uav_local_planner',
            executable='mp_node',
            name='mp_node',
            output='screen',
            parameters=[mp_config],
        ),

        # ── Legacy VFH3D (OctoMap-based) — launch with use_mp:=false ──────
        Node(
            condition=UnlessCondition(use_mp),
            package='uav_local_planner',
            executable='vfh3d_node',
            name='vfh3d_node',
            output='screen',
            parameters=[{
                'az_sectors':     72,
                'el_sectors':     36,
                'bbox_radius':    3.0,
                'robot_radius':   0.30,
                'safety_radius':  0.15,
                'h_high':         0.5,
                'h_low':          0.2,
                'w_goal':         3.0,
                'w_current':      1.0,
                'w_prev':         2.0,
                'max_speed':      2.5,
                'min_speed':      0.2,
                'max_vz':         1.0,
                'min_clearance':  0.6,
                'update_rate_hz': 20.0,
            }],
        ),
    ])
