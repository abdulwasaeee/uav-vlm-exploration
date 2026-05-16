import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_mp             = LaunchConfiguration('use_mp',             default='true')
    with_global_planner = LaunchConfiguration('with_global_planner', default='true')
    with_vlm            = LaunchConfiguration('with_vlm',            default='false')
    with_spf            = LaunchConfiguration('with_spf',            default='false')

    fusion_dir  = get_package_share_directory('uav_depth_fusion')
    mapping_dir = get_package_share_directory('uav_mapping')
    planner_dir = get_package_share_directory('uav_planner_interface')
    local_dir   = get_package_share_directory('uav_local_planner')
    control_dir = get_package_share_directory('uav_control')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_mp', default_value='true',
            description='true = Motion Primitive planner, false = legacy VFH3D'),

        DeclareLaunchArgument(
            'with_global_planner', default_value='true',
            description='Launch OctoMap + A* global planner (required for nav goals)'),
        DeclareLaunchArgument(
            'with_vlm', default_value='false',
            description='Launch VLM spatial grounding + user instruction interface'),
        DeclareLaunchArgument(
            'with_spf', default_value='false',
            description='Launch SPF orchestrator (requires with_vlm:=true)'),

        # ── T = 0 s : Sensor fusion ───────────────────────────────────────
        # Must come first: provides /drone/tof_merged/points, /drone/odom, TF tree.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(fusion_dir, 'launch', 'fusion.launch.py'))),

        # ── T = 3 s : Local planner + waypoint manager ────────────────────
        # Needs /drone/odom and /drone/tof_merged/points from fusion.
        # mp_node reads config from uav_local_planner/config/mp_params.yaml.
        TimerAction(period=3.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(local_dir, 'launch', 'local_planner.launch.py')),
                launch_arguments={'use_mp': use_mp}.items())]),

        # ── T = 3 s : OctoMap (optional) ─────────────────────────────────
        # Skip with with_global_planner:=false when testing reactive avoidance only.
        TimerAction(period=3.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mapping_dir, 'launch', 'mapping.launch.py')),
                condition=IfCondition(with_global_planner))]),

        # ── T = 7 s : Global planner (optional) ──────────────────────────
        # Waits for OctoMap to populate before accepting planning requests.
        TimerAction(period=7.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(planner_dir, 'launch', 'planner.launch.py')),
                condition=IfCondition(with_global_planner))]),

        # ── T = 5 s : Flight controller ───────────────────────────────────
        # Has internal STARTUP delay (EKF2 warmup); safe to launch early.
        TimerAction(period=5.0, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(control_dir, 'launch', 'control.launch.py')))]),
    ])
