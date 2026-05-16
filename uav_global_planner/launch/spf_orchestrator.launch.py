
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="uav_global_planner",
            executable="spf_orchestrator",
            name="spf_orchestrator",
            output="screen",
        ),
    ])
