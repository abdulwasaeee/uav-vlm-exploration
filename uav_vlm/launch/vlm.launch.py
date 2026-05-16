from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_api", default_value="true"),
        Node(
            package="uav_vlm",
            executable="vlm_spatial_grounding",
            name="vlm_spatial_grounding",
            output="screen",
            remappings=[
                ("/drone/rgbd/image",  "/drone/rgbd/image"),
                ("/drone/rgbd/depth",  "/drone/rgbd/depth"),
                ("/drone/odom",        "/drone/odom"),
            ],
        ),
        Node(
            package="uav_vlm",
            executable="user_instruction_node",
            name="user_instruction_node",
            output="screen",
        ),
    ])
