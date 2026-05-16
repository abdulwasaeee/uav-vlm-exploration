from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('uav_mapping')
    cfg = os.path.join(pkg, 'config', 'octomap.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('resolution', default_value='0.2'),

        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[cfg],
            remappings=[
                # Feed our merged ToF cloud into octomap_server
                ('cloud_in', '/drone/tof_merged/points'),
            ],
            # SensorDataQoS on the subscription side to match our publisher
            arguments=['--ros-args', '--log-level', 'warn'],
        ),
    ])