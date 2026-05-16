from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_control',
            executable='setpoint_publisher_node',
            name='setpoint_publisher',
            output='screen',
            parameters=[{
                'publish_rate_hz':       20.0,
                'yaw_speed':             0.5,
                'hover_on_complete':     True,
                'vehicle_status_topic':  '/fmu/out/vehicle_status_v2',
            }],
        ),
    ])