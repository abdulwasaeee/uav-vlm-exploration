from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_exploration',
            executable='frontier_explorer_node',
            name='frontier_explorer',
            output='screen',
            parameters=[{
                # Map bounds — match your SDF world
                'map_xmin': -10.0,
                'map_xmax':  10.0,
                'map_ymin': -10.0,
                'map_ymax':  10.0,
                'map_zmin':   0.3,
                'map_zmax':   4.0,  # below hanging beam at 4.2m

                # Two altitude layers — 2.0m clears inflated floor, 3.5m upper layer
                'explore_altitudes': [2.0, 3.5],

                # Frontier detection
                'min_frontier_size': 5,     # ignore tiny clusters
                'cluster_radius':    1.0,   # metres — cluster merge radius
                'scan_interval_s':   3.0,   # how often to rescan for frontiers
                'goal_timeout_s':   20.0,   # A* timeout per goal

                # Navigation
                'min_goal_distance': 1.0,   # don't send goals closer than this

                # Scoring weights — VLM extension points
                'w_size':     1.0,   # reward large unexplored regions
                'w_distance': 0.5,   # penalise distant frontiers
                'w_altitude': 0.3,   # prefer current altitude layer
            }],
        ),
    ])