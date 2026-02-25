from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tank_base_controller',
            executable='controller_node',
            name='tank_base_controller',
            output='screen',
            parameters=[
                {'wheelbase': 0.1368},
                {'track_width': 0.1446},
                {'wheel_diameter': 0.065}
            ]
        )
    ])
