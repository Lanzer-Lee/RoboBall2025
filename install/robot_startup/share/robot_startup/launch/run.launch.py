from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_pkg',
            executable='linear_control_node',
            output='screen',
            parameters=[
                {"control_value": 0.0},
                {"control_mode": "velocity"}
            ]
        ),
        Node(
            package='board_pkg',
            executable='serial_node',
            output='screen',
        )
    ])

