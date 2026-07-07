from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='geometry_overlap_py',
            executable='geometry_provider_node',
            name='geometry_provider_node',
            output='screen',
        ),
        Node(
            package='geometry_overlap_py',
            executable='overlap_checker_node',
            name='overlap_checker_node',
            output='screen',
        ),
    ])
