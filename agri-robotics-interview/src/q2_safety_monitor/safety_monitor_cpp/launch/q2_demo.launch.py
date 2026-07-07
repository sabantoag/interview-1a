from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='safety_monitor_cpp',
            executable='telemetry_publisher_node',
            name='telemetry_publisher_node',
            output='screen',
        ),
        Node(
            package='safety_monitor_cpp',
            executable='safety_monitor_node',
            name='safety_monitor_node',
            output='screen',
        ),
    ])
