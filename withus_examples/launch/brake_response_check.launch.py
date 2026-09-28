"""Run only the bounded stationary brake client against an existing driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Require an explicit brake percentage for every run."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'brake_percent',
            description='Required brake request, finite percentage from 0 to 100.'),
        DeclareLaunchArgument(
            'duration', default_value='1.0',
            description='Publish duration in seconds, greater than 0 and at most 1.5.'),
        DeclareLaunchArgument('namespace', default_value='withus'),
        Node(
            package='withus_examples',
            executable='brake_response_check',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            arguments=[
                '--brake-percent', LaunchConfiguration('brake_percent'),
                '--duration', LaunchConfiguration('duration'),
                '--namespace', LaunchConfiguration('namespace'),
            ],
        ),
    ])
