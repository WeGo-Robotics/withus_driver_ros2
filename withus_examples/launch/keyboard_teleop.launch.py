"""Run the keyboard client in this terminal against a separately started driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Expose keyboard preferences while fetching control limits from the driver."""
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='withus'),
        DeclareLaunchArgument(
            'key_timeout_sec', default_value='1.0',
            description='Stop publishing drive commands after this many seconds without input.'),
        DeclareLaunchArgument('initial_speed', default_value='0.1'),
        DeclareLaunchArgument('speed_step', default_value='0.1'),
        DeclareLaunchArgument(
            'left_steering_sign', default_value='1',
            description='CAN steering sign for a: 1 or -1; confirm the physical direction.'),
        Node(
            package='withus_examples',
            executable='keyboard_teleop',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            # The client opens /dev/tty itself. emulate_tty does not forward stdin.
            arguments=[
                '--namespace', LaunchConfiguration('namespace'),
                '--key-timeout-sec', LaunchConfiguration('key_timeout_sec'),
                '--initial-speed', LaunchConfiguration('initial_speed'),
                '--speed-step', LaunchConfiguration('speed_step'),
                '--left-steering-sign', LaunchConfiguration('left_steering_sign'),
            ],
        ),
    ])
