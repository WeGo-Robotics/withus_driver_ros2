"""Run only the vehicle-status client; start withus_bringup separately."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Expose the monitor namespace and reporting parameters."""
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='withus'),
        DeclareLaunchArgument('report_period_sec', default_value='1.0'),
        DeclareLaunchArgument('status_topic_timeout_sec', default_value='0.5'),
        Node(
            package='withus_examples',
            executable='status_monitor',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'report_period_sec': ParameterValue(
                    LaunchConfiguration('report_period_sec'), value_type=float),
                'status_topic_timeout_sec': ParameterValue(
                    LaunchConfiguration('status_topic_timeout_sec'), value_type=float),
            }],
        ),
    ])
