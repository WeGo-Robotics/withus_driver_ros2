"""Start automatic 0x501 requests; the remote A switch selects vehicle mode."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare("withus_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("feedback_timeout_ms", default_value="200"),
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([
            bringup, "config", "withus_driver.yaml"])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup, "launch", "withus_bringup.launch.py"])),
            launch_arguments={
                "can_interface": LaunchConfiguration("can_interface"),
                "feedback_timeout_ms": LaunchConfiguration("feedback_timeout_ms"),
                "params_file": LaunchConfiguration("params_file"),
                "enable_tx": "true",
                "control_profile": "hardware_remote_mode",
            }.items(),
        ),
    ])
