from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("feedback_timeout_ms", default_value="200"),
        DeclareLaunchArgument("enable_tx", default_value="true"),
        DeclareLaunchArgument("control_profile", default_value="hardware_drive",
                              description="hardware_drive or vcan_reference",
                              choices=["hardware_drive", "vcan_reference"]),
        DeclareLaunchArgument("params_file", default_value=PathJoinSubstitution([
            FindPackageShare("withus_bringup"), "config", "withus_driver.yaml"])),
    ]
    overrides = {
        "can_interface": ParameterValue(LaunchConfiguration("can_interface"), value_type=str),
        "feedback_timeout_ms": ParameterValue(LaunchConfiguration("feedback_timeout_ms"), value_type=int),
        "enable_tx": ParameterValue(LaunchConfiguration("enable_tx"), value_type=bool),
        "control_profile": ParameterValue(LaunchConfiguration("control_profile"), value_type=str),
    }
    return LaunchDescription(arguments + [Node(
        package="withus_driver", executable="withus_driver_node",
        name="withus_driver", namespace="withus", output="screen",
        parameters=[LaunchConfiguration("params_file"), overrides],
        # Node shutdown budget is at most 10.5 seconds, plus RX join.
        sigterm_timeout="12", sigkill_timeout="3",
    )])
