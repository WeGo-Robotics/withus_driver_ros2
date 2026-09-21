from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    can_interface = LaunchConfiguration("can_interface")

    return LaunchDescription([
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="SocketCAN interface name",
        ),

        Node(
            package="withus_driver",
            executable="withus_driver_node",
            name="withus_driver",
            namespace="withus",
            output="screen",
            parameters=[{
                "can_interface": ParameterValue(
                    can_interface,
                    value_type=str,
                ),
            }],
        ),
    ])