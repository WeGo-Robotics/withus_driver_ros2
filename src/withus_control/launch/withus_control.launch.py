# control_launcher.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'gui_control',
            default_value='False',
            description='Whether to launch the GUI control node'
        ),
        Node(
            package='withus_control',  # Replace with the actual name of your ROS 2 package
            executable='send_command',
            output='screen',
        ),
        # Node(
        #     package='withus_control',  # Replace with the actual name of your ROS 2 package
        #     executable='pyqt_control',
        #     output='screen',
        #     condition=IfCondition(LaunchConfiguration('gui_control'))
        # ),
        Node(
            package='withus_control',  # Replace with the actual name of your ROS 2 package
            executable='recv_status',
            output='screen',
            condition=UnlessCondition(LaunchConfiguration('gui_control'))
        ),
    ],
    )