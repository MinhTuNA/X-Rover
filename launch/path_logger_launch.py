import launch
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", "0"),
        Node(
            package='xrover',
            executable='connect_server_node',
            name='connect_server',
            output='screen',
            parameters=[],
        ),
        Node(
            package='xrover', 
            executable='path_logger_node',
            name='path_logger',
            output='screen',
            parameters=[],
        ),
    ])