import launch
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", "0"),
            Node(
                package="xrover", 
                executable="um982_node",  
                name="um982",
                output="screen", 
                parameters=[],
            ),
            Node(
                package="xrover",
                executable="delta_node",
                name="delta",
                output="screen",
                parameters=[],
            ),
            Node(
                package="xrover",
                executable="connect_server_node",
                name="connect_server", 
                output="screen", 
                parameters=[],
            ),
            Node(
                package="xrover",
                executable="rtcm_receiver_node",
                name="rtcm_receiver",
                output="screen",
                parameters=[],
            ),
            Node(
                package="xrover",
                executable="fs_i6_node",
                name="fs_i6",
                output="screen",
                parameters=[],
            ),
            Node(
                package="xrover",
                executable="navigation_node",
                name="navigation",
                output="screen",
                parameters=[],
            ),
            Node(
                package="xrover",
                executable="load_path_node",
                name="load_path",
                output="screen",
                parameters=[],
            ),
            Node(
                package='xrover', 
                executable='path_logger_node',
                name='path_logger',
                output='screen',
                parameters=[],
            ),
            Node(
                package='xrover', 
                executable='signal_light_node',
                name='signal_light',
                output='screen',
                parameters=[],
            ),
            Node(
                package='xrover',
                executable='s21c_node',
                name='s21c',
                output='screen',
                parameters=[],
            ),
           
        ]
    )
