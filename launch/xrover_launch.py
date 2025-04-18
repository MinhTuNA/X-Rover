import launch
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", "0"),
            Node(
                package="xrover",  # Tên gói
                executable="um982_node",  # Tên node
                name="um982",
                output="screen",  # Hiển thị đầu ra trên màn hình
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
                executable="motor_controller_node",
                name="motor_controller",
                output="screen",
                parameters=[],
            ),
            Node(
                package="xrover",  # Tên
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
           
        ]
    )
