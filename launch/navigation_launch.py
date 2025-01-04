import launch
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", "2"),
            Node(
                package="xrover",  # Tên gói
                executable="navigation_node",  # Tên node
                name="navigation",
                output="screen",  # Hiển thị đầu ra trên màn hình
                parameters=[],
            ),
        ]
    )
