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
                executable="execute_program_node",  # Tên của console_script (được định nghĩa trong setup.py)
                name="execute_program",  # Tên node
                output="screen",  # Hiển thị đầu ra trên màn hình
                parameters=[],
            ),
        ]
    )
