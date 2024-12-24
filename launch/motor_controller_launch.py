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
                executable="motor_controller_node",  # Tên của console_script (được định nghĩa trong setup.py)
                name="motor_controller",  # Tên node
                output="screen",  # Hiển thị đầu ra trên màn hình
                parameters=[],
            ),
        ]
    )
