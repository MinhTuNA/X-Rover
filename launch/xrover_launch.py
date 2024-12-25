import launch
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable("ROS_DOMAIN_ID", "2"),
            Node(
                package="xrover",
                executable="navigator_node",
                name="navigation_node",
                output="screen",
                parameters=[],
            ),
            Node(
                package="xrover",  # Tên gói
                executable="connect_server_node",  # Tên của console_script (được định nghĩa trong setup.py)
                name="connect_server_node",  # Tên node
                output="screen",  # Hiển thị đầu ra trên màn hình
                parameters=[],
            ),
            Node(
                package="xrover",
                executable="execute_program_node",
                name="execute_program",
                output="screen",
                parameters=[],
            ),
            Node(
                package='xrover',  # Tên gói
                executable='motor_controller_node',  # Tên của console_script (được định nghĩa trong setup.py)
                name='motor_controller',  # Tên node
                output='screen',  # Hiển thị đầu ra trên màn hình
                parameters=[]
            ),
            Node(
                package="xrover",
                executable="um982_node",
                name="um982",
                output="screen",
                parameters=[],
            ),
            Node(
                package="xrover",
                executable="imu_node",
                name="imu",
                output="screen",
                parameters=[],
            ),
            Node(
                package="xrover",
                executable="s21c_node",
                name="s21c",
                output="screen",
                parameters=[],
            ),
        ]
    )
