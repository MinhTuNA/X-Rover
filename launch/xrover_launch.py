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
                executable="navigation_node",
                name="navigation",
                output="screen",
                parameters=[],
            ),
            Node(
                package="xrover",
                # Tên của console_script (được định nghĩa trong setup.py)
                executable="connect_server_node",
                name="connect_server",
                output="screen",
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
                package='xrover',
                executable='motor_controller_node',
                name='motor_controller',
                output='screen',
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
