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
            # Node(
            #     package="xrover",
            #     executable="signal_light_node",
            #     name="signal_light",
            #     output="screen",
            #     parameters=[],
            # ),
        ]
    )
