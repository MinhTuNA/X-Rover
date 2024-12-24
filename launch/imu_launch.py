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
                executable="imu_node",
                name="imu",
                output="screen",
                parameters=[],
            ),
        ]
    )
