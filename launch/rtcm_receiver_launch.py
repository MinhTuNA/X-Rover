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
                executable="rtcm_receiver_node",
                name="rtcm_receiver", 
                output="screen", 
                parameters=[],
            ),
        ]
    )
