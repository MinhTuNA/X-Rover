#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from .Const import *
from .SensorManager import SensorManager
from .MotionController import MotionController

class Navigation(Node):
    def __init__(self):
        super().__init__("navigation_node")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        self.sensor_manager = SensorManager(self)
        self.motion_controller = MotionController(self, self.sensor_manager)
        self.goal_subscription = self.create_subscription(
            NavSatFix, "/gps/goal", self.goal_callback, 10
        )
        self.goal_subscription
        self.timer = self.create_timer(0.1, self.navigate)

    def goal_callback(self, msg):
        self.goal_lat = msg.latitude
        self.goal_lon = msg.longitude

    def detect_obstacle(self):

        return False  # Ví dụ: không có vật cản

    def avoid_obstacle(self):
        """Điều khiển robot tránh vật cản."""
        twist = Twist()
        twist.angular.z = 0.5
        self.cmd_vel_pub.publish(twist)

    def navigate(self):
        if self.goal_lat is None or self.goal_lon is None:
            self.get_logger().info("Chưa có mục tiêu")
            return
        if self.sensor_manager.current_lat is None or self.sensor_manager.current_lon is None:
            self.get_logger().info("Chưa có vị trí hiện tại")
            return
        if self.detect_obstacle():
            self.avoid_obstacle()
            return
        twist = self.motion_controller.compute_twist(self.goal_lat, self.goal_lon)
        self.motion_controller.publish_twist(twist)


def main(args=None):
    rclpy.init(args=args)
    navigation = Navigation()
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
