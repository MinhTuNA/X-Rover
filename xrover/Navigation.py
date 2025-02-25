#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
from .lib.ConstVariable import COMMON
from .lib.NavigationController import NavigationController


class Navigation(Node):
    def __init__(self):
        super().__init__("navigation_node")
        self.goal_lat = None
        self.goal_lon = None
        self.current_lat = None
        self.current_lon = None
        self.current_heading = None
        self.navigation_controller = NavigationController()
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/rover/vel",
            10
        )
        self.create_subscription(
            NavSatFix, "/gps/goal",
            self.goal_callback,
            10
        )
        self.create_subscription(
            NavSatFix,
            "/gps/fix",
            self.gps_callback,
            10
        )
        self.create_subscription(
            Float32,
            "/gps/heading",
            self.heading_callback,
            10
        )
        self.timer = self.create_timer(0.1, self.navigate)

    def goal_callback(self, msg):
        self.goal_lat = msg.latitude
        self.goal_lon = msg.longitude

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def heading_callback(self, msg):
        self.current_heading = msg.data

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
        if self.current_lat is None or self.current_lon is None:
            self.get_logger().info("Chưa có vị trí hiện tại")
            return
        if self.detect_obstacle():
            self.avoid_obstacle()
            return
        twist = self.navigation_controller.compute_twist(
            current_lat=self.current_lat,
            current_lon=self.current_lon,
            target_lat=self.goal_lat,
            target_lon=self.goal_lon,
            current_heading=self.current_heading
        )
        self.cmd_vel_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    navigation = Navigation()
    rclpy.spin(navigation)
    navigation.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
