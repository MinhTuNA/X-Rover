#!/usr/bin/env python3
import json
import rclpy
import os
from enum import Enum
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32, String, Int32
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from .lib.ConstVariable import COMMON
from .lib.NavigationController import NavigationController
from .VariableManager import instance
from .lib.ControlLib import ControlLib

current_path = os.path.dirname(os.path.abspath(__file__))


class Mode(Enum):
    GPS = 0
    CAMERA = 1
    FUSION = 2
    CONTROL = 3


class ControlMode(Enum):
    ROVER = 1000
    DELTA = 2000


class Navigation(Node):
    def __init__(self):
        super().__init__("navigation_node")
        self.start_lat = None
        self.start_lon = None
        self.goal_lat = None
        self.goal_lon = None
        self.current_lat = None
        self.current_lon = None
        self.current_heading = None
        self.is_running = False
        self.is_done = False
        self.mode = None
        self.control_x = 0
        self.control_z = 0
        self.current_swa = None
        self.control_mode = None
        self.control_lib = ControlLib()
        self.navigation_controller = NavigationController()
        instance.load(COMMON.variable_path)
        self.load_variable()
        self.get_logger().info("navigation node initialized")
        self.cmd_vel_pub = self.create_publisher(Twist, "/rover/vel", 10)
        self.rover_status_pub = self.create_publisher(String, "/rover/move_status", 10)
        self.create_subscription(NavSatFix, "/gps/goal", self.goal_callback, 10)
        self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, 10)
        self.create_subscription(Float32, "/gps/heading", self.heading_callback, 10)
        self.create_subscription(String, "/start", self.start_callback, 10)
        self.create_subscription(String, "/stop", self.stop_callback, 10)
        self.create_subscription(String, "/mode", self.mode_callback, 10)
        self.create_subscription(Int32, "fs_i6/ch1", self.control_x_callback, 10)
        self.create_subscription(Int32, "fs_i6/ch0", self.control_z_callback, 10)
        self.create_subscription(Int32, "fs_i6/swd", self.control_mode_callback, 10)
        self.create_service(Trigger, "/rover/get/mode", self.get_mode_callback)

        self.timer = self.create_timer(0.05, self.navigate)

    def get_mode_callback(self, request, response):
        response.success = True
        response.message = str(self.mode.value)
        return response

    def load_variable(self):
        mode = instance.get("mode", 3)
        self.mode = Mode(int(mode))
        self.get_logger().info(f"mode: {self.mode}")

    def goal_callback(self, msg):
        self.goal_lat = msg.latitude
        self.goal_lon = msg.longitude

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def heading_callback(self, msg):
        self.current_heading = msg.data

    def start_callback(self, msg):
        self.is_running = True
        self.is_done = False

    def stop_callback(self, msg):
        self.is_running = False

    def mode_callback(self, msg):
        self.mode = Mode(int(msg.data))
        instance.set("mode", int(msg.data))
        instance.save()
        self.get_logger().info(f"mode: {self.mode}")

    def set_twist(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)

    def control_x_callback(self, msg):
        if self.control_mode == ControlMode.ROVER:
            ch2_value = int(msg.data)
            new_x = self.control_lib.rover_x(ch2_value)
            self.control_x = new_x
            # self.get_logger().info(f"x: {self.control_x}")
        elif self.control_mode == ControlMode.DELTA:
            self.control_x = 0

    def control_z_callback(self, msg):
        if self.control_mode == ControlMode.ROVER:
            ch3_value = int(msg.data)
            new_z = self.control_lib.rover_z(ch3_value)
            self.control_z = new_z
        elif self.control_mode == ControlMode.DELTA:
            self.control_z = 0

    def control_mode_callback(self, msg):
        # self.get_logger().info(f"control_mode: {msg.data}")
        if self.current_swa == int(msg.data):
            return
        self.current_swa = int(msg.data)
        if msg.data == 1000:
            self.control_mode = ControlMode.ROVER
            self.get_logger().info("control mode: rover")
        elif msg.data == 2000:
            self.control_mode = ControlMode.DELTA
            self.get_logger().info("control mode: delta")
        else:
            self.get_logger().info("invalid control mode")

    def detect_obstacle(self):

        return False  # Ví dụ: không có vật cản

    def avoid_obstacle(self):
        """Điều khiển robot tránh vật cản."""
        self.set_twist(0, 0.5)

    def navigate(self):

        if self.mode == Mode.GPS:
            if not self.is_running:
                return
            if self.goal_lat is None or self.goal_lon is None:
                self.get_logger().info("waiting for goal")
                return

            if self.current_lat is None or self.current_lon is None:
                self.get_logger().info("waiting for gps fix")
                return

            if self.start_lat is None and self.start_lon is None:
                self.start_lat = self.current_lat
                self.start_lon = self.current_lon

            linear_x, angular_z = self.navigation_controller.compute_twist_stanley(
                start_lat=self.start_lat,
                start_lon=self.start_lon,
                current_lat=self.current_lat,
                current_lon=self.current_lon,
                target_lat=self.goal_lat,
                target_lon=self.goal_lon,
                current_heading=self.current_heading,
            )
            if linear_x == 0 and angular_z == 0:
                self.is_done = True
                self.start_lat = self.goal_lat
                self.start_lon = self.goal_lon
            self.set_twist(linear_x, angular_z)

        elif self.mode == Mode.CAMERA:
            pass
        elif self.mode == Mode.FUSION:
            pass
        elif self.mode == Mode.CONTROL:
            # self.get_logger().info(f"x: {self.control_x} z: {self.control_z}")
            self.set_twist(self.control_x, self.control_z)

    def handle_destroy(self):
        self.get_logger().info("navigation node has been stopped")
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    navigation = Navigation()
    try:
        rclpy.spin(navigation)
    except KeyboardInterrupt:
        pass
    finally:
        navigation.handle_destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
