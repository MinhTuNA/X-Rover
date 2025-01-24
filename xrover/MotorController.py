#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
from .Const import *
from .Driver import Driver
class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller_node")
        self.get_logger().info("motor controller has been started")
        self.wheel_radius = 0.05
        self.wheel_base = 0.3
        self.driver = Driver()
        self.driver.connect()
        # self.linear_velocity = 0
        # self.angular_velocity = 0
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
            
    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        omega_left, omega_right = self.wheel_speeds(
            linear_velocity=linear_velocity, angular_velocity=angular_velocity
        )
        # self.get_logger().info(f"Received velocity: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
        self.get_logger().info(f"left >> {omega_left} | right >> {omega_right}")
        f_left, f_right = self.motor_speeds(
            omega_left=omega_left, omega_right=omega_right
        )
        self.driver.motor_controller(device_address=1, left_rpm=f_left, right_rpm=f_right, left_torque=100, right_torque=100)
        self.get_logger().info(
            f"left speed >> {f_left} RPM | right speed >> {f_right} RPM"
        )

    def wheel_speeds(self, linear_velocity, angular_velocity):
        omega_left = (
            linear_velocity - (angular_velocity * self.wheel_base / 2)
        ) / self.wheel_radius
        omega_right = (
            linear_velocity + (angular_velocity * self.wheel_base / 2)
        ) / self.wheel_radius
        return omega_left, omega_right

    "RPM"
    def motor_speeds(self, omega_left, omega_right):
        f_left = (omega_left / (2 * math.pi)) * 60
        f_right = (omega_right / (2 * math.pi)) * 60
        return f_left, f_right
        
    
def main(args=None):
    rclpy.init(args=args)
    motor = MotorController()
    rclpy.spin(motor)
    motor.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
