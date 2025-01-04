#!/usr/bin/env python3
import math
import json
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist

class SensorManager:
    def __init__(self,node):
        self.node = node
        self.current_lat = None
        self.current_lon = None
        self.compass_heading = None
        self.offset = 0
        self.imu_roll = None
        self.imu_pitch = None
        self.imu_yaw = None
        # self.current_angle = None
        self.accel_angle = None
        self.gyro_rate = None
        self.gps_subscription = self.node.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, 10
        )
        self.compass_subscription = self.node.create_subscription(
            Float32, "/compass/heading", self.compass_callback, 10
        )
        self.imu_subscription = self.node.create_subscription(
            Imu, "/imu/data", self.imu_callback, 10
        )
        
    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def compass_callback(self, msg):
        
        self.compass_heading = self.to_signed_angle(msg.data)
    
    def imu_callback(self, msg):
        acc = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ]
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        quat = [
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ]
        self.imu_roll, self.imu_pitch, self.imu_yaw = self.quaternion_to_euler(
            quat[0], quat[1], quat[2], quat[3]
        )
        roll_deg = math.degrees(self.imu_roll)
        pitch_deg = math.degrees(self.imu_pitch)
        yaw_deg = math.degrees(self.imu_yaw)
        self.offset = self.compass_heading - self.imu_yaw
        # self.current_angle = self.to_signed_angle(self.imu_yaw + self.offset)
        
    def to_signed_angle(self, angle):
        return (angle + 180) % 360 - 180
        
    def quaternion_to_euler(self, w, x, y, z):
        """
        Chuyển đổi quaternion (w, x, y, z) sang Euler angles (Roll, Pitch, Yaw).

        Args:
            w (float): Phần vô hướng của quaternion.
            x (float): Thành phần trục x.
            y (float): Thành phần trục y.
            z (float): Thành phần trục z.

        Returns:
            tuple: (Roll, Pitch, Yaw) theo radian.
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Gimbal lock
        else:
            pitch = math.asin(sinp)
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw
    
    # def get_current_angle(self):
    #     return self.current_angle
        
