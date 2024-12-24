#!/usr/bin/env python3
import math
import time
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist
import math


class Navigator(Node):
    def __init__(self):
        super().__init__("navigator_node")
        self.goal_lat = None
        self.goal_lon = None

        self.current_lat = None
        self.current_lon = None
        self.compass_heading = None

        self.imu_roll = None
        self.imu_pitch = None
        self.imu_yaw = None

        self.last_twist = Twist()

        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status_publisher = self.create_publisher(String, "/status", 10)

        self.gps_subscription = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, 10
        )
        self.compass_subscription = self.create_subscription(
            Float32, "/compass/heading", self.compass_callback, 10
        )
        self.goal_subscription = self.create_subscription(
            NavSatFix, "/gps/goal", self.goal_callback, 10
        )
        self.imu_subscription = self.create_subscription(
            Imu, "/imu/data", self.imu_callback, 10
        )
        self.gps_subscription
        self.compass_subscription
        self.goal_subscription
        self.imu_subscription

        # self.timer = self.create_timer(0.1, self.navigate)
        self.step_count = 30

    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def compass_callback(self, msg):
        self.compass_heading = msg.data

    def goal_callback(self, msg):
        self.goal_lat = msg.latitude
        self.goal_lon = msg.longitude

    def imu_callback(self, msg):
        # Extract acceleration (acc)
        acc = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ]

        # Extract angular velocity (gyro)
        gyro = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]

        # Extract quaternion (orientation)
        quat = [
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ]

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        self.imu_roll, self.imu_pitch, self.imu_yaw = self.quaternion_to_euler(
            quat[0], quat[1], quat[2], quat[3]
        )
        roll_deg = math.degrees(self.imu_roll)
        pitch_deg = math.degrees(self.imu_pitch)
        yaw_deg = math.degrees(self.imu_yaw)
        # Print the data
        self.get_logger().info(
            # f"\nAcceleration (acc): {acc}\n"
            # f"Angular Velocity (gyro): {gyro}\n"
            f"euler >> {int(roll_deg)} {int(pitch_deg)} {int(yaw_deg)}\n"
            # f"Quaternion (quat): {quat}"
        )

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

    def haversine(self, lat1, lon1, lat2, lon2):
        """Tính khoảng cách giữa hai điểm GPS (theo m)."""
        R = 6371000
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = (
            math.sin(delta_phi / 2.0) ** 2
            + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    def calculate_bearing(self, lat1, lon1, lat2, lon2):
        dLon = math.radians(lon2 - lon1)
        lat1 = math.radians(lat1)
        lat2 = math.radians(lat2)

        x = math.sin(dLon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - (
            math.sin(lat1) * math.cos(lat2) * math.cos(dLon)
        )
        bearing = math.atan2(x, y)
        return (math.degrees(bearing) + 360) % 360

    def detect_obstacle(self):

        return False  # Ví dụ: không có vật cản

    def avoid_obstacle(self):
        """Điều khiển robot tránh vật cản."""
        twist = Twist()
        twist.angular.z = 0.5
        self.cmd_vel_pub.publish(twist)

    def send_status(self, status):
        cmd = {
            "status": status,
        }
        msg_json = json.dumps(cmd)
        msg = String()
        msg.data = msg_json
        self.status_publisher.publish(msg)

    def navigate(self):
        if (
            self.current_lat is None
            or self.current_lon is None
            or self.compass_heading is None
        ):
            self.get_logger().info("sensor initializing")
            return

        if self.goal_lat is None or self.goal_lon is None:
            self.get_logger().info("chưa có điểm đích")
            return

        if self.detect_obstacle():
            self.avoid_obstacle()
            return

        distance = self.haversine(
            self.current_lat, self.current_lon, self.goal_lat, self.goal_lon
        )
        target_bearing = self.calculate_bearing(
            self.current_lat, self.current_lon, self.goal_lat, self.goal_lon
        )
        angle_error = target_bearing - self.compass_heading

        angle_error = (angle_error + 180) % 360 - 180

        self.get_logger().info(
            f"\nKhoảng cách: {distance:.2f} m"
            f"\ngóc địa lý 2 giữa 2 điểm: {target_bearing:.2f} độ "
            f"\ngóc địa lý robot: {self.compass_heading:.2f} độ"
            f"\ngóc lệch: {angle_error}"
        )
        twist = Twist()

        if distance > 10.0 and self.step_count > 0:
            if abs(angle_error) > 5:
                twist.angular.z = 0.3 if angle_error > 0 else -0.3
            else:
                twist.linear.x = 0.1
            self.step_count -= 1
        else:
            self.get_logger().info("Đã đến đích!")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.goal_lat = None
            self.goal_lon = None
            self.step_count = 30
            self.send_status("done")

        if (
            twist.linear.x != self.last_twist.linear.x
            or twist.angular.z != self.last_twist.angular.z
        ):
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(
                f"Twist command: Linear.x = {twist.linear.x}, Angular.z = {twist.angular.z}"
            )
            self.last_twist = twist
        else:
            self.get_logger().info("Lệnh không thay đổi, không gửi.")


def main(args=None):
    rclpy.init(args=args)
    navigator = Navigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
