#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math
from .lib.ConstVariable import COMMON, WHEEL
from .lib.ModbusDevice import Driver
from .lib.SerialDeviceScanner import DevicePortScanner

# from lib.PID import PIDController


class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller_node")
        self.get_logger().info("motor controller has been started")
        self.wheel_radius = COMMON.wheel_radius
        self.wheel_base = COMMON.wheel_base
        scanner = DevicePortScanner()
        ports = scanner.get_ports()
        self.rs485_port = scanner.find_rs485_port(ports)
        self.get_logger().info(f"RS485 port: {self.rs485_port}")
        self.driver = Driver(self.rs485_port)
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.kp = COMMON.motorKp
        self.ki = COMMON.motorKi
        self.kd = COMMON.motorKd
        # self.motor_pid = PIDController(
        #     kp=self.kp, ki=self.ki, kd=self.kd, output_limits=(0, 0.2)
        # )
        self.create_subscription(Twist, "/rover/vel", self.cmd_vel_callback, 10)
        self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)
        self.target_yaw = None
        self.current_yaw = 0.0
        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.update)
        self.mode = WHEEL.speed_mode

    def quaternion_to_euler(self, w, x, y, z):
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
        self.current_yaw = yaw_deg

    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

    def wheel_speeds(self, linear_velocity, angular_velocity):
        omega_left = (
            linear_velocity + (angular_velocity * self.wheel_base / 2)
        ) / self.wheel_radius
        omega_right = (
            linear_velocity - (angular_velocity * self.wheel_base / 2)
        ) / self.wheel_radius
        return omega_left, omega_right

    "RPM"

    def motor_speeds(self, omega_left, omega_right):
        f_left = (omega_left / (2 * math.pi)) * 60
        f_right = (omega_right / (2 * math.pi)) * 60
        return f_left, f_right

    def update(self):
        if self.mode == WHEEL.speed_mode:
            if self.linear_velocity == 0 and self.angular_velocity == 0:
                self.driver.set_motor(
                    left_rpm=0,
                    right_rpm=0,
                    left_torque=300,
                    right_torque=300,
                    right_mode=WHEEL.speed_mode,
                    left_mode=WHEEL.speed_mode,
                )
                self.get_logger().info(
                    f"left speed >> {0} RPM | right speed >> {0} RPM"
                )

            elif self.linear_velocity != 0 or self.angular_velocity != 0:
                omega_left, omega_right = self.wheel_speeds(
                    linear_velocity=self.linear_velocity,
                    angular_velocity=self.angular_velocity,
                )
                f_left, f_right = self.motor_speeds(
                    omega_left=omega_left, omega_right=omega_right
                )
                f_left = round(f_left, 4)
                f_right = round(f_right, 4)
                self.get_logger().info(f"f_left >> {f_left} | f_right >> {f_right}")
                self.driver.set_motor(
                    left_rpm=int(f_left),
                    right_rpm=int(f_right),
                    right_torque=600,
                    left_torque=600,
                    left_mode=WHEEL.speed_mode,
                    right_mode=WHEEL.speed_mode,
                )
            else:
                self.get_logger().info("invalid cmd")
        elif self.mode == WHEEL.torque_mode:
            pass

        # elif self.linear_velocity != 0 and self.angular_velocity == 0:
        #     self.target_yaw = self.current_yaw
        #     yaw_error = (
        #         (self.target_yaw - self.current_yaw)
        #         if self.target_yaw is not None
        #         else 0
        #     )
        #     current_time = self.get_clock().now()
        #     dt = (current_time - self.last_time).nanoseconds / 1e9
        #     self.last_time = current_time
        #     yaw_correction = self.motor_pid.compute(yaw_error, dt)
        #     adjusted_angular_velocity = self.angular_velocity + yaw_correction
        #     omega_left, omega_right = self.wheel_speeds(
        #         linear_velocity=self.linear_velocity,
        #         angular_velocity=adjusted_angular_velocity,
        #     )

        #     # self.get_logger().info(f"Received velocity: linear.x={msg.linear.x}, angular.z={msg.angular.z}")
        #     # self.get_logger().info(f"left >> {omega_left} | right >> {omega_right}")
        #     f_left, f_right = self.motor_speeds(
        #         omega_left=omega_left, omega_right=omega_right
        #     )
        #     self.driver.set_motor(
        #         left_rpm=int(f_left),
        #         right_rpm=int(f_right),
        #         left_torque=100,
        #         right_torque=100,
        #         left_mode=WHEEL.speed_mode,
        #         right_mode=WHEEL.speed_mode,
        #     )
        #     self.get_logger().info(
        #         f"left speed >> {f_left} RPM | right speed >> {f_right} RPM"
        #     )
        # elif self.linear_velocity != 0 and self.angular_velocity != 0:
        #     omega_left, omega_right = self.wheel_speeds(
        #         linear_velocity=self.linear_velocity,
        #         angular_velocity=self.angular_velocity,
        #     )
        #     f_left, f_right = self.motor_speeds(
        #         omega_left=omega_left, omega_right=omega_right
        #     )
        #     self.driver.set_motor(
        #         left_rpm=int(f_left),
        #         right_rpm=int(f_right),
        #         left_torque=100,
        #         right_torque=100,
        #         left_mode=WHEEL.speed_mode,
        #         right_mode=WHEEL.speed_mode,
        #     )
        #     self.get_logger().info(
        #         f"left speed >> {int(f_left)} RPM | right speed >> {int(f_right)} RPM"
        #     )
        # else:
        #     self.get_logger().info("invalid cmd")
        #     pass

    def handle_destroy(self):
        self.timer.cancel()
        self.driver.cleanup()
        self.get_logger().info("motor controller has been stopped")
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    motor = MotorController()
    try:
        rclpy.spin(motor)
    except KeyboardInterrupt:
        pass
    finally:
        motor.handle_destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
