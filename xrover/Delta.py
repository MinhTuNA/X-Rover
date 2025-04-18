import sys
import json
from enum import Enum
from PySide6.QtCore import QCoreApplication
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String, Int32
from .lib.RobotInterface import RobotInterface
from .lib.SerialDeviceScanner import DevicePortScanner
from .lib.ConstVariable import COMMON
from .lib.ControlLib import ControlLib
import time
import message_filters


class ControlMode(Enum):
    ROVER = 1000
    DELTA = 2000


class SwaMode(Enum):
    SWA_ON = 1000
    SWA_OFF = 2000


class Delta(Node):

    def __init__(self):
        super().__init__("delta_node")
        self.get_logger().info("Delta Node Started")
        # Các tham số cấu hình cho trục Z và các thông số di chuyển khác
        self.z_safe = None
        self.z_min = -621.2
        self.z_max = -831.2

        self.signal_green = self.create_publisher(Int32, "signal_light/green", 10)
        self.signal_yellow = self.create_publisher(Int32, "signal_light/yellow", 10)
        self.signal_red = self.create_publisher(Int32, "signal_light/red", 10)
        self.signal_buzz = self.create_publisher(Int32, "signal_light/buzz", 10)

        self.is_first_swa = True
        self.is_first_connect = True
        self.control_lib = ControlLib()
        self.scanner = DevicePortScanner()
        self.ports = self.scanner.list_serial_ports()
        self.delta_port = self.scanner.find_delta_x_port(self.ports)
        self.delta = None
        self.start_delta()
        self.control_mode = ControlMode.ROVER
        self.current_swa = None
        self.current_swd = None
        self.current_vrb = 0

        self.update_delta_position_timer = self.create_timer(
            0.003, self.update_delta_position_callback
        )

        # Các subscriber của ROS
        self.create_subscription(Point, "/delta/move", self.move_to_call_back, 10)
        self.create_subscription(String, "/delta/go_home", self.go_home_call_back, 10)
        self.create_subscription(
            String, "/delta/request_status", self.request_status_call_back, 10
        )

        self.create_subscription(Int32, "fs_i6/ch1", self.y_callback, 10)
        self.create_subscription(Int32, "fs_i6/ch0", self.x_callback, 10)
        self.create_subscription(Int32, "fs_i6/ch5", self.z_callback, 10)

        self.create_subscription(Int32, "fs_i6/swa", self.swa_callback, 10)

        self.create_subscription(Int32, "fs_i6/swd", self.control_mode_callback, 10)
        self.status_pub = self.create_publisher(String, "/status", 10)

        self.current_x = 0
        self.current_y = 0
        self.current_z = 0

    def update_delta_position_callback(self):
        # self.get_logger().info(f"move - x: {self.current_x}, y: {self.current_y}")
        self.delta.move_relative(
            X=self.current_x,
            Y=self.current_y,
            F=self.F,
            A=self.A,
            S=self.S,
            E=self.E,
        )

        self.current_x = 0
        self.current_y = 0

    def swa_callback(self, msg):

        if self.is_first_swa:
            self.is_first_swa = False
            self.current_swa = int(msg.data)
            return

        if self.current_swa == int(msg.data):
            return
        self.current_swa = int(msg.data)
        if self.current_swa == SwaMode.SWA_ON.value:
            self.delta.move(W=-70)
        elif self.current_swa == SwaMode.SWA_OFF.value:
            self.delta.move(W=70)
        else:
            pass

    def start_delta(self):
        self.delta = RobotInterface(port=self.delta_port)
        self.delta.open()
        if self.delta.is_connected():
            if self.is_first_connect:
                self.is_first_connect = False
                self.delta.robot_resume()
                self.get_logger().info("delta is connected")
        self.delta.go_home()
        self.signal_green.publish(Int32(data=1))
        self.signal_yellow.publish(Int32(data=0))
        self.signal_red.publish(Int32(data=0))
        self.signal_buzz.publish(Int32(data=0))
        self.delta.feedbackPositionSignal.connect(self.feedback_position_callback)
        self.delta.get_position()

    def feedback_position_callback(self, x, y, z, w):
        self.get_logger().info(f"x: {x}, y: {y}, z: {z}, w: {w}")

    def control_mode_callback(self, msg):
        # Kiểm tra nếu trạng thái không đổi thì bỏ qua
        if self.current_swd == int(msg.data):
            return
        self.current_swd = int(msg.data)
        if msg.data == 1000:
            self.control_mode = ControlMode.ROVER
            self.get_logger().info("control mode: rover")
        elif msg.data == 2000:
            self.control_mode = ControlMode.DELTA
            self.get_logger().info("control mode: delta")
        else:
            self.get_logger().info("invalid control mode")

    def z_callback(self, msg):
        if self.control_mode == ControlMode.DELTA:
            value = int(msg.data)
            if abs(value - self.current_vrb) > 1:
                self.current_vrb = value
                new_z = self.map_joystick_to_range(value, self.z_min, self.z_max)
                z_value = new_z - self.delta.Z
                z_value = round(z_value, 2)
                self.get_logger().info(f"z_value: {z_value}")
                self.delta.move_relative(
                    Z=z_value, F=self.F_z, A=self.A_z, S=self.S_z, E=self.E_z
                )
                # self.current_z = z_value
        else:
            pass

    def x_callback(self, msg):
        if self.control_mode == ControlMode.DELTA:
            value = int(msg.data)
            new_value = self.control_lib.delta_x(value)
            if new_value == 1:
                # self.get_logger().info("x: +2")
                # self.delta.move_relative(X=2, F=self.F, A=self.A, S=self.S, E=self.E)
                self.current_x = 2
            elif new_value == -1:
                # self.get_logger().info("x: -2")
                # self.delta.move_relative(X=-2, F=self.F, A=self.A, S=self.S, E=self.E)
                self.current_x = -2
        else:
            pass

    def y_callback(self, msg):
        if self.control_mode == ControlMode.DELTA:
            value = int(msg.data)
            new_value = self.control_lib.delta_y(value)
            if new_value == 1:
                # self.get_logger().info("y: +2")
                # self.delta.move_relative(Y=2, F=self.F, A=self.A, S=self.S, E=self.E)
                self.current_y = 2
            elif new_value == -1:
                # self.get_logger().info("y: -2")
                # self.delta.move_relative(Y=-2, F=self.F, A=self.A, S=self.S, E=self.E)
                self.current_y = -2
        else:
            pass

    def synced_callback(self, x_msg, y_msg, z_msg):
        # Xử lý tín hiệu X
        x_value = int(x_msg.data)
        delta_x = self.control_lib.delta_x(x_value)
        if delta_x == 1:
            self.current_x = 1
        elif delta_x == -1:
            self.current_x = -1
        else:
            self.current_x = 0

        # Xử lý tín hiệu Y
        y_value = int(y_msg.data)
        delta_y = self.control_lib.delta_y(y_value)
        if delta_y == 1:
            self.current_y = 1
        elif delta_y == -1:
            self.current_y = -1
        else:
            self.current_y = 0

        # Xử lý tín hiệu Z
        z_value_raw = int(z_msg.data)
        new_z = self.map_joystick_to_range(z_value_raw, self.z_min, self.z_max)
        # Giả sử self.delta có thuộc tính Z lưu vị trí hiện tại của trục z
        z_adjust = new_z - self.delta.Z
        self.current_z = round(z_adjust, 2)

        self.get_logger().info(
            f"move - x: {self.current_x}, y: {self.current_y}, z: {self.current_z}"
        )

        # Gọi lệnh di chuyển nếu cần (ở đây đang comment lại)
        self.delta.move_relative(
            X=self.current_x,
            Y=self.current_y,
            # Z=self.current_z,
            F=self.F,
            A=self.A,
            S=self.S,
            E=self.E,
        )

    def move_to_call_back(self, msg):
        x = msg.x
        y = msg.y
        z = msg.z
        self.get_logger().info(f"move - x: {x}, y: {y}, z: {z}")
        # self.move_delta(x=x, y=y, z=z)

    def go_home_call_back(self, msg):
        self.delta.go_home()

    def move_delta(self, x, y, z, is_relative=False):
        if not is_relative:
            self.delta.move(X=x, Y=y, Z=z)
        else:
            self.delta.move_relative(X=x, Y=y, Z=z)

    def request_status_call_back(self, msg):
        status_msg = String()
        if self.delta.is_connected():
            self.get_logger().info("Delta is connected")
            status = {"delta": "connected"}
        else:
            self.get_logger().info("Delta is not connected")
            status = {"delta": "not connected"}

        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)

    def map_joystick_to_range(
        self, joy_value, out_min, out_max, joy_min=1000, joy_max=2000
    ):
        value = out_min + ((joy_value - joy_min) / (joy_max - joy_min)) * (
            out_max - out_min
        )
        return round(value, 2)

    def handle_destroy(self):
        # Nếu cần, đóng kết nối đến delta
        # self.delta.close()
        self.destroy_node()
        self.get_logger().info("Delta Node Stopped")


def main(args=None):
    app = QCoreApplication(sys.argv)
    rclpy.init(args=args)
    node = None
    try:
        node = Delta()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        app.exec()
    except Exception as e:
        print(e)
    finally:
        if node is not None:
            node.handle_destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
