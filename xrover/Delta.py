import sys
import json
from PySide6.QtWidgets import QApplication
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from .lib.RobotInterface import RobotInterface
from .lib.SerialDeviceScanner import DevicePortScanner
from .lib.ConstVariable import COMMON


class Delta(Node):
    def __init__(self):
        super().__init__("delta_node")
        self.get_logger().info("Delta Node Started")
        self.z_safe = None
        self.is_first_connect = True

        self.scanner = DevicePortScanner()
        self.delta_port = self.scanner.find_delta_x_port()
        self.delta = RobotInterface(port=self.delta_port)
        self.delta.open()
        if self.delta.is_connected() == True:
            if self.is_first_connect == True:
                self.is_first_connect = False
                self.delta.robot_resume()
        self.delta.set_z_safe(COMMON.z_safe)

        self.create_subscription(Point, "/delta/move", self.move_to_call_back, 10)
        self.create_subscription(String, "/delta/go_home", self.go_home_call_back, 10)
        self.create_subscription(
            String, "/delta/request_status", self.request_status_call_back, 10
        )

        self.status_pub = self.create_publisher(String, "/status", 10)

    def move_to_call_back(self, msg):
        x = msg.x
        y = msg.y
        z = msg.z
        self.move_delta(x=x, y=y, z=z)

    def go_home_call_back(self, msg):
        self.delta.go_home()

    def move_delta(self, x, y, z, is_relative=False):
        if is_relative == False:
            self.delta.move(X=x, Y=y, Z=z)
        else:
            self.delta.move_relative(X=x, Y=y, Z=z)

    def request_status_call_back(self, msg):
        status_msg = String()

        if self.delta.is_connected():
            self.get_logger().info("Delta is connected")
            status = {
                "delta": "connected",
            }
        else:
            self.get_logger().info("Delta is not connected")
            status = {"delta": "not connected"}

        status_msg.data = json.dumps(status)
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    try:
        node = Delta()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
