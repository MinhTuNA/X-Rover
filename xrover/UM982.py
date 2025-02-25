import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import serial
import numpy as np
from .lib.UM982Lib import GPSDevice


class UM982Node(Node):
    def __init__(self):
        super().__init__("um982_node")
        self.gps_publisher = self.create_publisher(
            NavSatFix,
            "/gps/fix",
            10
        )
        self.heading_publisher = self.create_publisher(
            Float32,
            "/gps/heading",
            10
        )
        self.gps_device = GPSDevice()
        self.gps_port = str(self.gps_device.gps_port)
        self.gps_baudrate = self.gps_device.baudrate
        self.get_logger().info(
            f"port >> {self.gps_port} baud >> {self.gps_baudrate}")
        if self.gps_port is None or self.gps_baudrate is None:
            super().destroy_node()
            return
        self.gps_serial = serial.Serial(
            port=self.gps_port, baudrate=int(self.gps_baudrate), timeout=1)
        self.uart_timer = self.create_timer(0.5, self.read_uart_and_publish)

    def read_uart_and_publish(self):
        try:
            # Đọc dữ liệu từ UART
            data = self.gps_serial.readline()
            if data and b"\x00" not in data:
                um982_data = data.decode("utf-8", errors="ignore").strip()
                # self.get_logger().info(um982_data)
                self.data_parsed = self.gps_device.parse_um982_data(um982_data)
                # print(self.data_parsed)
                # self.get_logger().info(f"{self.data_parsed}")
        except Exception as e:
            self.get_logger().error(f"Error while reading from UART: {e}")

    def publish_coordinates_data(self, lat=None, lon=None, alt=None):
        gps_msg = NavSatFix()
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = alt
        self.gps_publisher.publish(gps_msg)

    def publish_heading(self, heading=None):
        heading_msg = Float32()
        heading_msg.data = heading
        self.heading_publisher.publish(heading_msg)

    def destroy_node(self):
        self.gps_serial.close()
        super().destroy_node()
        self.get_logger().info("UART connection closed.")


def main(args=None):
    rclpy.init(args=args)
    um982_node = UM982Node()
    try:
        rclpy.spin(um982_node)
    except KeyboardInterrupt:
        um982_node.get_logger().info("Node interrupted by user.")
    finally:
        um982_node.destroy_node()
        rclpy.shutdown()
        um982_node.get_logger().info("Node shut down.")


if __name__ == "__main__":
    main()
