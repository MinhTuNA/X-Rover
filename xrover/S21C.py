import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from .lib.SerialDeviceScanner import DevicePortScanner
from .lib.ConstVariable import ULTRASONIC
import serial
import os


class DistanceSensorsNode(Node):
    def __init__(self):
        super().__init__("s21c_sensors_node")
        self.s21c_baudrate = None
        self.s21c_min_range = None
        self.s21c_max_range = None
        self.load_setting()
        self.a_publisher = self.create_publisher(Range, "/sensor/A", 10)
        self.b_publisher = self.create_publisher(Range, "/sensor/B", 10)
        self.c_publisher = self.create_publisher(Range, "/sensor/C", 10)
        self.d_publisher = self.create_publisher(Range, "/sensor/D", 10)
        self.e_publisher = self.create_publisher(Range, "/sensor/E", 10)
        self.f_publisher = self.create_publisher(Range, "/sensor/F", 10)
        self.device = DevicePortScanner()
        self.s21c_port = self.device.find_s21c_port()
        self.s21c_field_of_view = 1.047
        self.timeout = 1
        self.sensor_data_index = 0
        try:
            self.serial_port = serial.Serial(
                port=self.s21c_port, baudrate=self.s21c_baudrate, timeout=self.timeout
            )
            self.get_logger().info(f"Connected to UART port: {self.s21c_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening UART port: {e}")
            raise e

        self.read_uart_data()

    def load_setting(self):
        self.s21c_baudrate = ULTRASONIC.baudrate
        self.s21c_min_range = ULTRASONIC.min_range
        self.s21c_max_range = ULTRASONIC.max_range

    def read_uart_data(self):
        try:
            while True:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.readline().decode("utf-8").strip()
                    sensor_data = self.parse_sensor_data(data)
                    self.publish_sensor_data(sensor_data)
                    self.get_logger().info(str(self.parse_sensor_data(data)))
                    # self.get_logger().info(f'{data}')
        except Exception as e:
            self.get_logger().error(f"Error reading from UART: {e}")

    def parse_sensor_data(self, data):
        lines = data.splitlines()

        parsed_data = {}
        for line in lines:
            parts = line.split()
            for part in parts:
                if ":" in part and part[0] in "ABCDEF":
                    key, value = part.split(":")
                    parsed_data[key] = int(value)

        return parsed_data

    def publish_sensor_data(self, data):
        if "A" in data:
            self.publish_range(self.a_publisher, "A", data["A"])
        if "B" in data:
            self.publish_range(self.b_publisher, "B", data["B"])
        if "C" in data:
            self.publish_range(self.c_publisher, "C", data["C"])
        if "D" in data:
            self.publish_range(self.d_publisher, "D", data["D"])
        if "E" in data:
            self.publish_range(self.e_publisher, "E", data["E"])
        if "F" in data:
            self.publish_range(self.f_publisher, "F", data["F"])

    def publish_range(self, publisher, frame_id, distance):
        if distance <= 0:
            return
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.s21c_field_of_view
        msg.min_range = float(self.s21c_min_range)
        msg.max_range = float(self.s21c_max_range)
        msg.range = distance / 1000.0
        publisher.publish(msg)
        # if frame_id == "E":
        #     self.get_logger().info(f"Published {frame_id}: {msg.range:.3f} m")


def main(args=None):
    rclpy.init(args=args)
    node = DistanceSensorsNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
