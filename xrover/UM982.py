from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from lib.UM982Lib import GPSDevice
from lib.ConstVariable import COMMON
import serial
import numpy as np
import rclpy
import socket
import threading
import time
import base64


class UM982Node(Node):
    def __init__(self):
        super().__init__("um982_node")

        self.tcp_server_ip = COMMON.tcp_server_ip
        self.tcp_server_port = COMMON.tcp_server_port

        self.gps_publisher = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.heading_publisher = self.create_publisher(Float32, "/gps/heading", 10)
        self.gps_ecef_publisher = self.create_publisher(Point, "/gps/ecef", 10)
        self.create_subscription(String, "/gps/rtcm", self.rtcm_callback, 10)

        self.gps_device = GPSDevice()
        self.gps_port = str(self.gps_device.gps_port)
        self.gps_baudrate = self.gps_device.baudrate
        self.get_logger().info(f"port >> {self.gps_port} baud >> {self.gps_baudrate}")
        if self.gps_port is None or self.gps_baudrate is None:
            super().destroy_node()
            return

        self.gps_serial = serial.Serial(
            port=self.gps_port, baudrate=int(self.gps_baudrate), timeout=1
        )

        gps_thread = threading.Thread(target=self.read_gps)
        gps_thread.daemon = True
        gps_thread.start()
        # self.uart_timer = self.create_timer(0.5, self.read_uart_and_publish)

    def format_hex(self, data: bytes) -> str:
        return " ".join(f"{b:02X}" for b in data)

    def rtcm_callback(self, msg: String):
        rtcm_data = msg.data
        raw_data = base64.b64decode(rtcm_data)
        hex_str = self.format_hex(raw_data)
        # print(f"hex >> {hex_str}")
        self.gps_serial.write(raw_data)

    def read_gps(self):
        try:
            while True:
                gps_data = self.gps_serial.readline().decode(errors="ignore").strip()
                if gps_data:
                    gps_data_parsed = self.gps_device.parse_um982_data(gps_data)
                    if gps_data_parsed is None:
                        continue
                    # print(f"gps >> {gps_data_parsed}")
                    if gps_data_parsed["type"] == "rtk_status_msg":
                        pass
                    if gps_data_parsed["type"] == "uniheading_msg":
                        heading = gps_data_parsed["heading"]
                        print(f"heading >> {heading}")
                    if gps_data_parsed["type"] == "adrnav_msg":
                        pos_type = gps_data_parsed["pos_type"]
                        lat = gps_data_parsed["lat"]
                        lon = gps_data_parsed["lon"]
                        alt = gps_data_parsed["height"]
                        print(f"lat >> {lat} \n" f"lon >> {lon} \n" f"alt >> {alt} \n")
                        self.publish_coordinates_data(lat=lat, lon=lon, alt=alt)
                    if gps_data_parsed["type"] == "rtcm_status_msg":
                        msg_id = gps_data_parsed["msg_id"]
                        print(f"RTCM {msg_id}")
                    if gps_data_parsed["type"] == "bestnavxyz_msg":
                        x = gps_data_parsed["P_X"]
                        y = gps_data_parsed["P_Y"]
                        z = gps_data_parsed["P_Z"]
                        self.publish_coordinates_ecef_data(ecef_x=x, ecef_y=y, ecef_z=z)
                        print(f"x >> {x}")
                        print(f"y >> {y}")
                        print(f"z >> {z}")
        except Exception as e:
            print(f"[ERROR] GPS Read Error: {e}")

    def read_uart_and_publish(self):
        try:
            # Đọc dữ liệu từ UART
            data = self.gps_serial.readline()
            if data and b"\x00" not in data:
                um982_data = data.decode("utf-8", errors="ignore").strip()
                self.get_logger().info(um982_data)

                # print(self.data_parsed)
                # self.get_logger().info(f"{self.data_parsed}")
        except Exception as e:
            self.get_logger().error(f"Error while reading from UART: {e}")

    def publish_coordinates_ecef_data(self, ecef_x=None, ecef_y=None, ecef_z=None):
        gps_msg = Point()
        gps_msg.x = float(ecef_x)
        gps_msg.y = float(ecef_y)
        gps_msg.z = float(ecef_z)
        self.gps_ecef_publisher.publish(gps_msg)

    def publish_coordinates_data(self, lat=None, lon=None, alt=None):
        gps_msg = NavSatFix()
        gps_msg.latitude = float(lat)
        gps_msg.longitude = float(lon)
        gps_msg.altitude = float(alt)
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
