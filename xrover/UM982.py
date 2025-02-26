
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
from lib.UM982Lib import GPSDevice
from lib.ConstVariable import COMMON
import serial
import numpy as np
import rclpy
import socket
import threading


class UM982Node(Node):
    def __init__(self):
        super().__init__("um982_node")

        self.tcp_server_ip = COMMON.tcp_server_ip
        self.tcp_server_port = COMMON.tcp_server_port

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
        rtcm_thread = threading.Thread(
            target=self.receive_rtcm_and_send_to_serial)
        rtcm_thread.daemon = True
        rtcm_thread.start()

        # Chạy Thread 2: Đọc GPS từ COM9
        gps_thread = threading.Thread(target=self.read_gps)
        gps_thread.daemon = True
        gps_thread.start()
        # self.uart_timer = self.create_timer(0.5, self.read_uart_and_publish)

    def receive_rtcm_and_send_to_serial(self):
        try:
            # Kết nối đến TCP Server để nhận RTCM
            tcp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcp_client.connect((self.tcp_server_ip, self.tcp_server_port))
            print(
                f"[INFO] Connected to RTCM Server {self.tcp_server_ip}:{self.tcp_server_port}")

            while True:
                data = tcp_client.recv(1024)  # Nhận 1024 bytes từ TCP Server
                if not data:
                    print("[INFO] No more RTCM data. Closing connection.")
                    break  # Dừng nếu mất kết nối
                self.gps_serial.write(data)  # Gửi RTCM vào COM9
                # print(f"[RTCM] Sent {len(data)} bytes to gps")

        except Exception as e:
            print(f"[ERROR] RTCM Transfer Error: {e}")

        finally:
            tcp_client.close()

    def read_gps(self):
        try:
            while True:
                gps_data = self.gps_serial.readline().decode(errors='ignore').strip()
                if gps_data:
                    gps_data_parsed = self.gps_device.parse_um982_data(
                        gps_data
                    )
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
                        print(
                            f"pos type: {pos_type} \n"
                            f"lat >> {lat} \n"
                            f"lon >> {lon} \n"
                        )
                    if gps_data_parsed["type"] == "rtcm_status_msg":
                        msg_id = gps_data_parsed["msg_id"]
                        print(f"RTCM {msg_id}")
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
