from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from .lib.UM982Lib import GPSDevice
from .lib.ConstVariable import COMMON
from .lib.SerialDeviceScanner import DevicePortScanner
from .lib.ConstVariable import GPS
from .VariableManager import instance
import serial
import numpy as np
import rclpy
import socket
import threading
import time
import base64
import json
import subprocess

class UM982Node(Node):
    def __init__(self):
        super().__init__("um982_node")

        self.tcp_server_ip = COMMON.tcp_server_ip
        self.tcp_server_port = COMMON.tcp_server_port
        
        self.rate = None

        self.gps_status_publisher = self.create_publisher(String, "/gps/status/rate", 10)
        self.gps_publisher = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.heading_publisher = self.create_publisher(Float32, "/gps/heading", 10)
        self.gps_ecef_publisher = self.create_publisher(Point, "/gps/ecef", 10)
        self.adrnava_publisher = self.create_publisher(String, "/gps/ADRNAVA", 10)
        self.uniheadinga_publisher = self.create_publisher(String, "/gps/UNIHEADINGA", 10)
        self.rtkstatusa_publisher = self.create_publisher(String, "/gps/RTKSTATUSA", 10)
        self.rtcmstatusa_publisher = self.create_publisher(String, "/gps/RTCMSTATUSA", 10)
        self.bestnavxyza_publisher = self.create_publisher(String, "/gps/BESTNAVXYZA", 10)
        
        self.create_subscription(String,"/gps/rtcm", self.rtcm_callback, 10)
        self.create_subscription(String,"/gps/rate", self.rate_callback, 10)
        self.create_subscription(String,"/gps/freset", self.freset_callback, 10)
        self.create_subscription(String,"/gps/save_config", self.save_config_callback, 10)
        self.create_subscription(String,"/rover/request_status", self.request_status_callback, 10)
        
        self.device = DevicePortScanner()
        self.ports = self.device.list_serial_ports()
        self.gps_port = self.device.find_um982_port(self.ports)
        self.gps_baudrate = GPS.baudrate
        self.gps_device = GPSDevice()
        self.gps_serial = None
        self.get_logger().info(f"port >> {self.gps_port} baud >> {self.gps_baudrate}")
        if self.gps_port is None or self.gps_baudrate is None:
            super().destroy_node()
            return
        try:
            self.gps_serial = serial.Serial(
                port=self.gps_port, 
                baudrate=self.gps_baudrate,
                timeout=1,
                dsrdtr=False,
                rtscts=False,
            )
        except Exception as e:
            self.get_logger().error(f"Error opening serial port: {e}")
            raise
        
        instance.load(COMMON.variable_path)
        self.load_variable()
        
        gps_thread = threading.Thread(target=self.read_gps)
        gps_thread.daemon = True
        gps_thread.start()
        # self.uart_timer = self.create_timer(0.5, self.read_uart_and_publish)

    def request_status_callback(self, msg):
        gps_status = String()
        gps_status.data = str(self.rate)
        self.gps_status_publisher.publish(gps_status)
    
    def load_variable(self):
        self.rate = int(instance.get("gps.rate",1))
        self.get_logger().info(f"gps rate: {self.rate}")
        self.set_gps_rate(self.rate)
        
    def format_hex(self, data: bytes) -> str:
        return " ".join(f"{b:02X}" for b in data)

    def rate_callback(self, msg: String):
        frequency = int(msg.data)
        self.set_gps_rate(frequency)
        instance.set("gps.rate", frequency)
        instance.save()
        
    def set_gps_rate(self,rate):
        self.rate = rate
        _rate = 1/int(rate)
        _rate = round(_rate, 2)
        cmd = (
            f"{self.gps_device.ADRNAVA_CMD} {_rate}\r\n"
            f"{self.gps_device.BESTNAVXYZA_CMD} {_rate}\r\n"
            f"{self.gps_device.UNIHEADINGA_CMD} {_rate}\r\n"
            f"{self.gps_device.RTCMSTATUSA_CMD} {_rate}\r\n"
            f"{self.gps_device.RTKSTATUSA_CMD} {_rate}\r\n"
        )
        self.gps_serial.write(cmd.encode())
        
    
    def freset_callback(self, msg: String):
        cmd = self.gps_device.FRESET_CMD + "\r\n"
        self.gps_serial.write(cmd.encode())
    
    def save_config_callback(self, msg: String):
        self.save_config()
        self.get_logger().info("saved config")
    
    def save_config(self):
        cmd = self.gps_device.SAVECONFIG_CMD + "\r\n"
        self.gps_serial.write(cmd.encode())

    # send rtcm data to gps
    def rtcm_callback(self, msg: String):
        rtcm_data = msg.data
        raw_data = base64.b64decode(rtcm_data)
        hex_str = self.format_hex(raw_data)
        # self.get_logger().info(f"hex >> {hex_str}")
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
                        rtk_status = String()
                        rtk_status.data = json.dumps(gps_data_parsed) 
                        self.rtkstatusa_publisher.publish(rtk_status)
                        # self.get_logger().info(f"RTK Status: {gps_data_parsed['pos_type']}")
                        
                    if gps_data_parsed["type"] == "uniheading_msg":
                        uniheading = String()
                        uniheading.data = json.dumps(gps_data_parsed)
                        self.uniheadinga_publisher.publish(uniheading)
                        heading = gps_data_parsed["heading"]
                        heading = (float(heading) + COMMON.off_set_heading) % 360

                        # self.get_logger().info(f"heading >> {heading}")
                        self.publish_heading(heading)
                        
                    if gps_data_parsed["type"] == "adrnav_msg":
                        adrnav = String()
                        adrnav.data = json.dumps(gps_data_parsed)
                        self.adrnava_publisher.publish(adrnav)
                        pos_type = gps_data_parsed["pos_type"]
                        # self.get_logger().info(f"ADR Nav: {pos_type}")
                        lat = gps_data_parsed["lat"]
                        lon = gps_data_parsed["lon"]
                        alt = gps_data_parsed["height"]
                        self.get_logger().info(f"lat >> {lat} \n" f"lon >> {lon} \n" f"alt >> {alt} \n")
                        self.publish_coordinates_data(lat=lat, lon=lon, alt=alt)
                        
                    if gps_data_parsed["type"] == "rtcm_status_msg":
                        rtcm_status = String()
                        rtcm_status.data = json.dumps(gps_data_parsed)
                        self.rtcmstatusa_publisher.publish(rtcm_status)
                        msg_id = gps_data_parsed["msg_id"]
                        # self.get_logger().info(f"RTCM {msg_id}")
                        
                    if gps_data_parsed["type"] == "bestnavxyz_msg":
                        bestnavxyz = String()
                        bestnavxyz.data = json.dumps(gps_data_parsed)
                        self.bestnavxyza_publisher.publish(bestnavxyz)
                        x = gps_data_parsed["P_X"]
                        y = gps_data_parsed["P_Y"]
                        z = gps_data_parsed["P_Z"]
                        # pos_type = gps_data_parsed["pos_type"]
                        # self.get_logger().info(f"Best Nav: {pos_type}")
                        self.publish_coordinates_ecef_data(ecef_x=x, ecef_y=y, ecef_z=z)
                        # self.get_logger().info(f"x >> {x}")
                        # self.get_logger().info(f"y >> {y}")
                        # self.get_logger().info(f"z >> {z}")
                        
        except Exception as e:
            self.get_logger().info(f"[ERROR] GPS Read Error: {e}")

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
        heading_robot = float(heading)
        heading_msg = Float32()
        heading_msg.data = heading_robot
        self.heading_publisher.publish(heading_msg)

    def destroy_node(self):
        self.gps_serial.reset_input_buffer()
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
