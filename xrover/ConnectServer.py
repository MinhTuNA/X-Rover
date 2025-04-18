#!/usr/bin/env python3
import rclpy
import cv2
import base64
from cv_bridge import CvBridge
import json
import os
import socketio
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Image, Imu
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist, Point
from std_srvs.srv import Trigger

from .lib.ConstVariable import COMMON

sio = socketio.Client()


class ConnectServer(Node):
    def __init__(self):
        super().__init__("connect_server_node")
        self.server_address = COMMON.server_address
        self.gps_data = None
        self.gps_heading = None

        # subscribers
        self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, 10)
        self.create_subscription(Float32, "/gps/heading", self.heading_callback, 10)
        self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)
        self.create_subscription(
            Image, "/camera/color/image_raw", self.image_callback, 10
        )
        self.create_subscription(String, "/rover/mode", self.rover_mode_callback, 10)
        
        #status
        self.create_subscription(String,"/gps/status/rate", self.gps_status_callback, 10)
        
        self.create_subscription(String, "/gps/ADRNAVA", self.adrnav_callback, 10)
        self.create_subscription(String, "/gps/UNIHEADINGA", self.uniheading_callback, 10)
        self.create_subscription(String, "/gps/RTKSTATUSA", self.rtkstatus_callback, 10)
        self.create_subscription(String, "/gps/RTCMSTATUSA", self.rtcmstatus_callback, 10)
        self.create_subscription(String, "/gps/BESTNAVXYZA", self.bestnav_callback, 10)

        # publishers
        self.publish_program_cmd = self.create_publisher(String, "/program_cmd", 10)
        self.rover_vel_pub = self.create_publisher(Twist, "/navigation/vel", 10)
        self.move_delta_pub = self.create_publisher(Point, "/delta/move", 10)
        self.delta_go_home_pub = self.create_publisher(String, "delta/go_home", 10)
        self.request_delta_status_pub = self.create_publisher(
            String, "delta/request_status", 10
        )
        self.target_point_pub = self.create_publisher(NavSatFix, "/target_point", 10)
        self.start_pub = self.create_publisher(String, "/start", 10)
        self.stop_pub = self.create_publisher(String, "/stop", 10)
        self.mode_pub = self.create_publisher(String, "/mode", 10)
        self.method_control_pub = self.create_publisher(String, "/method_control", 10)
        self.start_logger_pub = self.create_publisher(String, "/logger/start", 10)
        self.stop_logger_pub = self.create_publisher(String, "/logger/stop", 10)
        self.freset_pub = self.create_publisher(String, "/gps/freset", 10)
        self.set_rate_pub = self.create_publisher(String, "/gps/rate", 10)
        self.save_config_pub = self.create_publisher(String, "/gps/save_config", 10)
        self.request_rover_status_pub = self.create_publisher(String, "/rover/request_status", 10)
        # socketio
        self.sio = sio
        self.is_connected = False
        self.hasRegisteredEvents = False

        if not self.hasRegisteredEvents:
            self.sio.on("connect", self.on_connect, namespace="/rover")
            self.sio.on("path", self.save_path, namespace="/rover")
            self.sio.on("move_rover", self.rover_vel, namespace="/rover")
            self.sio.on("start", self.start, namespace="/rover")
            self.sio.on("stop", self.stop, namespace="/rover")
            self.sio.on("start_logging", self.start_logging, namespace="/rover")
            self.sio.on("stop_logging", self.stop_logging, namespace="/rover")
            self.sio.on("delta_go_home", self.delta_go_home, namespace="/rover")
            self.sio.on("move_delta", self.move_delta, namespace="/rover")
            self.sio.on("disconnect", self.disconnect, namespace="/rover")
            self.sio.on("freset", self.freset_callback, namespace="/rover")
            self.sio.on("save_config", self.save_config_callback, namespace="/rover")
            self.sio.on("set_rate", self.rate_callback, namespace="/rover")
            self.sio.on(
                "request_delta_status", self.request_delta_status, namespace="/rover"
            )
            self.sio.on(
                "request_rover_status", self.request_rover_status, namespace="/rover"
            )
            self.sio.on("set_mode", self.set_mode, namespace="/rover")
            self.sio.on("set_method_control", self.set_method_control, namespace="/rover")
            self.sio.on("target_point", self.target_point, namespace="/rover")

    # socketio events
    
    def on_connect(self):
        self.is_connected = True
        print("- Connected to server")

    def save_path(self, data):
        file_path = COMMON.file_path

        if os.path.exists(file_path):
            self.get_logger().info(
                f"File {file_path} already exists. Updating contents..."
            )
        else:
            self.get_logger().info(
                f"File {file_path} does not exist. Creating new file..."
            )

        with open(file_path, "w") as json_file:
            json.dump(data, json_file, indent=4)

        self.get_logger().info(f"Data has been saved to {file_path}")

    def rover_vel(self, data):
        x = data["x"]
        z = data["z"]
        # self.get_logger().info(f"x: {x}, z: {z}")
        self.publish_rover_vel(x, z)

    def start(self, data):
        signal = String()
        signal.data = "start"
        self.start_pub.publish(signal)
        self.get_logger().info(f"started")

    def stop(self, data):
        signal = String()
        signal.data = "stop"
        self.stop_pub.publish(signal)
        self.get_logger().info(f"stopped")
    
    def start_logging(self, data):
        signal = String()
        signal.data = str(data)
        self.start_logger_pub.publish(signal)
        # self.get_logger().info(f"started logger {data}")

    def stop_logging(self, data):
        signal = String()
        signal.data = str(data)
        self.stop_logger_pub.publish(signal)
        # self.get_logger().info(f"stopped logger {data}")

    def set_mode(self, data):
        signal = String()
        signal.data = str(data)
        self.mode_pub.publish(signal)
        self.get_logger().info(f"set mode: {data}")
    
    def set_method_control(self, data):
        signal = String()
        signal.data = str(data)
        self.method_control_pub.publish(signal)
        self.get_logger().info(f"method control: {data}")
    
    def freset_callback(self, data):
        signal = String()
        self.freset_pub.publish(signal)
        self.get_logger().info("freset")
        
    def save_config_callback(self, data):
        signal = String()
        self.save_config_pub.publish(signal)
        self.get_logger().info("save config")
        
    def rate_callback(self, data):
        signal = String()
        signal.data = str(data)
        self.set_rate_pub.publish(signal)
        # self.get_logger().info(f"set rate: {data}")

    def move_delta(self, data):
        pos = Point()
        pos.x = float(data["x"])
        pos.y = float(data["y"])
        pos.z = float(data["z"])
        # print(f"delta >> x: {pos.x}, y: {pos.y}, z: {pos.z} ")
        self.get_logger().info(f"delta >> x: {pos.x}, y: {pos.y}, z: {pos.z} ")
        self.move_delta_pub.publish(pos)

    def delta_go_home(self, data):
        signal = String()
        self.delta_go_home_pub.publish(signal)

    def request_delta_status(self, data):
        signal = String()
        # self.get_logger().info("requesting delta status")
        self.request_delta_status_pub.publish(signal)

    def request_rover_status(self, data):
        # self.get_logger().info("requesting rover status")
        signal = String()
        self.request_rover_status_pub.publish(signal)
    
    def target_point(self, data):
        latitude = data["latitude"]
        longitude = data["longitude"]
        target_point = NavSatFix()
        target_point.latitude = latitude
        target_point.longitude = longitude
        self.get_logger().info(f"target point: {latitude}, {longitude}")
        self.target_point_pub.publish(target_point)

    def disconnect(self):
        print("Disconnected from server")

    #
    #
    #
    # call back functions

    def gps_callback(self, msg):
        """Xử lý dữ liệu GPS."""
        self.gps_data = {
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
        }
        self.send_gps_data()
        
    def gps_status_callback(self, msg):
        rate = msg.data
        self.send_to_server(name="gps_rate", data=rate)
        # self.get_logger().info(f"gps rate: {rate}")

    def heading_callback(self, msg):
        self.gps_heading = msg.data
        self.send_gps_data()

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
        if sio.connected:
            self.sio.emit(
                "IMU_data", {"acc": acc, "gyro": gyro, "quat": quat}, namespace="/rover"
            )
            # self.get_logger().info("Send IMU data to server")

    def image_callback(self, msg):
        try:
            cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
            _, img_encoded = cv2.imencode(".jpg", cv_image)
            img_bytes = img_encoded.tobytes()
            img_base64 = base64.b64encode(img_bytes).decode("utf-8")
            if self.sio.connected:
                self.sio.emit("Frame", {"frame": img_base64}, namespace="/rover")
                # self.get_logger().info("Send frame to server")
                # print("Send frame to server")
            else:
                # self.get_logger().info("Not connected to server")
                pass
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

    def adrnav_callback(self, msg):
        adrnav_data = json.loads(msg.data)
        self.send_to_server(name="adrnav", data=adrnav_data)
        # self.get_logger().info(f"ADRNAVA: {adrnav_data}")

    def uniheading_callback(self, msg):
        uniheading_data = json.loads(msg.data)
        self.send_to_server(name="uniheading", data=uniheading_data)
        # self.get_logger().info(f"UNIHEADINGA: {uniheading_data}")

    def rtkstatus_callback(self, msg):
        rtkstatus_data = json.loads(msg.data)
        self.send_to_server(name="rtkstatus", data=rtkstatus_data)
        # self.get_logger().info(f"RTKSTATUSA: {rtkstatus_data}")

    def rtcmstatus_callback(self, msg):
        rtcmstatus_data = json.loads(msg.data)
        self.send_to_server(name="rtcmstatus", data=rtcmstatus_data)
        # self.get_logger().info(f"RTCMSTATUSA: {rtcmstatus_data}")

    def bestnav_callback(self, msg):
        bestnav_data = json.loads(msg.data)
        self.send_to_server(name="bestnav", data=bestnav_data)
        # self.get_logger().info(f"BESTNAVXYZA: {bestnav_data}")

    def rover_mode_callback(self, msg):
        mode = msg.data
        self.send_to_server(name="rover_mode", data=mode)
    #
    #
    # action functions

    def send_to_server(self,name,data):
        if sio.connected:
            sio.emit(name, data, namespace="/rover")
            # self.get_logger().info(f"Send to server: {name} {data}")
        else:
            self.get_logger().info("Not connected to server")
    
    def send_gps_data(self):
        if self.gps_data is None or self.gps_heading is None:
            return

        location_data = {
            **self.gps_data,
            "heading": self.gps_heading,
        }
        if sio.connected:
            # self.get_logger().info(f"sending gps data to server")
            sio.emit("gps_data", location_data,namespace="/rover")
            self.gps_data = None
            self.gps_heading = None

    def connect_to_server(self):
        try:
            self.sio.connect(self.server_address)
            if self.sio.connected:
                self.get_logger().info("Connected to Socket.IO server.")
            else:
                self.get_logger().warn(
                    "Failed to connect to Socket.IO server, continuing without connection."
                )
        except socketio.exceptions.ConnectionError as e:
            self.get_logger().error(
                f"ConnectionError: {e}. Continuing without connection."
            )
        except Exception as e:
            self.get_logger().error(
                f"An unexpected error occurred: {e}. Continuing without connection."
            )

    def publish_rover_vel(self, x=None, z=None):
        if x is None or z is None:
            self.get_logger().info("x or z is None")
            return
        twist = Twist()
        twist.linear.x = float(x)
        twist.angular.z = float(z)
        self.rover_vel_pub.publish(twist)
        # self.get_logger().info(
        #     f"linear.x={twist.linear.x}, angular.z={twist.angular.z}"
        # )
        # print(f"linear.x={twist.linear.x}, angular.z={twist.angular.z}")


def main(args=None):
    rclpy.init(args=args)
    node = ConnectServer()
    node.connect_to_server()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
