#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Image, Imu
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist, Point
import cv2
import base64
from cv_bridge import CvBridge
import json
import os
import socketio
from .lib.ConstVariable import COMMON
sio = socketio.Client()


class ConnectServer(Node):
    def __init__(self):
        super().__init__("connect_server_node")
        self.server_address = COMMON.server_address
        self.gps_data = None
        self.gps_heading = None
        self.create_subscription(
            NavSatFix,
            "/gps/fix",
            self.gps_callback,
            10
        )
        self.create_subscription(
            Float32,
            "/gps/heading",
            self.heading_callback,
            10
        )
        self.create_subscription(
            Imu,
            "/imu/data",
            self.imu_callback,
            10
        )
        self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback,
            10
        )
        self.create_subscription(
            String,
            "/status",
            self.status_callback,
            10
        )

        self.publish_program_cmd = self.create_publisher(
            String,
            "/program_cmd",
            10
        )
        self.rover_vel_pub = self.create_publisher(
            Twist,
            "/rover/vel",
            10
        )
        self.move_delta_pub = self.create_publisher(
            Point,
            "/delta/move",
            10
        )
        self.delta_go_home_pub = self.create_publisher(
            String,
            "delta/go_home",
            10
        )
        self.request_delta_status_pub = self.create_publisher(
            String,
            "delta/request_status",
            10
        )

        # socketio
        self.sio = sio
        self.is_connected = False
        self.hasRegisteredEvents = False

        if not self.hasRegisteredEvents:
            self.sio.on('connect', self.on_connect, namespace='/rover')
            self.sio.on('program', self.program, namespace='/rover')
            self.sio.on('move_rover', self.rover_vel, namespace='/rover')
            self.sio.on('run_program', self.run_program, namespace='/rover')
            self.sio.on('delta_go_home', self.delta_go_home,
                        namespace='/rover')
            self.sio.on('move_delta', self.move_delta, namespace='/rover')
            self.sio.on('disconnect', self.disconnect, namespace='/rover')
            self.sio.on('request_delta_status',
                        self.request_delta_status, namespace='/rover')

    # socketio events

    def on_connect(self):
        self.is_connected = True
        print("- Connected to server")

    def program(self, data):
        # print("Received from server:", data)
        file_path = "program.json"

        if os.path.exists(file_path):
            print(f"File {file_path} already exists. Updating contents...")
        else:
            print(f"File {file_path} does not exist. Creating new file...")

        with open(file_path, "w") as json_file:
            json.dump(data, json_file, indent=4)

        print(f"Data has been saved to {file_path}")

    def rover_vel(self, data):
        x = data["x"]
        z = data["z"]
        self.publish_rover_vel(x, z)
        # print(f"cmd vel: x: {x} || z : {z}")

    def run_program(self, data):
        self.get_logger().info("Received from server:", data)
        id = data
        cmd = {
            "program_id": id,
            "cmd": "run",
        }
        cmd_json = json.dumps(cmd)

        msg = String()
        msg.data = cmd_json
        self.publish_program_cmd.publish(msg)
        # self.get_logger().info(f"cmd: {cmd}")

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
        self.get_logger().info("requesting delta status")
        self.request_delta_status_pub.publish(signal)

    def disconnect():
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

    def heading_callback(self, msg):
        self.gps_heading = msg.data
        self.send_gps_data()

    def imu_callback(self, msg):
        acc = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ]
        gyro = [msg.angular_velocity.x,
                msg.angular_velocity.y, msg.angular_velocity.z]
        quat = [
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
        ]
        if (sio.connected):
            self.sio.emit("IMUDataFromXrover", {
                "acc": acc,
                "gyro": gyro,
                "quat": quat
            }, namespace="/rover")
            self.get_logger().info("Send IMU data to server")

    def image_callback(self, msg):
        try:
            cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
            _, img_encoded = cv2.imencode(".jpg", cv_image)
            img_bytes = img_encoded.tobytes()
            img_base64 = base64.b64encode(img_bytes).decode('utf-8')
            if self.sio.connected:
                self.sio.emit(
                    "Frame", {"frame": img_base64}, namespace='/rover')
                # self.get_logger().info("Send frame to server")
                # print("Send frame to server")
            else:
                self.get_logger().info("Not connected to server")
        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")

    def status_callback(self, msg):
        self.get_logger().info(f"Status: {msg.data}")
        if (self.sio.connected):
            self.sio.emit("status", msg.data, namespace="/rover")
            self.get_logger().info("Send status to server")
    #
    #
    #
    #
    # action functions

    def send_gps_data(self):
        if self.gps_data is None or self.gps_heading is None:
            return

        location_data = {
            **self.gps_data,
            "heading": self.gps_heading,
        }
        if sio.connected:
            sio.emit("GPSdataFromXrover", location_data)
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
        if (x is None or z is None):
            self.get_logger().info("x or z is None")
            return
        twist = Twist()
        twist.linear.x = float(x)
        twist.angular.z = float(z)
        self.rover_vel_pub.publish(twist)
        self.get_logger().info(
            f"linear.x={twist.linear.x}, angular.z={twist.angular.z}")
        # print(f"linear.x={twist.linear.x}, angular.z={twist.angular.z}")


def main(args=None):
    rclpy.init(args=args)
    node = ConnectServer()
    node.connect_to_server()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
