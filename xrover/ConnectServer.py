#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String
import json
import os
import socketio

sio = socketio.Client()


class ConnectServer(Node):
    def __init__(self):
        super().__init__("connect_server_node")
        self.gps_data = None
        self.imu_heading = None
        self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, 10)
        self.create_subscription(Float32, "/imu/heading", self.imu_callback, 10)
        self.publish_program_cmd = self.create_publisher(String, "/program_cmd", 10)

    def gps_callback(self, msg):
        """Xử lý dữ liệu GPS."""
        self.gps_data = {
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
        }
        self.send_data_to_server()

    def imu_callback(self, msg):
        self.imu_heading = msg.data
        self.send_data_to_server()

    def send_data_to_server(self):
        if self.gps_data is None or self.imu_heading is None:
            return

        location_data = {
            **self.gps_data,
            "heading": self.imu_heading,
        }
        if sio.connected:
            sio.emit("GPSdataFromXrower", location_data)
            self.gps_data = None
            self.imu_heading = None

    def connect_to_server(self):
        try:
            sio.connect("http://192.168.1.214:8901")
            if sio.connected:
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

    def run(self, id):
        cmd = {
            "program_id": id,
            "cmd": "run",
        }
        cmd_json = json.dumps(cmd)

        msg = String()
        msg.data = cmd_json
        self.publish_program_cmd.publish(msg)
        # self.get_logger().info(f"cmd: {cmd}")


@sio.event
def connect():
    print("Connected to server")


@sio.event
def program(data):
    # print("Received from server:", data)
    file_path = "program.json"

    if os.path.exists(file_path):
        print(f"File {file_path} already exists. Updating contents...")
    else:
        print(f"File {file_path} does not exist. Creating new file...")

    with open(file_path, "w") as json_file:
        json.dump(data, json_file, indent=4)

    print(f"Data has been saved to {file_path}")


@sio.event
def run_program(data):
    node.run(id=data)

@sio.event
def cmd_vel(cmd_vel):
    print(f"cmd vel >> x = {cmd_vel}")
    print(f"cmd vel >> z = {cmd_vel}")

    

@sio.event
def disconnect():
    print("Disconnected from server")


def main(args=None):
    rclpy.init(args=args)
    global node
    node = ConnectServer()
    node.connect_to_server()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
