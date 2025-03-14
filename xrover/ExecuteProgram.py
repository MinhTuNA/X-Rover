#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import json
import os



class ExecuteProgram(Node):
    def __init__(self):
        super().__init__("execute_program_node")
        self.get_logger().info("execute node has been started")
        self.create_subscription(String, "/rover/move_status", self.status_callback, 10)
        self.create_subscription(String, "/program_cmd", self.program_cmd_callback, 10)
        self.program_publisher = self.create_publisher(NavSatFix, "/gps/goal", 10)
        self.program = None
        self.index = 0
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        self.file_path = os.path.join(self.current_dir, "path.json")

    def program_cmd_callback(self, msg):
        cmd = json.loads(msg.data)
        try:
            with open(self.file_path, "r") as json_file:
                path = json.load(json_file)

                for line in path:
                    self.get_logger().info(f"line >> {line}")
                # self.send_program()
        except FileNotFoundError:
            self.get_logger().error(f"File {self.file_path} not found.")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error reading JSON: {e}")

    def status_callback(self, msg):
        status = msg.data
        self.get_logger().info(f"status >> {status}")

        if status == "done":
            self.send_program()

    def send_program(self):
        if self.program is None or len(self.program) == 0:
            return
        if self.index >= len(self.program):
            self.index = 0
            return

        goal = NavSatFix()
        goal.longitude = self.program[self.index][0]
        goal.latitude = self.program[self.index][1]
        self.get_logger().info(f"point : {self.program[self.index]}")
        self.program_publisher.publish(goal)
        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    execute = ExecuteProgram()
    rclpy.spin(execute)
    execute.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
