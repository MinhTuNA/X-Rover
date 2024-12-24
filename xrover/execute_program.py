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
        self.create_subscription(String, "/status", self.status_callback, 10)
        self.create_subscription(String, "/program_cmd", self.program_cmd_callback, 10)
        self.program_publisher = self.create_publisher(NavSatFix, "/gps/goal", 10)
        self.program = None
        self.index = 0

    def program_cmd_callback(self, msg):
        cmd = json.loads(msg.data)
        program_id = cmd["program_id"]
        command = cmd["cmd"]
        file_path = "program.json"
        try:
            with open(file_path, "r") as json_file:
                programs = json.load(json_file)

                for program in programs:
                    if int(program_id) == int(program["id"]):
                        # self.get_logger().info(
                        #     f"ID: {program['id']}, Name: {program['name']}")
                        self.program = program["coordinates"]
                        # self.get_logger().info(f'program >> {self.program}')
                        break
                    # print(f"ID: {program['id']}, Name: {program['name']}")
                    # for coord in program['coordinates']:
                    #     print(f"Coordinate: {coord}")
                self.send_program()
        except FileNotFoundError:
            self.get_logger().error(f"File {file_path} not found.")
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error reading JSON: {e}")

    def status_callback(self, msg):
        str_msg = json.loads(msg.data)
        status = str_msg["status"]
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
