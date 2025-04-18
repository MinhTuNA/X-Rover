#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
import json
import os
import time
import threading
from .lib.PathManager import PathManager

class LoadPath(Node):
    def __init__(self):
        super().__init__("load_path_node")
        self.get_logger().info("load path node has been started")
        self.create_subscription(String, "/rover/move_status", self.status_callback, 10)
        self.create_subscription(String, "/start", self.start_callback, 10)
        self.create_subscription(String, "/stop", self.stop_callback, 10)
        self.segment_publisher = self.create_publisher(String, "/path/segment", 10)
        self.index = 0
        self.is_segment_done = False
        self.stop_move_event = False
        self.stop_thread_flag = False
        self.path_manager = PathManager()
        self.path_data = self.path_manager.get_path_data()
        self.is_running = False
        self.path_manager_thread = threading.Thread(target=self.start_move)
        self.path_manager_thread.start()
        self.get_logger().info("thread has been started")

    def start_move(self):
        while not self.stop_thread_flag:
            if not self.is_running:
                continue
            self.index = 0
            for i in range(self.index,len(self.path_data)-1):
                if self.stop_move_event:
                    break
                point_data1 = self.path_manager.get_point(i)
                start_lat,start_lon = self.path_manager.get_coordinate(point_data1)
                point_data2 = self.path_manager.get_point(i+1)
                end_lat,end_lon = self.path_manager.get_coordinate(point_data2)
                self.send_segment(start_lat,start_lon,end_lat,end_lon)
                self.get_logger().info(f"send segment {start_lat},{start_lon} -> {end_lat},{end_lon}")
                self.wait_segment_done()
                # self.get_logger().info(f"segment {i} done")
                if i == len(self.path_data)-2:
                    self.get_logger().info("last segment")
                    self.stop_move_event = True
                    self.is_running = False
                    break
                if self.stop_move_event:
                    break
            self.index = 0
    
    def wait_segment_done(self):
        while True:
            if self.stop_move_event:
                self.is_segment_done = True
                self.get_logger().info("stop move event")
                break
            # time.sleep(1)
            # self.is_segment_done = True
            if self.is_segment_done:
                self.get_logger().info("segment done")
                break

    def start_callback(self, msg):
        self.is_running = True
        self.stop_move_event = False
        
        
    
    def stop_callback(self, msg):
        self.stop_move_event = True
        self.is_running = False
        self.get_logger().info("stopped")
        

    def status_callback(self, msg):
        status = msg.data
        self.get_logger().info(f"status >> {status}")

        if status == "segment_done":
            self.is_segment_done = True

    def send_segment(self, start_lat, start_lon, end_lat, end_lon):
        segment = {
            "start_lat": start_lat,
            "start_lon": start_lon,
            "end_lat": end_lat,
            "end_lon": end_lon
        }
        segment = json.dumps(segment)
        segment_msg = String()
        segment_msg.data = segment
        self.segment_publisher.publish(segment_msg)
    
    def handle_destroy(self):
        self.get_logger().info("destroy")
        self.stop_move_event = True
        self.stop_thread_flag = True
        self.path_manager_thread.join()
        self.get_logger().info("stopped")
        self.is_running = False
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    load_path = LoadPath()
    try:
        rclpy.spin(load_path)
    except KeyboardInterrupt:
        pass
    finally:
        load_path.handle_destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
