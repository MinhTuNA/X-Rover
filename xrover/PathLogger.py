import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from .lib.ConstVariable import COMMON
from sensor_msgs.msg import NavSatFix
import requests
class PathLogger(Node):
    def __init__(self):
        super().__init__("path_logger")
        self.get_logger().info("path logger has been initialized")
        self.create_subscription(String, "/logger/start", self.start_callback, 10)
        self.create_subscription(String, "/logger/stop", self.stop_callback, 10)
        self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, 10)
        self.is_running = False
        self.index = 0
        self.lat = None
        self.lon = None
        self.logger_timer = self.create_timer(0.5, self.logger_callback)
        self.file = None
        self.last_lat = None
        self.last_lon = None
        
    def open_file(self):
        try:
            open(COMMON.file_path_logger, "w").close()
            self.file = open(COMMON.file_path_logger, "a")
            self.get_logger().info(f"file opened")
        except Exception as e:
            self.get_logger().error(f"Error opening file: {e}")
    
    def gps_callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude

    def logger_callback(self):
        if self.is_running:
            if self.lat is None or self.lon is None:
                self.get_logger().info("no gps data")
                return
            if self.last_lat == self.lat and self.last_lon == self.lon:
                self.get_logger().info("no change")
                return
            # self.get_logger().info("logging")
            entry = {
                "index": self.index,
                "lat": self.lat,
                "lon": self.lon,
            }
            self.last_lat = self.lat
            self.last_lon = self.lon
            self.lat = None
            self.lon = None
            self.file.write(f"{json.dumps(entry)}\n")
            self.file.flush()
            self.index += 1
        else:
            # self.get_logger().info("not logging")
            pass
    def start_callback(self, msg):

        self.open_file()
        self.index = 0
        self.is_running = True
        self.get_logger().info(f"start logging {msg.data}")

    def stop_callback(self, msg):
        self.is_running = False
        self.file.close()
        try:
            with open(COMMON.file_path_logger, "r") as f:
                data = [json.loads(line.strip()) for line in f]
            # self.get_logger().info(f"file saved {data}")
            self.update_path(msg.data, data)
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
        self.get_logger().info(f"stop logging {msg.data}")

    def update_path(self, id, data):
        try:
            payload = {
                'lines': data
            }
            self.get_logger().info(f"data: {payload}")
            response = requests.patch(COMMON.api_save_path+str(id), json=payload)
            self.get_logger().info(f"response: {response.json()}")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def handle_destroy(self):
        self.get_logger().info("path logger has been destroyed")
        if self.file is not None:
            self.file.close()
        self.destroy_node()

def main(args=None):
    rclpy.init(args=args)
    path_logger = PathLogger()
    try:
        rclpy.spin(path_logger)
    except KeyboardInterrupt:
        pass
    finally:
        path_logger.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
