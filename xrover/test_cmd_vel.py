import rclpy
import random
from rclpy.node import Node
from geometry_msgs.msg import Twist

class cmd_vel(Node):
    def __init__(self):
        super().__init__("cmd_vel_node")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer = self.create_timer(10, self.publish_cmd_vel)
      
    def publish_cmd_vel(self):
        twist = Twist()
        twist.linear.x = random.uniform(-1, 1)
        twist.angular.z = random.uniform(-1, 1)
        self.cmd_vel_pub.publish(twist)  
        self.get_logger().info(f"linear.x={twist.linear.x}, angular.z={twist.angular.z}")
        print(f"linear.x={twist.linear.x}, angular.z={twist.angular.z}")
    
    def destroy_node(self):
        self.get_logger().info("Destroying node")
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    cmd_vel_node = cmd_vel()
    rclpy.spin(cmd_vel_node)
    cmd_vel_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()