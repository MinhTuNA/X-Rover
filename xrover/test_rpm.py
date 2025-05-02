#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
import random

class RpmPublisherNode(Node):
    def __init__(self):
        super().__init__('rpm_publisher')
        
        # Create publisher for navigation/rpm topic
        self.publisher = self.create_publisher(
            Float32MultiArray,
            'navigation/rpm',
            10
        )
        
        # Create timer to publish at 10Hz
        self.timer = self.create_timer(0.02, self.publish_rpm)
        self.get_logger().info('RPM publisher node started')
        
    def publish_rpm(self):
        # Create message
        msg = Float32MultiArray()
        
        # Simulated RPM values
        left_rpm = 5.0
        right_rpm = 5.0
        
        msg.data = [float(left_rpm), float(right_rpm)]
        
        # Publish message
        self.publisher.publish(msg)
        self.get_logger().info(f'Published RPM: left={left_rpm:.2f}, right={right_rpm:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = RpmPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
