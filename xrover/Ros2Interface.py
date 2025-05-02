import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32,Float32MultiArray
from PySide6.QtCore import QObject,QThread,Signal,Slot

class Ros2Interface(QObject):
    rpm_signal = Signal(float,float)

    def __init__(self):
        super().__init__()
        self.node = None

    def rpm_callback(self, msg: Float32MultiArray):
        left_rpm = msg.data[0]
        right_rpm = msg.data[1]
        self.rpm_signal.emit(left_rpm,right_rpm)
        # print(f'Ros2Interface:> Received rpm: {left_rpm}, {right_rpm}')

    def run(self):
        rclpy.init()
        self.node = Node('ros2_interface')
        self.node.create_subscription(Float32MultiArray, 'navigation/rpm', self.rpm_callback, 10)
        print('Ros2Interface:> initialized')
        rclpy.spin(self.node)
        
    
    def clean_up(self):
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('Ros2Interface:> rclpy shutdown complete')