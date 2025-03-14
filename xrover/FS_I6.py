import rclpy
from rclpy.node import Node
from .lib.ibus import IBusBM
from std_msgs.msg import Int32
from .lib.SerialDeviceScanner import DevicePortScanner


class FS_I6(Node):
    def __init__(self):
        super().__init__("fs_i6_node")
        scanner = DevicePortScanner()
        self.fs_i6_port = scanner.find_fs_i6_port()
        self.get_logger().info(f"FS_I6 port: {self.fs_i6_port}")
        self.ibus = IBusBM(self.fs_i6_port)
        self.ibus.run_ibus()
        self.create_timer(0.001, self.ibus.loop)
        self.create_timer(0.001, self.read_chanel)
        self.ch0_pub = self.create_publisher(Int32, "fs_i6/ch0", 10)
        self.ch1_pub = self.create_publisher(Int32, "fs_i6/ch1", 10)
        self.ch2_pub = self.create_publisher(Int32, "fs_i6/ch2", 10)
        self.ch3_pub = self.create_publisher(Int32, "fs_i6/ch3", 10)
        self.ch4_pub = self.create_publisher(Int32, "fs_i6/ch4", 10)
        self.ch5_pub = self.create_publisher(Int32, "fs_i6/ch5", 10)
        self.swa_pub = self.create_publisher(Int32, "fs_i6/swa", 10)
        self.swb_pub = self.create_publisher(Int32, "fs_i6/swb", 10)
        self.swc_pub = self.create_publisher(Int32, "fs_i6/swc", 10)
        self.swd_pub = self.create_publisher(Int32, "fs_i6/swd", 10)

    def read_chanel(self):
        self.ibus.readChannel(0)
        self.ibus.readChannel(1)
        self.ibus.readChannel(2)
        self.ibus.readChannel(3)
        self.ibus.readChannel(4)
        self.ibus.readChannel(5)
        self.ibus.readChannel(6)
        self.ibus.readChannel(7)
        self.ibus.readChannel(8)
        self.ibus.readChannel(9)


def main(args=None):
    rclpy.init(args=args)
    try:
        fs_i6 = FS_I6()
        rclpy.spin(fs_i6, timeout_sec=0.1)
    except Exception as e:
        print(e)
    finally:
        fs_i6.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
