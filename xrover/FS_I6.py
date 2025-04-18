import rclpy
from rclpy.node import Node
from .lib.ibus import IBusBM
from std_msgs.msg import Int32
from .lib.SerialDeviceScanner import DevicePortScanner
import signal
import time


class FS_I6(Node):
    def __init__(self):
        super().__init__("fs_i6_node")
        self.signal_green = self.create_publisher(Int32, "signal_light/green", 10)
        self.signal_yellow = self.create_publisher(Int32, "signal_light/yellow", 10)
        self.signal_red = self.create_publisher(Int32, "signal_light/red", 10)
        self.signal_buzz = self.create_publisher(Int32, "signal_light/buzz", 10)
        scanner = DevicePortScanner()
        ports = scanner.get_ports()
        self.fs_i6_port = scanner.find_fs_i6_port(ports)
        self.get_logger().info(f"FS_I6 port: {self.fs_i6_port}")
        self.ibus = IBusBM(self.fs_i6_port)

        self.ibus.reset_ibus_port()
        time.sleep(3)
        self.ibus.run_ibus()
        self.timer_loop = self.create_timer(0.005, self.ibus.loop)
        self.timer_read_chanel = self.create_timer(0.005, self.read_chanel)
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
        ch0 = Int32()
        ch1 = Int32()
        ch2 = Int32()
        ch3 = Int32()
        ch4 = Int32()
        ch5 = Int32()
        swa = Int32()
        swb = Int32()
        swc = Int32()
        swd = Int32()
        ch0.data = self.ibus.readChannel(0)
        ch1.data = self.ibus.readChannel(1)
        ch2.data = self.ibus.readChannel(2)
        ch3.data = self.ibus.readChannel(3)
        ch4.data = self.ibus.readChannel(4)
        ch5.data = self.ibus.readChannel(5)
        swa.data = self.ibus.readChannel(6)
        swb.data = self.ibus.readChannel(7)
        swc.data = self.ibus.readChannel(8)
        swd.data = self.ibus.readChannel(9)
        self.ch0_pub.publish(ch0)
        self.ch1_pub.publish(ch1)
        self.ch2_pub.publish(ch2)
        self.ch3_pub.publish(ch3)
        self.ch4_pub.publish(ch4)
        self.ch5_pub.publish(ch5)
        self.swa_pub.publish(swa)
        self.swb_pub.publish(swb)
        self.swc_pub.publish(swc)
        self.swd_pub.publish(swd)
        # self.get_logger().info(f"ch0: {self.ibus.channel[0]}")

    def handle_destroy(self):
        self.ibus.ibus_destroy()
        self.timer_loop.cancel()
        self.timer_read_chanel.cancel()
        self.get_logger().info("FS_I6 port closed")
        self.get_logger().info("FS_I6 node destroyed")
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    fs_i6 = FS_I6()
    try:
        rclpy.spin(fs_i6)
    except KeyboardInterrupt:
        pass
    finally:
        fs_i6.handle_destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
