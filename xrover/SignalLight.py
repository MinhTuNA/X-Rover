import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from .lib.SignalLightLib import SignalLightLib


class SignalLight(Node):
    def __init__(self):
        super().__init__("signal_light")
        self.signal_light_lib = SignalLightLib()
        self.signal_light_lib.set_red(0)
        self.signal_light_lib.set_yellow(0)
        self.signal_light_lib.set_green(0)
        self.signal_light_lib.set_buzz(0)
        self.sub_red = self.create_subscription(
            Int32, "signal_light/red", self.red_callback, 10
        )
        self.sub_yellow = self.create_subscription(
            Int32, "signal_light/yellow", self.yellow_callback, 10
        )
        self.sub_green = self.create_subscription(
            Int32, "signal_light/green", self.green_callback, 10
        )
        self.sub_buzz = self.create_subscription(
            Int32, "signal_light/buzz", self.buzz_callback, 10
        )

    def red_callback(self, msg):
        value = int(msg.data)
        self.signal_light_lib.set_red(value)

    def yellow_callback(self, msg):
        value = int(msg.data)
        self.signal_light_lib.set_yellow(value)

    def green_callback(self, msg):
        value = int(msg.data)
        self.signal_light_lib.set_green(value)

    def buzz_callback(self, msg):
        value = int(msg.data)
        self.signal_light_lib.set_buzz(value)

    def handle_destroy(self):
        self.signal_light_lib.cleanup()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    signal_light = SignalLight()
    try:
        rclpy.spin(signal_light)
    except KeyboardInterrupt:
        signal_light.get_logger().info("Keyboard interrupt, shutting down...")
    finally:
        signal_light.handle_destroy()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
