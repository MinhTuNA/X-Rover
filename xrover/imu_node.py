import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
import serial
from .const import *
from .h30_imu import YesenseDecoder
import time


class IMU_Node(Node):
    def __init__(self):
        super().__init__("imu_node")
        self.get_logger().info("IMU Node Initialized")
        self.imu_publisher = self.create_publisher(Imu, "/imu/data", 10)
        self.port = H30_IMU_PORT
        self.baudrate = H30_IMU_BAUDRATE

        self.decoder = YesenseDecoder()
        self.imu_sensor()
        # self.uart_timer = self.create_timer(0.1, self.read_from_uart)

    def imu_sensor(self):
        consecutive_errors = 0
        last_time = time.time()
        try:
            with serial.Serial(
                port=self.port, baudrate=self.baudrate, timeout=1
            ) as ser:
                ser.flushInput()
                while True:
                    data = self.decoder.read_from_uart_(ser)
                    if isinstance(data, dict):
                        tid = data.get("tid")
                        acc = data.get("acc")
                        gyro = data.get("gyro")
                        euler = data.get("euler")
                        quat = data.get("quat")

                        consecutive_errors = 0
                        imu_msg = Imu()
                        imu_msg.header = Header()
                        imu_msg.header.stamp = self.get_clock().now().to_msg()
                        imu_msg.header.frame_id = "base_link"

                        # Orientation (quaternion)
                        imu_msg.orientation = Quaternion(
                            x=quat[1],
                            y=quat[2],
                            z=quat[3],
                            w=quat[0],
                        )
                        imu_msg.orientation_covariance = [
                            0.0
                        ] * 9  # Không có độ không chắc chắn

                        # Angular velocity (gyro)
                        imu_msg.angular_velocity.x = gyro[0]
                        imu_msg.angular_velocity.y = gyro[1]
                        imu_msg.angular_velocity.z = gyro[2]
                        imu_msg.angular_velocity_covariance = [0.0] * 9

                        # Linear acceleration (acc)
                        imu_msg.linear_acceleration.x = acc[0]
                        imu_msg.linear_acceleration.y = acc[1]
                        imu_msg.linear_acceleration.z = acc[2]
                        imu_msg.linear_acceleration_covariance = [0.0] * 9

                        # Log Euler angles (optional)
                        roll = euler[0]
                        pitch = euler[1]
                        yaw = euler[2]
                        self.get_logger().info(
                            f"\neuler >> {int(euler[0])} {int(euler[1])} {int(euler[2])}"
                        )

                        self.imu_publisher.publish(imu_msg)

                    elif data == "BUF_FULL":
                        consecutive_errors += 1
                        if consecutive_errors >= 3:
                            print("Too many buffer full errors")
                            consecutive_errors = 0
                            # pass
                            break
                            # pass
                    elif "Error" in data:
                        print(data)
                        # pass
                        # break

                    current_time = time.time()
                    elapsed_time = (current_time - last_time) * 1000
                    self.decoder.timing_count += elapsed_time

                    if self.decoder.timing_count >= CNT_PER_SECOND:
                        self.decoder.msg_rate = self.decoder.msg_count
                        self.decoder.msg_count = 0
                        self.decoder.timing_count = 0
                        self.get_logger().info(
                            f"Message Rate: {self.decoder.msg_rate} msgs/sec"
                        )

                    last_time = current_time
        except serial.SerialException as e:
            print(f"UART Error: {e}")

    def destroy_node(self):
        self.serial_port.close()
        super().destroy_node()
        self.get_logger().info("Stop IMU node")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = IMU_Node()
        rclpy.spin(node, timeout_sec=0.1)
    except Exception as e:
        print(e)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
