import sys
import rclpy
import array
import base64
from rclpy.node import Node
from std_msgs.msg import String
from .lib.SerialDeviceScanner import DevicePortScanner
from .lib.ConstVariable import RF_MODULE
from PySide6.QtCore import QTimer, QObject, Signal, Slot
from PySide6.QtNetwork import QTcpSocket, QHostAddress
from PySide6.QtWidgets import QApplication
from PySide6.QtSerialPort import QSerialPort
from .lib.ConstVariable import COMMON
import os
import serial

os.environ["QT_QPA_PLATFORM"] = "offscreen"

class TcpClient(QObject):
    data_received = Signal(bytes)  # Emit signal khi nhận data dạng bytes
    disconnected = Signal()         # Thêm signal mới
    connection_failed = Signal()
    connected = Signal()
    
    def __init__(self, host="127.0.0.1", port=8765):
        super().__init__()
        self.socket = QTcpSocket(self)
        self.host = host
        self.port = port

        self.socket.connected.connect(self.on_connected)
        self.socket.readyRead.connect(self.on_ready_read)
        self.socket.errorOccurred.connect(self.on_error)
        self.socket.disconnected.connect(self.on_disconnected)

    def connect_to_server(self):
        print(f"Connecting to {self.host}:{self.port}...")
        self.socket.connectToHost(QHostAddress(self.host), self.port)

    @Slot()
    def on_connected(self):
        print(f"[CLIENT] Connected to {self.host}:{self.port}")
        self.connected.emit()

    @Slot()
    def on_ready_read(self):
        data = self.socket.readAll().data()  # data là bytes
        self.data_received.emit(data)

    @Slot()
    def on_disconnected(self):
        print(f"[CLIENT] Disconnected from {self.host}:{self.port}")
        self.disconnected.emit()

    @Slot()
    def on_error(self, socket_error):
        print(f"[CLIENT] Connection error: {socket_error}")
        self.connection_failed.emit()

    def disconnect_from_server(self):
        self.socket.disconnectFromHost()
        self.socket.close()


class RTCMReceiverNode(Node):
    def __init__(self):
        super().__init__("tcp_client_node")

        self.rtcm_publisher = self.create_publisher(String, "/gps/rtcm", 10)
        self.tcp_host = COMMON.tcp_server_ip
        self.tcp_port = COMMON.tcp_server_port
        # Tạo TCP Client
        self.client = TcpClient(host=self.tcp_host, port=self.tcp_port)
        self.client.data_received.connect(self.handle_tcp_data)
        # self.client.connected.connect(self.handle_disconnect_rf)

        self.client.disconnected.connect(self.handle_disconnected)
        self.client.connection_failed.connect(self.handle_connection_failed)
        self.scanner = DevicePortScanner()
        self.ports = self.scanner.list_serial_ports()
        self.rf_port = self.scanner.find_rf_module_port(self.ports)
        self.rf_baudrate = RF_MODULE.baudrate
        self.rf_serial = None
        
        # Kết nối tới server
        self.client.connect_to_server()
        # self.connect_to_rf_module()
        
        self.reconnect_timer = QTimer()
        self.reconnect_timer.setSingleShot(True)
        self.reconnect_timer.timeout.connect(self.client.connect_to_server)

    def connect_to_rf_module(self):
        try:
            self.rf_serial = QSerialPort(self.rf_port)
            self.rf_serial.setBaudRate(QSerialPort.BaudRate.Baud115200)
            if not self.rf_serial.open(QSerialPort.ReadWrite):
                self.get_logger().error(f"Failed to open RF module port: {self.rf_port}")
                return
            else:
                self.get_logger().info(f"RF module connected to port: {self.rf_port}")
                # self.rf_serial.readyRead.connect(self.handle_rf_data)
                
        except:
            self.get_logger().error(f"Error opening RF module port: {self.rf_port}")
    
    def format_hex(self,data: bytes) -> str:
        return " ".join(f"{b:02X}" for b in data)
    
    def handle_disconnect_rf(self):
        self.rf_serial.readyRead.disconnect(self.handle_rf_data)
    
    def handle_rf_data(self):
        rtcm_data = self.rf_serial.read(1024)
        if rtcm_data is None:
            
            return
        encoded = base64.b64encode(rtcm_data).decode('utf-8')  # bytes -> base64 string
        msg = String()
        msg.data = encoded
        self.rtcm_publisher.publish(msg)
        # self.get_logger().info(f"hex >> {rtcm_data}")
    
    def handle_tcp_data(self, data: bytes):
        encoded = base64.b64encode(data).decode('utf-8')  # bytes -> base64 string
        msg = String()
        msg.data = encoded
        self.rtcm_publisher.publish(msg)
        hex_str = self.format_hex(data)
        
        
    
    def handle_disconnected(self):
        # self.rf_serial.readyRead.connect(self.handle_rf_data)
        self.get_logger().warn("Disconnected, will try to reconnect in 1 seconds...")
        self.reconnect_timer.start(1000)
        
    def handle_connection_failed(self):
        # self.rf_serial.readyRead.connect(self.handle_rf_data)
        self.get_logger().warn("Connection failed, retrying in 1 seconds...")
        self.reconnect_timer.start(1000)
        


def main():
    rclpy.init()

    app = QApplication(sys.argv)

    node = RTCMReceiverNode()

    timer = QTimer()
    timer.start(100)
    timer.timeout.connect(lambda: None)

    def cleanup():
        print("[INFO] Disconnecting...")
        # node.client.disconnect_from_server()
        rclpy.shutdown()

    def handleIntSignal(signum, frame):
        print("Received SIGINT - Closing...")
        cleanup()
        app.quit()

    import signal
    signal.signal(signal.SIGINT, handleIntSignal)

    # Chạy ROS2 spin trong thread riêng
    from threading import Thread
    ros_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    # Chạy event loop của QApplication
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
