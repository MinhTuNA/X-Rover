import sys
import array
from lib.ConstVariable import RF_MODULE
from PySide6.QtCore import QTimer, QObject, Signal, Slot
from PySide6.QtNetwork import QTcpSocket, QHostAddress
from PySide6.QtWidgets import QApplication
from PySide6.QtSerialPort import QSerialPort
from lib.ConstVariable import COMMON
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
        # print(f"RTCMReceiver: >> Connecting to {self.host}:{self.port}...")
        self.socket.connectToHost(QHostAddress(self.host), self.port)

    @Slot()
    def on_connected(self):
        print(f"RTCMReceiver: >> Connected to {self.host}:{self.port}")
        self.connected.emit()

    @Slot()
    def on_ready_read(self):
        data = self.socket.readAll().data()  # data là bytes
        self.data_received.emit(data)

    @Slot()
    def on_disconnected(self):
        print(f"RTCMReceiver: >> Disconnected {self.host}:{self.port}")
        self.disconnected.emit()

    @Slot()
    def on_error(self, socket_error):
        print(f"RTCMReceiver: >> Connection error: {socket_error}")
        self.connection_failed.emit()
        self.socket.abort()
        QTimer.singleShot(1000, self.connect_to_server)

    def _disconnect_(self):
        self.socket.disconnectFromHost()
        self.socket.close()


class RTCMReceiver(QObject):
    rtcm_data_signal = Signal(bytes)
    def __init__(self,port = None):
        super().__init__()
        self.tcp_host = None
        self.tcp_port = None
        self.client = None
        self.rf_port = port
        self.rf_baudrate = None
        self.rf_serial = None
    
    def init_rtcm(self):
        self.tcp_host = COMMON.tcp_server_ip
        self.tcp_port = COMMON.tcp_server_port
       
        self.client = TcpClient(host=self.tcp_host, port=self.tcp_port)
        self.client.data_received.connect(self.handle_tcp_data)

        self.client.disconnected.connect(self.handle_disconnected)
        self.client.connection_failed.connect(self.handle_connection_failed)
        self.rf_baudrate = RF_MODULE.baudrate
        self.rf_serial = None
        self.client.connect_to_server()
        self.connect_to_rf_module()
        

    def connect_to_rf_module(self):
        try:
            self.rf_serial = QSerialPort(self.rf_port)
            self.rf_serial.setBaudRate(QSerialPort.BaudRate.Baud57600)
            if not self.rf_serial.open(QSerialPort.ReadWrite):
                print(f"RTCMReceiver: >> Failed to open RF module port: {self.rf_port}")
                return
            else:
                print(f"RTCMReceiver: >> RF module connected to port: {self.rf_port}")
                self.rf_serial.readyRead.connect(self.handle_rf_data)
                
        except:
            print(f"RTCMReceiver: >> Error opening RF module port: {self.rf_port}")
    
    def format_hex(self,data: bytes) -> str:
        return " ".join(f"{b:02X}" for b in data)
    
    def handle_disconnect_rf(self):
        if self.rf_serial:
            self.rf_serial.readyRead.disconnect(self.handle_rf_data)
    
    def handle_rf_data(self):
        rtcm_qba = self.rf_serial.readAll()
        if not rtcm_qba:
            return
        rtcm_data: bytes = bytes(rtcm_qba)
        self.rtcm_data_signal.emit(rtcm_data)
        hex_str = self.format_hex(rtcm_data)
        # print(f"RTCMReceiver: >> RF data: {hex_str}")
    def handle_tcp_data(self, data: bytes):
        self.rtcm_data_signal.emit(data)
        hex_str = self.format_hex(data)
        # print(f"RTCMReceiver: >> TCP data: {hex_str}")
    
    def handle_disconnected(self):
        # print("RTCMReceiver: >> Disconnected, will try to reconnect in 1 seconds...")
        pass
        
    def handle_connection_failed(self):
        # print("RTCMReceiver: >> Connection failed, retrying in 1 seconds...")
        pass
     
    
    def clean_up(self):
        self.client._disconnect_()
        self.rf_serial.close()
        print(f"RTCMReceiver: >> closed RF Module: {self.rf_port}")
        


def main():
    app = QApplication(sys.argv)

    rtcm_receiver = RTCMReceiver()

    timer = QTimer()
    timer.start(100)
    timer.timeout.connect(lambda: None)

    def cleanup():
        print("RTCMReceiver: >> Disconnecting...")
        rtcm_receiver.clean_up()
    def handleIntSignal(signum, frame):
        print("RTCMReceiver: >> Received SIGINT - Closing...")
        cleanup()
        app.quit()

    import signal
    signal.signal(signal.SIGINT, handleIntSignal)

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
