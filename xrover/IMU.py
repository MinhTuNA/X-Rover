from PySide6.QtCore import QObject, Signal
from PySide6.QtSerialPort import QSerialPort
import time
from lib.IMULib import YesenseDecoder
from lib.ConstVariable import IMU


class IMUReader(QObject):
    imu_signal = Signal(dict)
    error = Signal(str)

    def __init__(self, port: str, parent=None):
        super().__init__(parent)
        self._port = port
        self._baudrate = IMU.baudrate
        self._buf_len = IMU.uart_buf_len
        self._cnt_per_sec = IMU.cnt_per_seconds
        self.decoder = YesenseDecoder()
        self.serial_port = None
        self.last_time = time.time()
    
    def connect_imu(self):
        self.serial_port = QSerialPort()
        self.serial_port.setPortName(self._port)
        self.serial_port.setBaudRate(self._baudrate)
        if not self.serial_port.open(QSerialPort.ReadOnly):
            self.error.emit(f"Cannot open {self._port}: {self.serial_port.errorString()}")
            return
        print(f"IMU: >> connected IMU :{self._port}")
        self.serial_port.clear(QSerialPort.AllDirections)
        self.serial_port.readyRead.connect(self.run)
    
    def clean_up(self):
        self.serial_port.readyRead.disconnect(self.run)
        self.serial_port.close()
        print(f"IMU: >> closed IMU :{self._port}")

    def run(self):
        
        consecutive_errors = 0

        raw = bytes(self.serial_port.readAll())
        result = {}
        ret = self.decoder.data_proc(raw, result)
        if ret == "ANALYSIS_OK":
            self.imu_signal.emit(result)
            self.decoder.msg_count += 1
            consecutive_errors = 0

        elif ret == "BUF_FULL":
            consecutive_errors += 1
            if consecutive_errors >= 3:
                self.error.emit("Too many buffer full errors")
                return 

        elif ret == "ANALYSIS_ERROR":
            self.error.emit(result)
            
        # Cập nhật msg rate
        now = time.time()
        elapsed = (now - self.last_time) * 1000
        self.decoder.timing_count += elapsed
        if self.decoder.timing_count >= int(self._cnt_per_sec):
            self.decoder.msg_rate = self.decoder.msg_count
            # print(f"IMU: >> rate: {self.decoder.msg_rate}")
            self.decoder.msg_count = 0
            self.decoder.timing_count = 0
        self.last_time = now

