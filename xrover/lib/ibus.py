import time
import serial
from PySide6.QtCore import QObject, Signal as pyqtSignal, Slot
from PySide6.QtSerialPort import QSerialPort
import atexit
import threading
import subprocess

class IBusBM:
    PROTOCOL_LENGTH = 0x20  # Tổng độ dài tối đa của gói tin
    PROTOCOL_OVERHEAD = 3  # Overhead: 1 byte lệnh + 2 byte checksum
    PROTOCOL_TIMEGAP = 3  # Thời gian chờ giữa các gói (ms)
    PROTOCOL_CHANNELS = 14  # Số kênh servo tối đa
    PROTOCOL_COMMAND40 = 0x40  # Lệnh servo
    PROTOCOL_COMMAND_DISCOVER = 0x80  # Lệnh DISCOVER sensor
    PROTOCOL_COMMAND_TYPE = 0x90  # Lệnh trả về loại sensor
    PROTOCOL_COMMAND_VALUE = 0xA0  # Lệnh trả về dữ liệu sensor
    SENSORMAX = 10  # Số sensor tối đa

    STATE_GET_LENGTH = 0
    STATE_GET_DATA = 1
    STATE_GET_CHKSUML = 2
    STATE_GET_CHKSUMH = 3
    STATE_DISCARD = 4

    IBusBMfirst = None

    def __init__(self, ibus_port):
        super().__init__()
        self.ibus_port = ibus_port
        self.ibus_baudrate = 115200
        self.stream = None
        self.state = self.STATE_DISCARD
        self.last = self.current_millis()
        self.ptr = 0
        self.len = 0
        self.chksum = 0
        self.lchksum = 0
        self.buffer = bytearray(self.PROTOCOL_LENGTH)
        self.channel = [0] * self.PROTOCOL_CHANNELS

        self.sensors = []
        self.NumberSensors = 0

        self.cnt_poll = 0
        self.cnt_sensor = 0
        self.cnt_rec = 0

        if IBusBM.IBusBMfirst is None:
            IBusBM.IBusBMfirst = self
        else:
            current = IBusBM.IBusBMfirst
            while hasattr(current, "IBusBMnext") and current.IBusBMnext is not None:
                current = current.IBusBMnext
            current.IBusBMnext = self
        self.IBusBMnext = None

    def current_millis(self):
        return int(time.monotonic() * 1000)

    def loop(self):
        if self.stream is None:
            return

        if self.IBusBMnext:
            self.IBusBMnext.loop()

        while self.stream.in_waiting > 0:
            now = self.current_millis()
            if now - self.last >= self.PROTOCOL_TIMEGAP:
                self.state = self.STATE_GET_LENGTH
            self.last = now

            v_byte = self.stream.read(1)
            if len(v_byte) == 0:
                break
            v = v_byte[0]

            if self.state == self.STATE_GET_LENGTH:
                if v <= self.PROTOCOL_LENGTH and v > self.PROTOCOL_OVERHEAD:
                    self.ptr = 0
                    self.len = v - self.PROTOCOL_OVERHEAD
                    self.chksum = (0xFFFF - v) & 0xFFFF
                    self.state = self.STATE_GET_DATA
                else:
                    self.state = self.STATE_DISCARD
            elif self.state == self.STATE_GET_DATA:
                # Lưu byte vào buffer
                self.buffer[self.ptr] = v
                self.ptr += 1
                self.chksum = (self.chksum - v) & 0xFFFF  # đảm bảo 16-bit
                if self.ptr == self.len:
                    self.state = self.STATE_GET_CHKSUML
            elif self.state == self.STATE_GET_CHKSUML:
                self.lchksum = v
                self.state = self.STATE_GET_CHKSUMH
            elif self.state == self.STATE_GET_CHKSUMH:
                received_checksum = ((v << 8) | self.lchksum) & 0xFFFF
                if self.chksum == received_checksum:
                    command = self.buffer[0]
                    adr = command & 0x0F
                    if command == self.PROTOCOL_COMMAND40:
                        for i in range(1, self.PROTOCOL_CHANNELS * 2 + 1, 2):
                            channel_index = (i - 1) // 2
                            if i + 1 < len(self.buffer):
                                self.channel[channel_index] = self.buffer[i] | (
                                    self.buffer[i + 1] << 8
                                )
                        self.cnt_rec += 1
                    elif adr <= self.NumberSensors and adr > 0 and self.len == 1:
                        time.sleep(0.0001)
                        s = self.sensors[adr - 1]
                        cmd_type = command & 0xF0
                        if cmd_type == self.PROTOCOL_COMMAND_DISCOVER:
                            self.cnt_poll += 1
                            response = bytearray()
                            response.append(0x04)
                            response.append(self.PROTOCOL_COMMAND_DISCOVER + adr)
                            self.chksum = (
                                0xFFFF - (0x04 + self.PROTOCOL_COMMAND_DISCOVER + adr)
                            ) & 0xFFFF
                            response += self.chksum.to_bytes(2, byteorder="little")
                            self.stream.write(response)
                        elif cmd_type == self.PROTOCOL_COMMAND_TYPE:
                            response = bytearray()
                            response.append(0x06)
                            response.append(self.PROTOCOL_COMMAND_TYPE + adr)
                            response.append(s["sensorType"])
                            response.append(s["sensorLength"])
                            self.chksum = (
                                0xFFFF
                                - (
                                    0x06
                                    + self.PROTOCOL_COMMAND_TYPE
                                    + adr
                                    + s["sensorType"]
                                    + s["sensorLength"]
                                )
                            ) & 0xFFFF
                            response += self.chksum.to_bytes(2, byteorder="little")
                            self.stream.write(response)
                        elif cmd_type == self.PROTOCOL_COMMAND_VALUE:
                            self.cnt_sensor += 1
                            response = bytearray()
                            t = 0x04 + s["sensorLength"]
                            response.append(t)
                            self.chksum = (0xFFFF - t) & 0xFFFF
                            t = self.PROTOCOL_COMMAND_VALUE + adr
                            response.append(t)
                            self.chksum = (self.chksum - t) & 0xFFFF
                            t = s["sensorValue"] & 0xFF
                            response.append(t)
                            self.chksum = (self.chksum - t) & 0xFFFF
                            t = (s["sensorValue"] >> 8) & 0xFF
                            response.append(t)
                            self.chksum = (self.chksum - t) & 0xFFFF
                            if s["sensorLength"] == 4:
                                t = (s["sensorValue"] >> 16) & 0xFF
                                response.append(t)
                                self.chksum = (self.chksum - t) & 0xFFFF
                                t = (s["sensorValue"] >> 24) & 0xFF
                                response.append(t)
                                self.chksum = (self.chksum - t) & 0xFFFF
                            response += self.chksum.to_bytes(2, byteorder="little")
                            self.stream.write(response)
                        else:
                            adr = 0
                    self.state = self.STATE_DISCARD
            elif self.state == self.STATE_DISCARD:
                pass

    def readChannel(self, channelNr):
        if 0 <= channelNr < self.PROTOCOL_CHANNELS:
            return self.channel[channelNr]
        return 0

    def addSensor(self, sensorType, sensorLength=2):
        if sensorLength not in (2, 4):
            sensorLength = 2
        if self.NumberSensors < self.SENSORMAX:
            sensor = {
                "sensorType": sensorType,
                "sensorLength": sensorLength,
                "sensorValue": 0,
            }
            self.sensors.append(sensor)
            self.NumberSensors += 1
        return self.NumberSensors

    def setSensorMeasurement(self, adr, value):
        if 1 <= adr <= self.NumberSensors:
            self.sensors[adr - 1]["sensorValue"] = value

    def run_ibus(self):
        if self.ibus_port is not None:
            try:
                self.stream = serial.Serial(
                    self.ibus_port, baudrate=self.ibus_baudrate, timeout=0.001
                )
                print(f"connected to {self.ibus_port}")
            except Exception as e:
                print(f"[IBUS] Error: {e}")
                self.stream = None
    
    def reset_ibus_port(self):
        try:
            subprocess.run(["stty", "-F", self.ibus_port, "sane"], check=True)
            print(f"Successfully reset {self.ibus_port}")
        except subprocess.CalledProcessError as e:
            print(f"Error resetting {self.ibus_port}: {e}")

    def ibus_destroy(self):
        if self.stream is not None:
            self.stream.reset_input_buffer()
            self.stream.close()
        self.stream = None


# from PySide6.QtCore import QObject, Signal, Slot, QTimer
# from PySide6.QtSerialPort import QSerialPort
# from PySide6.QtWidgets import QApplication
# from SerialDeviceScanner import DevicePortScanner
# import signal
# import subprocess
# import time
# import os
# import sys
# os.environ["QT_QPA_PLATFORM"] = "offscreen"

# class IBusBM(QObject):
#     # Protocol constants
#     PROTOCOL_LENGTH = 0x20
#     PROTOCOL_OVERHEAD = 3
#     PROTOCOL_TIMEGAP = 3  # ms
#     PROTOCOL_CHANNELS = 14
#     PROTOCOL_COMMAND40 = 0x40
#     PROTOCOL_COMMAND_DISCOVER = 0x80
#     PROTOCOL_COMMAND_TYPE = 0x90
#     PROTOCOL_COMMAND_VALUE = 0xA0
#     SENSORMAX = 10

#     STATE_GET_LENGTH = 0
#     STATE_GET_DATA = 1
#     STATE_GET_CHKSUML = 2
#     STATE_GET_CHKSUMH = 3
#     STATE_DISCARD = 4

#     # Channel signals
#     ch0_signal = Signal(int)
#     ch1_signal = Signal(int)
#     ch2_signal = Signal(int)
#     ch3_signal = Signal(int)
#     vra_signal = Signal(int)
#     vrb_signal = Signal(int)
#     swa_signal = Signal(int)
#     swb_signal = Signal(int)
#     swc_signal = Signal(int)
#     swd_signal = Signal(int)

#     IBusBMfirst = None

#     def __init__(self, port: str, baudrate: int = 115200, parent=None):
#         super().__init__(parent)
#         self.port_name = port
#         self.baudrate = baudrate
#         self.serial = QSerialPort(self)
#         self.serial.setPortName(self.port_name)
#         self.serial.setBaudRate(self.baudrate)
#         self.serial.readyRead.connect(self._on_ready_read)

#         # Parsing state
#         self.state = self.STATE_DISCARD
#         self.last = self._current_millis()
#         self.ptr = 0
#         self.len = 0
#         self.chksum = 0
#         self.lchksum = 0
#         self.buffer = bytearray(self.PROTOCOL_LENGTH)
#         self.channel = [0] * self.PROTOCOL_CHANNELS

#         # Sensor emulation
#         self.sensors = []
#         self.NumberSensors = 0

#         # Linked list of instances
#         self._next = None
#         if IBusBM.IBusBMfirst is None:
#             IBusBM.IBusBMfirst = self
#         else:
#             current = IBusBM.IBusBMfirst
#             while current._next:
#                 current = current._next
#             current._next = self

#     @Slot()
#     def start(self):
#         if not self.serial.open(QSerialPort.ReadWrite):
#             print(f"[IBUS] Error opening {self.port_name}: {self.serial.errorString()}")
#         else:
#             print(f"[IBUS] Connected to {self.port_name}")

#     @Slot()
#     def stop(self):
#         if self.serial.isOpen():
#             self.serial.close()
#             print(f"[IBUS] Disconnected from {self.port_name}")

#     @Slot()
#     def reset_port(self):
#         try:
#             subprocess.run(["stty", "-F", self.port_name, "sane"], check=True)
#             print(f"[IBUS] Successfully reset {self.port_name}")
#         except subprocess.CalledProcessError as e:
#             print(f"[IBUS] Error resetting {self.port_name}: {e}")

#     def addSensor(self, sensorType: int, sensorLength: int = 2) -> int:
#         if sensorLength not in (2, 4):
#             sensorLength = 2
#         if self.NumberSensors < self.SENSORMAX:
#             self.sensors.append({
#                 "sensorType": sensorType,
#                 "sensorLength": sensorLength,
#                 "sensorValue": 0
#             })
#             self.NumberSensors += 1
#         return self.NumberSensors

#     def setSensorMeasurement(self, adr: int, value: int):
#         if 1 <= adr <= self.NumberSensors:
#             self.sensors[adr - 1]["sensorValue"] = value

#     def readChannel(self, channelNr: int) -> int:
#         if 0 <= channelNr < self.PROTOCOL_CHANNELS:
#             return self.channel[channelNr]
#         return 0

#     def _on_ready_read(self):
#         data = self.serial.readAll()
#         # Convert QByteArray to bytes
#         chunk = bytes(data)
#         for v in chunk:
#             self._process_byte(v)
#         # propagate to next instance
#         if self._next:
#             self._next._on_ready_read()

#     def _process_byte(self, v: int):
#         now = self._current_millis()
#         if now - self.last >= self.PROTOCOL_TIMEGAP:
#             self.state = self.STATE_GET_LENGTH
#         self.last = now

#         if self.state == self.STATE_GET_LENGTH:
#             if v <= self.PROTOCOL_LENGTH and v > self.PROTOCOL_OVERHEAD:
#                 self.ptr = 0
#                 self.len = v - self.PROTOCOL_OVERHEAD
#                 self.chksum = (0xFFFF - v) & 0xFFFF
#                 self.state = self.STATE_GET_DATA
#             else:
#                 self.state = self.STATE_DISCARD

#         elif self.state == self.STATE_GET_DATA:
#             self.buffer[self.ptr] = v
#             self.ptr += 1
#             self.chksum = (self.chksum - v) & 0xFFFF
#             if self.ptr == self.len:
#                 self.state = self.STATE_GET_CHKSUML

#         elif self.state == self.STATE_GET_CHKSUML:
#             self.lchksum = v
#             self.state = self.STATE_GET_CHKSUMH

#         elif self.state == self.STATE_GET_CHKSUMH:
#             received = ((v << 8) | self.lchksum) & 0xFFFF
#             if self.chksum == received:
#                 self._handle_frame()
#             self.state = self.STATE_DISCARD

#     def _handle_frame(self):
#         cmd = self.buffer[0]
#         adr = cmd & 0x0F
#         if cmd == self.PROTOCOL_COMMAND40:
#             # servo channels
#             for i in range(1, self.PROTOCOL_CHANNELS * 2 + 1, 2):
#                 idx = (i - 1) // 2
#                 if i + 1 < len(self.buffer):
#                     self.channel[idx] = self.buffer[i] | (self.buffer[i+1] << 8)
#             # emit signals
#             self.ch0_signal.emit(self.channel[0])
#             self.ch1_signal.emit(self.channel[1])
#             self.ch2_signal.emit(self.channel[2])
#             self.ch3_signal.emit(self.channel[3])
#             self.vra_signal.emit(self.channel[4])
#             self.vrb_signal.emit(self.channel[5])
#             self.swa_signal.emit(self.channel[6])
#             self.swb_signal.emit(self.channel[7])
#             self.swc_signal.emit(self.channel[8])
#             self.swd_signal.emit(self.channel[9])
#             print(f"IBUS: >> ch0: {self.channel[0]}")
#             print(f"IBUS: >> ch1: {self.channel[1]}")
#             print(f"IBUS: >> ch2: {self.channel[2]}")
#             print(f"IBUS: >> ch3: {self.channel[3]}")
#             print(f"IBUS: >> vra: {self.channel[4]}")
#             print(f"IBUS: >> vrb: {self.channel[5]}")
#             print(f"IBUS: >> swa: {self.channel[6]}")
#             print(f"IBUS: >> swb: {self.channel[7]}")
#             print(f"IBUS: >> swc: {self.channel[8]}")
#             print(f"IBUS: >> swd: {self.channel[9]}")
            

#         elif 0 < adr <= self.NumberSensors and self.len == 1:
#             # sensor emulation if needed
#             s = self.sensors[adr-1]
#             cmd_type = cmd & 0xF0
#             # responses omitted for brevity
#             pass

#     def _current_millis(self) -> int:
#         return int(time.monotonic() * 1000)

# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     scanner = DevicePortScanner()
#     ports =scanner.list_serial_ports()
#     port = scanner.find_fs_i6_port(ports=ports)
#     print(f"IBUS: >> port: {port}")
#     ibus = IBusBM(port)

#     ibus.ch0_signal.connect(lambda v: print(f"Channel 0: {v}"))
#     ibus.ch1_signal.connect(lambda v: print(f"Channel 1: {v}"))

#     ibus.start()

#     timer = QTimer()
#     timer.start(100)
#     timer.timeout.connect(lambda: None)

#     def handle_sigint(sig, frame):
#         ibus.stop()
#         app.quit()

#     signal.signal(signal.SIGINT, handle_sigint)

#     sys.exit(app.exec())
