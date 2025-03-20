import time
import serial
from PySide6.QtCore import QObject, Signal as pyqtSignal, Slot
from PySide6.QtSerialPort import QSerialPort
import atexit
import threading


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

    # Các trạng thái của state machine
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
                    self.ibus_port, baudrate=self.ibus_baudrate, timeout=1
                )
                print(f"connected to {self.ibus_port}")
            except Exception as e:
                print(f"[IBUS] Error: {e}")
                self.stream = None

        # while True:
        #     self.loop()
        #     self.handleEmitChanel()
        #     time.sleep(0.001)

    def handleEmitChanel(self):
        pass

    def ibus_destroy(self):
        self.stream.reset_input_buffer()
        self.stream.close()
        self.stream = None


# Ví dụ sử dụng
if __name__ == "__main__":
    ibus = IBusBM()
    ibus.run_ibus()
