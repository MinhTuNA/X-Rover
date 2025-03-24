import sys
import glob
import serial
import cv2
import time
from PySide6.QtCore import QObject
import serial.tools
import serial.tools.list_ports
from .ConstVariable import *
import os
import sys


class DevicePortScanner(QObject):

    def __init__(self):
        super().__init__()
        self.hand_camera_order = None
        self.calib_camera_order = None

        self.rs485_serial_number = RS485.serial_number
        self.um982_serial_number = GPS.serial_number
        self.imu_serial_number = IMU.serial_number
        self.ultrasonic_serial_number = ULTRASONIC.serial_number
        self.fs_i6_serial_number = FS_I6.serial_number
        

    def get_ports(self):
        list_ports = self.list_serial_ports()
        return list_ports

    def refresh(self):
        self.ports = self.list_serial_ports()

        # kiểm tra xem có cổng nào là của DeltaX không
        delta_x_port = self.find_delta_x_port()
        if delta_x_port is not None:
            pass

        # kiểm tra xem có cổng nào là của Encoder X không
        encoder_x_port = self.find_encoder_x_port()

        # kiểm tra xem có cổng nào là của camera không
        first_camera_port = self.find_first_camera_port()
        if first_camera_port is not None:
            pass

        second_camera_port = self.find_second_camera_port()
        if second_camera_port is not None:
            pass

    def refresh_camera_ports(self):
        self.thread().msleep(100)

        self.camera_ports = self.list_video_ports()
        print("Camera ports: ", self.camera_ports)

        # kiểm tra xem có cổng nào là của camera không
        first_camera_port = self.find_first_camera_port()
        if first_camera_port is not None:
            pass

        second_camera_port = self.find_second_camera_port()
        if second_camera_port is not None:
            pass

    def list_serial_ports(self):

        ports = serial.tools.list_ports.comports()
        # result = []
        # try:
        #     s = serial.Serial(port)
        #     s.close()
        #     result.append(port)
        # except (OSError, serial.SerialException):
        #     pass
        return ports

    def check_camera(self, device_id):
        cap = cv2.VideoCapture(device_id)
        if not cap.isOpened():
            return False
        ret, frame = cap.read()
        cap.release()
        return ret

    def list_video_ports(self):
        if sys.platform.startswith("win"):
            # Không áp dụng cho Windows
            raise EnvironmentError("This function is not applicable for Windows.")
        elif sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
            # Tìm tất cả các cổng video trong /dev/
            ports = glob.glob("/dev/video*")
        elif sys.platform.startswith("darwin"):
            # Tìm các cổng video trên macOS, thường là trong /dev/
            ports = glob.glob("/dev/video*")
        else:
            raise EnvironmentError("Unsupported platform")

        _new_ports = []

        for port in ports:
            if self.check_camera(port):
                _new_ports.append(port)
        # print(_new_ports)
        return _new_ports

    def find_delta_x_port(self, ports):
        if len(ports) == 0:
            return None
        for port in ports:
            if port.serial_number == DELTA.serial_number:
                return port.device
        return None

    def find_encoder_x_port(self, ports):
        if len(ports) == 0:
            return None
        for port in ports:
            try:
                ser = serial.Serial(port, baudrate=115200, timeout=1)
                ser.write(b"IsXConveyor\n")
                response = ser.readline().decode("utf-8").strip()
                ser.close()
                if response == "YesXConveyor":
                    return port
            except (OSError, serial.SerialException):
                pass
        return None

    

    def find_rs485_port(self, ports):
        if len(ports) == 0:
            return None
        for port in ports:
            if port.serial_number == self.rs485_serial_number:
                return port.device
        return None

    def find_imu_port(self, ports):
        if len(ports) == 0:
            return None
        for port in ports:
            if port.serial_number == self.imu_serial_number:
                return port.device
        return None

    def find_um982_port(self, ports):
        if len(ports) == 0:
            return None
        for port in ports:
            if port.serial_number == self.um982_serial_number:
                return str(port.device)
        return None

    def find_s21c_port(self, ports):
        if len(ports) == 0:
            return None
        for port in ports:
            if port.serial_number == self.ultrasonic_serial_number:
                return port.device
        return None

    def find_fs_i6_port(self, ports):
        if len(ports) == 0:
            return None
        for port in ports:
            if port.serial_number == self.fs_i6_serial_number:
                return port.device
        return None


if __name__ == "__main__":
    scanner = DevicePortScanner()
    ports = scanner.list_serial_ports()
    # print(f"rs485 >> {scanner.find_rs485_port()}")
    # print(f"imu >> {scanner.find_imu_port()}")
    # print(f"um982 >> {scanner.find_um982_port()}")
    # print(f"s21c >> {scanner.find_s21c_port()}")
    # print(f"fs_i6 >> {scanner.find_fs_i6_port()}")
    # print("xencoder: ", scanner.find_encoder_x_port())
    print("deltax", scanner.find_delta_x_port(ports))
