from lib.ConstVariable import ULTRASONIC
from PySide6.QtCore import QObject, Signal
import serial
import time

class UltrasonicSensors(QObject):
    a_signal = Signal(int)
    b_signal = Signal(int)
    c_signal = Signal(int)
    d_signal = Signal(int)
    e_signal = Signal(int)
    f_signal = Signal(int)
    
    def __init__(self,port = None):
        super().__init__()
        self.s21c_baudrate = None
        self.s21c_min_range = None
        self.s21c_max_range = None
        self.load_setting()
        self.s21c_port = port
        self.s21c_field_of_view = 1.047
        self.timeout = 1
        self.sensor_data_index = 0
        try:
            self.serial_port = serial.Serial(
                port=self.s21c_port, baudrate=self.s21c_baudrate, timeout=self.timeout
            )
            print(f"UltrasonicSensors: >> Connected to ultrasonic sensors: {self.s21c_port}")
        except serial.SerialException as e:
            print(f"UltrasonicSensors: >> Error opening ultrasonic sensors: {e}")
            raise e

    def load_setting(self):
        self.s21c_baudrate = ULTRASONIC.baudrate
        self.s21c_min_range = ULTRASONIC.min_range
        self.s21c_max_range = ULTRASONIC.max_range

    def read_uart_data(self):
        try:
            while True:
                if self.serial_port.in_waiting > 0:
                    try:
                        data = self.serial_port.read_all().decode("utf-8",errors="ignore").strip()
                        sensor_data = self.parse_sensor_data(data)
                        self.publish_sensor_data(sensor_data)
                        # print(f"S21C: >> {str(self.parse_sensor_data(data))}")
                    except Exception as e:
                        print(f"S21C: >> Error reading from UART: {e}")
                time.sleep(0.1)
        except Exception as e:
            print(f"S21C: >> Error reading from UART: {e}")

    def parse_sensor_data(self, data:str):
        lines = data.splitlines()

        parsed_data = {}
        for line in lines:
            parts = line.split()
            for part in parts:
                if ":" in part and part[0] in "ABCDEF":
                    key, value = part.split(":")
                    parsed_data[key] = int(value)

        return parsed_data

    def publish_sensor_data(self, data):
        if "A" in data:
            self.a_signal.emit(data["A"])
        if "B" in data:
            self.b_signal.emit(data["B"])
        if "C" in data:
            self.c_signal.emit(data["C"])
        if "D" in data:
            self.d_signal.emit(data["D"])
        if "E" in data:
            self.e_signal.emit(data["E"])
        if "F" in data:
            self.f_signal.emit(data["F"])
        
    def clean_up(self):
        self.serial_port.close()
        print(f"UltrasonicSensors: >> Closed ultrasonic sensors: {self.s21c_port}")

