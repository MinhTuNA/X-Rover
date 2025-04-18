from lib.ConstVariable import WHEEL,RS485
from lib.ModbusDevice import Driver
from lib.SerialDeviceScanner import DevicePortScanner
import time
class testWheel():
    def __init__(self):
        print("init")
        scanner = DevicePortScanner()
        ports = scanner.get_ports()
        self.rs485_port = scanner.find_rs485_port(ports)
        self.baud_rate = RS485.baudrate
        self.driver = Driver(self.rs485_port)
        print(f"RS485 port: {self.rs485_port}")
            


if __name__ == "__main__":
    wheel = testWheel()
    rpm = 60
    while True:
        wheel.driver.set_motor(left_rpm=rpm*5,right_rpm=-rpm*5)
        time.sleep(0.5)