import time
import sys
from PySide6 import QtCore
from PySide6.QtCore import QEventLoop
from PySide6.QtSerialPort import QSerialPort

class Device():
    def __init__(self, port = "/dev/ttyUSB1", baudrate=115200, confirm_msg = "YesDelta", request_msg = "IsDelta"):
        self.comport = port
        self.baudrate = baudrate
        self.serial_m = QSerialPort()
        self._is_connected = False
        self._is_responded = True
        self.latest_response = ""
        self.robot_model = ""
        self.confirm_msg = confirm_msg
        self.request_msg = request_msg
        self.timeout = 4.0
        self.last_command = ""

        self.event_loop = QEventLoop()
        
    def connect(self):
        #print("connecting...\n")
        self.serial_m.setPortName(self.comport)
        self.serial_m.setBaudRate(self.baudrate)
        
        if not self.serial_m.open(QSerialPort.OpenModeFlag.ReadWrite):
            print("-DeltaX connect false\n")
            return False
        else:
            self.serial_m.readyRead.connect(self.serial_read_event)
            print("- Deltax connected\n")
            if self.serial_m.isOpen():
                self.send(self.request_msg)
                self.wait_response()

                if self._is_connected == True:
                    self.send("ROBOTMODEL")
                    self.wait_response()
        
            # print(self._is_connected)
            return self._is_connected
    
    def connect_only(self):
        self.serial_m.setPortName(self.comport)
        if not self.serial_m.open(QSerialPort.OpenModeFlag.ReadWrite):
            print("-DeltaX connect false\n")
            return False
        else:
            print("- Deltax connected\n")
            return True
    
    def disconnect(self):
        #if self.serial_m.open(QSerialPort.OpenModeFlag.ReadWrite):
        self.serial_m.close()

    def is_connected(self):
        return self._is_connected
    
    def is_responded(self):
        return self._is_responded

    def serial_read_event(self):
        if self.serial_m.canReadLine() == True:
            self.latest_response = self.serial_m.readLine().data().decode()

            self.latest_response = self.latest_response.replace('\n', '')
            self.latest_response = self.latest_response.replace('\r', '')
            
            if self.latest_response.startswith(self.confirm_msg):
                self._is_connected = True
                self._is_responded = True
                return
            
            if self.latest_response.startswith("MODEL:"):
                self.robot_model = self.latest_response.replace("MODEL:", "")
                self._is_responded = True
                return

            self.response_handling(self.latest_response)
            self._is_responded = True


    def response_handling(self, response):
        pass
                    
    def send(self, data):
        if self.serial_m.isOpen() == False:
            print('serial is not open')
            return
        
        self.last_command = data

        if not data.endswith('\n'):
            data = data + '\n'
            
        self._is_responded = False
        # print(f'{data.encode()}')
        self.serial_m.write(data.encode())

    def wait_response(self):
        start = time.time()
        while time.time() - start < self.timeout:
            self.event_loop.processEvents()
            if self._is_responded == True:
                return True
        
        return False


if __name__ == "__main__":
    app = QtCore.QCoreApplication(sys.argv)
    print("__test__")
    test_device = Device()
    test_device.connect()
    sys.exit(app.exec_())    