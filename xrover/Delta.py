import sys
import json
from enum import Enum
from PySide6.QtCore import QCoreApplication, QObject, Signal
from lib.RobotInterface import RobotInterface
from lib.ControlLib import ControlLib

class ControlMode(Enum):
    ROVER = 1000
    DELTA = 2000


class SwaMode(Enum):
    SWB_ON = 1000
    SWB_OFF = 2000


class Delta(QObject):
    position_signal = Signal(float,float,float,float)
    isconnected_signal = Signal(bool)
    def __init__(self,port=None):
        super().__init__()
        self.z_safe = None
        self.z_min = -621.2
        self.z_max = -831.2
        self.is_first_connect = True
        self.control_lib = ControlLib()
        self.delta_port = port
        self.delta = None
        self.current_x = 0
        self.current_y = 0
        self.current_z = 0


    def start_delta(self):
        self.delta = RobotInterface(port=self.delta_port)
        self.delta.open()
        if self.delta.is_connected():
            if self.is_first_connect:
                self.is_first_connect = False
                self.delta.robot_resume()
                print("Delta: >> delta is connected")
        self.delta.go_home()
       
        self.delta.feedbackPositionSignal.connect(self.feedback_position)
        self.delta.get_position()

    def feedback_position(self, x, y, z, w):
        self.position_signal.emit(x,y,z,w)
        # print(f"Delta: >> feedback position {x}, {y}, {z}, {w}")
       
    def move_to(self, x:float, y:float, z:float):
        print(f"Delta: >> move to {x}, {y}, {z}")
        self.move_delta(x=x, y=y, z=z)

    def go_home(self):
        self.delta.go_home()
        print(f"Delta: >> go home")

    def move_delta(self, x, y, z, is_relative=False):
        if not is_relative:
            self.delta.move(X=x, Y=y, Z=z)
        else:
            self.delta.move_relative(X=x, Y=y, Z=z)

    def status_callback(self):
        if self.delta.is_connected():
            self.isconnected_signal.emit(True)
            self.position_signal.emit(self.delta.X, self.delta.Y, self.delta.Z, self.delta.W)
        else:
            self.isconnected_signal.emit(False)

    def handle_destroy(self):
        self.delta.disconnect()
        print("Delta: >> Delta closed")
        
        


