import math
from .ScurveInterpolator import Scurve_Interpolator
from .serialDevice import Device
from PySide6.QtCore import QCoreApplication, Signal as pyqtSignal, QObject
import sys


class RobotInterface(Device, QObject):
    feedbackPositionSignal = pyqtSignal(float, float, float, float)
    estopSignal = pyqtSignal()

    def __init__(self, port="/dev/ttyACM1"):
        Device.__init__(self)  # Khởi tạo lớp Device
        QObject.__init__(self)  # Khởi tạo lớp QObject
        # super().__init__()

        self.comport = port
        self.confirm_msg = "YesDelta"
        self.request_msg = "IsDelta"

        self.X, self.Y, self.Z, self.W, self.F, self.A, self.S, self.E, self.J = 0, 0, 0, 0, 500, 8000, 30, 40, 155000
        self.old_X, self.old_Y, self.old_Z = self.X, self.Y, self.Z
        self.input_value = [0, 0, 0, 0, 0, 0]

        self.scurve_tool = Scurve_Interpolator()
        self.new_response = ""

        self.is_sync = True
        self.rover_vel = 100
        self.con_angle = float(0.0)

        self.is_feedback_position = False

    def response_handling(self, response):
        # print(response) Delta:EStop Pressing!
        self.new_response = str(response)

        if self.last_command.count('Position') > 0:
            self.__get_robot_position(response)
        elif self.new_response.startswith("I") or self.new_response.startswith("A"):
            self.__get_robot_input(response)
        elif self.new_response.startswith("Delta:EStop Pressing!"):
            self.estopSignal.emit()

        elif self.new_response.startswith("Delta:Robot is stopped or paused!"):
            self.estopSignal.emit()

    def open(self):
        return super().connect()

    def disconnect(self):
        return super().disconnect()

    def reconnect(self, _new_port):
        super().disconnect()
        self.comport = _new_port
        return super().connect_only()

    def robot_resume(self, is_wait=True):
        self.send("Emergency:Resume")
        if is_wait:
            self.wait_response()

    def move_relative(self, X=None, Y=None, Z=None, W=None, F=None, A=None, S=None, E=None, J=None):
        self.old_X, self.old_Y, self.old_Z = self.X, self.Y, self.Z
        new_gcode = "G01"

        if X != None:
            self.X = self.X + X
            new_gcode = new_gcode + ' X' + str(self.X)
        if Y != None:
            self.Y = self.Y + Y
            new_gcode = new_gcode + ' Y' + str(self.Y)
        if Z != None:
            self.Z = self.Z + Z
            new_gcode = new_gcode + ' Z' + str(self.Z)
        if W != None:
            self.W = self.W + W
            new_gcode = new_gcode + ' W' + str(self.W)
        if F != None:
            self.F = F
            self.scurve_tool.max_vel = F
            new_gcode = new_gcode + ' F' + str(F)
        if A != None:
            self.A = A
            self.scurve_tool.max_acc = A
            new_gcode = new_gcode + ' A' + str(A)
        if S != None:
            self.S = S
            self.scurve_tool.vel_start = S
            new_gcode = new_gcode + ' S' + str(E)
        if E != None:
            self.E = E
            self.scurve_tool.vel_end = E
            new_gcode = new_gcode + ' E' + str(E)
        if J != None:
            self.J = J
            self.scurve_tool.max_jer = J
            new_gcode = new_gcode + ' J' + str(J)

        if self.is_feedback_position:
            self.feedbackPositionSignal.emit(self.X, self.Y, self.Z, self.W)

        self.send(new_gcode)
        self.wait_response()

    def move(self, X=None, Y=None, Z=None, W=None, F=None, A=None, S=None, E=None, J=None, sync=False, time_offset=0):
        self.old_X, self.old_Y, self.old_Z = self.X, self.Y, self.Z
        new_gcode = "G01"

        if X != None:
            self.X = X
            new_gcode = new_gcode + ' X' + str(X)
        if Y != None:
            self.Y = Y
            new_gcode = new_gcode + ' Y' + str(Y)
        if Z != None:
            self.Z = Z
            new_gcode = new_gcode + ' Z' + str(Z)
        if W != None:
            self.W = W
            new_gcode = new_gcode + ' W' + str(W)
        if F != None:
            self.F = F
            self.scurve_tool.max_vel = F
            new_gcode = new_gcode + ' F' + str(F)
        if A != None:
            self.A = A
            self.scurve_tool.max_acc = A
            new_gcode = new_gcode + ' A' + str(A)
        if S != None:
            self.S = S
            self.scurve_tool.vel_start = S
            new_gcode = new_gcode + ' S' + str(E)
        if E != None:
            self.E = E
            self.scurve_tool.vel_end = E
            new_gcode = new_gcode + ' E' + str(E)
        if J != None:
            self.J = J
            self.scurve_tool.max_jer = J
            new_gcode = new_gcode + ' J' + str(J)

        if sync == False:
            if self.is_feedback_position:
                self.feedbackPositionSignal.emit(
                    self.X, self.Y, self.Z, self.W)
            print(new_gcode)
            self.send(new_gcode)

            self.wait_response()
        else:
            new_x, new_y = self.scurve_tool.find_sync_point(
                self.old_X, self.old_Y, self.old_Z, self.X, self.Y, self.Z, self.rover_vel, self.con_angle, time_offset)
            new_x = round(float(new_x), 2)
            new_y = round(float(new_y), 2)
            # print (f'new x {new_x}')
            # print (f'new y {new_y}')

            self.X = new_x
            self.Y = new_y

            new_gcode = 'G01 X{0} Y{1} Z{2} W{3}'.format(
                new_x, new_y, self.Z, self.W)
            if F != None:
                new_gcode = new_gcode + ' F' + str(F)
            if A != None:
                new_gcode = new_gcode + ' A' + str(A)
            if S != None:
                new_gcode = new_gcode + ' S' + str(E)
            if E != None:
                new_gcode = new_gcode + ' E' + str(E)
            if J != None:
                new_gcode = new_gcode + ' J' + str(J)

            if self.is_feedback_position:
                self.feedbackPositionSignal.emit(
                    self.X, self.Y, self.Z, self.W)

            self.send(new_gcode)
            self.wait_response()

    def sleep(self, time_ms=1000, sync=False, is_wait=True):
        if sync == False:
            self.send('G04 P{}'.format(time_ms))
            if is_wait:
                self.wait_response()
        else:
            distance = self.rover_vel * (float(time_ms) / 1000)
            # print('sleep distance: ' + str(distance))
            new_x = self.X + distance * math.cos(math.radians(self.con_angle))
            new_y = self.Y + distance * math.sin(math.radians(self.con_angle))
            old_F = self.F
            self.move(X=round(float(new_x), 2), Y=round(
                float(new_y), 2), F=abs(self.rover_vel), sync=False)
            self.F = old_F
            self.scurve_tool.max_vel = self.F

            self.X = new_x
            self.Y = new_y

    def set_z_safe(self, z_safe, is_wait=True):
        self.send("M207 Z{0}".format(z_safe))
        if is_wait:
            self.wait_response()

    def set_output(self, pin, state, is_wait=True):
        gcode = ""
        if state != 0:
            gcode = 'M03 D{0}'.format(pin)
        else:
            gcode = 'M05 D{0}'.format(pin)

        self.send(gcode)
        if is_wait:
            self.wait_response()

    def go_home(self, is_wait=True):
        self.send("G28")
        if is_wait:
            self.wait_response()

        self.get_position(is_wait=is_wait)

    def set_absolute(self, is_wait=True):
        self.send("G90")
        if is_wait:
            self.wait_response()

    def set_relative(self, is_wait=True):
        self.send("G91")
        if is_wait:
            self.wait_response()

    def turn_off_axis4(self, is_wait=True):
        self.send("M60 D0")
        if is_wait:
            self.wait_response()

    def turn_on_axis4(self, is_wait=True):
        self.send("M60 D1")
        if is_wait:
            self.wait_response()

    def read_input(self, pin, is_wait=True):
        gcode = ""
        if pin > 3:
            gcode = 'M07 A{0}'.format(pin - 4)
        else:
            gcode = 'M07 I{0}'.format(pin)

        self.send(gcode)
        if is_wait:
            self.wait_response()

    def get_position(self, is_wait=True):
        self.send("Position")
        if is_wait:
            self.wait_response()

    def get_robot_model(self, is_wait=True):
        self.send("ROBOTMODEL")
        if is_wait:
            self.wait_response()

    def set_sync_vel(self, rover_vel=100):
        self.rover_vel = rover_vel

    def set_sync_angle(self, con_angle=0.0):
        self.con_angle = float(con_angle)

    def __get_robot_position(self, response):
        if response.count(',') > 2:
            paras = response.split(',')
            for i in range(len(paras)):
                if i == 0:
                    self.X = float(paras[i])
                if i == 1:
                    self.Y = float(paras[i])
                if i == 2:
                    self.Z = float(paras[i])
                if i == 3:
                    self.W = float(paras[i])
                if i == 4:
                    self.U = float(paras[i])
                if i == 5:
                    self.V = float(paras[i])
        self.feedbackPositionSignal.emit(self.X, self.Y, self.Z, self.W)
    # ex response: "I1 V0" or "A1 V2400"
    # nếu
    def __get_robot_input(self, response):
        _key_value = response.split(' ')
        _index = int(_key_value[0][1])
        _value = int(_key_value[1][1:])

        if _key_value[0][0] == 'A':
            _index = _index + 4

        self.input_value[_index] = _value


if __name__ == "__main__":
    app = QCoreApplication(sys.argv)
    print("__test__")
    test_device = RobotInterface("COM16")
    test_device.open()
    sys.exit(app.exec_())
