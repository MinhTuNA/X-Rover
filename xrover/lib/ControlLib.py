from PySide6.QtCore import QObject, Signal as pyqtSignal, QTimer


class ControlLib(QObject):
    def __init__(self):
        super().__init__()
        self.z_min = -0.5
        self.z_max = 0.5  # rad/s
        self.x_min = -0.5
        self.x_max = 0.5  # m/s
        self.joystick_current_x = 0
        self.joystick_current_z = 0
        self.control_min_value = 1000
        self.control_max_value = 2000
        self.vra_min = 0.005
        self.vra_max = 1
        self.vrb_min = 0.001
        self.vrb_max = 1

    def rover_x(self, value):

        deadzone_min = 1500
        deadzone_max = 1570
        center = 1515
        new_x_value = 0
        if deadzone_min <= value <= deadzone_max or value < self.control_min_value or value > self.control_max_value:
            new_x_value = 0
        else:
            new_x_value = self.map_joystick_to_range(value, self.x_min, self.x_max)
        return new_x_value

    def rover_z(self, value):
        deadzone_min = 1400
        deadzone_max = 1600
        center = 1500
        new_z_value = 0
        if deadzone_min <= value <= deadzone_max or value < self.control_min_value or value > self.control_max_value:
            new_z_value = 0
        else:
            new_z_value = self.map_joystick_to_range(value, self.z_min, self.z_max)
        return new_z_value
    
    def vra(self,value):
        if value < self.control_min_value or value > self.control_max_value:
            return 0
        return self.map_vr_to_range(value, self.vra_min, self.vra_max)

    def vrb(self,value):
        if value < self.control_min_value or value > self.control_max_value:
            return 0
        return self.map_vr_to_range(value, self.vrb_min, self.vrb_max)

    def map_joystick_to_range(
        self, joy_value, out_min, out_max, joy_min=1000, joy_max=2000
    ):
        value = out_min + ((joy_value - joy_min) / (joy_max - joy_min)) * (
            out_max - out_min
        )
        return round(value, 2)

    def map_vr_to_range(self, value, out_min, out_max, joy_min=1000, joy_max=2000):
        value = out_min + ((value - joy_min) / (joy_max - joy_min)) * (
            out_max - out_min
        )
        return round(value, 6)

    def delta_x(self, value):
        deadzone_min = 1450
        deadzone_max = 1550
        center = 1515
        new_x_value = 0
        if deadzone_min <= value <= deadzone_max:
            new_x_value = 0
        elif value > deadzone_max:
            new_x_value = 1
        elif value < deadzone_min:
            new_x_value = -1
        return new_x_value

    def delta_y(self, value):
        deadzone_min = 1450
        deadzone_max = 1550
        center = 1515
        new_x_value = 0
        if deadzone_min <= value <= deadzone_max:
            new_x_value = 0
        elif value > deadzone_max:
            new_x_value = 1
        elif value < deadzone_min:
            new_x_value = -1
        return new_x_value
