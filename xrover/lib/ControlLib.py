from PySide6.QtCore import QObject, Signal as pyqtSignal, QTimer


class ControlLib(QObject):
    def __init__(self):
        super().__init__()
        self.z_min = -2
        self.z_max = 2  # rad/s
        self.x_min = 0
        self.x_max = 1  # m/s
        self.joystick_current_x = 0
        self.joystick_current_z = 0

    def rover_x(self, value):
        deadzone_min = 1495
        deadzone_max = 1505
        center = 1500
        new_x_value = 0
        if abs(value - self.joystick_current_x) > 1:
            self.joystick_current_x = value
            if deadzone_min <= value <= deadzone_max:
                new_x_value = 0
            else:
                new_x_value = self.map_joystick_to_range(value, self.x_min, self.x_max)
        return new_x_value

    def rover_z(self, value):
        deadzone_min = 1495
        deadzone_max = 1505
        center = 1500
        new_z_value = 0
        if abs(value - self.joystick_current_z) > 1:
            self.joystick_current_z = value
            if deadzone_min <= value <= deadzone_max:
                new_z_value = 0
            else:
                new_z_value = self.map_joystick_to_range(value, self.z_min, self.z_max)
        return new_z_value

    def map_joystick_to_range(
        self, joy_value, out_min, out_max, joy_min=1000, joy_max=2000
    ):
        value = out_min + ((joy_value - joy_min) / (joy_max - joy_min)) * (
            out_max - out_min
        )
        return round(value, 2)
