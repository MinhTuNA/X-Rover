#!/usr/bin/env python3
from lib.ConstVariable import COMMON
from lib.NavigationController import NavigationController
from VariableManager import instance
from lib.ControlLib import ControlLib
from PySide6.QtCore import QObject, Signal, QTimer
from enum import Enum

class Control(Enum):
    FS_I6 = 0
    FREE = 1

class Mode(Enum):
    GPS = 0
    CAMERA = 1
    VELOCITY = 2

class Navigation(QObject):
    is_rover_connected_signal = Signal(bool)
    rover_mode_signal = Signal(int)
    method_control_signal = Signal(int)
    def __init__(self,port = None):
        super().__init__()

        #-----------gps-----------
        self.start_lat = None
        self.start_lon = None
        self.goal_lat = None
        self.goal_lon = None
        self.current_lat = None
        self.current_lon = None
        self.current_alt = None
        self.current_heading = None

        #-----------control-----------
        self.mode = None
        self.control = Control.FREE
        self.is_running = False
        self.is_segment_done = False
        self.control_x = 0
        self.control_z = 0
        self.left_rpm = 0
        self.right_rpm = 0
        self.current_swa = None
        self.linear_vel_x = COMMON.rover_speed
        self.control_lib = ControlLib()
        self.navigation_controller = NavigationController(port)
        instance.load(COMMON.variable_path)
        self.load_variable()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.navigate)

    def run(self):
        self.timer.start(50)
    
    def load_variable(self):
        mode = instance.get("mode")
        self.mode = Mode(int(mode))
        print(f"Navigation: >> mode: {self.mode}")

    def segment_callback(self, segment: dict):
        self.start_lat = segment["start_lat"]
        self.start_lon = segment["start_lon"]
        self.goal_lat = segment["end_lat"]
        self.goal_lon = segment["end_lon"]
    
    def target_point_callback(self, latitude:str, longitude:str):
        self.goal_lat = float(latitude)
        self.goal_lon = float(longitude)
        # print(f"target point: {latitude}, {longitude}")

    def gps_callback(self, latitude:str, longitude:str, altitude:str):
        self.current_lat = float(latitude)
        self.current_lon = float(longitude)
        self.current_alt = float(altitude)
        # print(f"gps: {latitude}, {longitude}, {altitude}")

    def heading_callback(self, heading: float):
        self.current_heading = heading
        # print(f"heading: {heading}")
        
    def start_navigation(self):
        self.is_running = True
        self.is_segment_done = False

    def stop_navigation(self):
        self.is_running = False

    def set_mode(self, mode: int):
        self.mode = Mode(mode)
        instance.set("mode", mode)
        instance.save()
        print(f"Navigation: >> set mode: {self.mode}")
    
    def rover_status(self):
        self.rover_mode_signal.emit(self.mode.value)
        self.is_rover_connected_signal.emit(True)

    def set_twist(self, linear_x, angular_z):
        if linear_x is None or angular_z is None:
            print("Navigation [ERROR]: >> linear_x or angular_z is None")
            return
        self.navigation_controller.set_velocity(linear_velocity=linear_x, angular_velocity=angular_z)
    
    def set_rpm(self, left_rpm:int, right_rpm:int):
        if self.control != Control.FREE:
            return
        if self.mode != Mode.CAMERA:
            return
        self.navigation_controller.set_rpm(left_rpm=left_rpm, right_rpm=right_rpm)
        print(f"Navigation: >> left_rpm: {left_rpm} right_rpm: {right_rpm}")
    
    def set_velocity(self, linear_x=None, angular_z=None):
        if self.control != Control.FREE:
            return
        if self.mode != Mode.VELOCITY:
            return
        if linear_x is None or angular_z is None:
            print("Navigation [ERROR]: >> linear_x or angular_z is None")
            return
        self.control_x = linear_x
        self.control_z = angular_z

    def control_x_callback(self, value: int):
        if self.control != Control.FS_I6:
            return
        new_x = self.control_lib.rover_x(value)
        if new_x > 0.5:
            print("Navigation [ERROR]: >> control_x is too large")
            return
        self.control_x = new_x
        # print(f"Navigation: >> control_x: {self.control_x}")

    def control_z_callback(self, value: int):
        if self.control != Control.FS_I6:
            return
        new_z = self.control_lib.rover_z(value)
        if new_z > 0.5:
            print("Navigation [ERROR]: >> control_z is too large")
            return
        self.control_z = new_z
        # print(f"Navigation: >> control_z: {self.control_z}")
    # def control_mode_callback(self, msg):
    #     if self.control == Control.FREE:
    #         return
    
    def swa_callback(self, value: int):
        if self.current_swa == value:
            return
        self.current_swa = value
        if self.current_swa == 1000:
            self.control = Control.FREE
            print(f"Navigation: >> control: {self.control}")
        elif self.current_swa == 2000:
            self.control = Control.FS_I6
            print(f"Navigation: >> control: {self.control}")
        else:
            print("Navigation [ERROR]: >> invalid swa")

    def navigate(self):
        if self.control == Control.FREE:
            if self.mode == Mode.GPS:
                if not self.is_running:
                    print("Navigation: >> not running")
                    return
                if self.goal_lat is None or self.goal_lon is None:
                    print("Navigation: >> waiting for goal")
                    return

                if self.current_lat is None or self.current_lon is None:
                    print("Navigation: >> waiting for gps fix")
                    return

                if self.start_lat is None or self.start_lon is None:
                    self.start_lat = self.current_lat
                    self.start_lon = self.current_lon

                linear_x, angular_z = self.navigation_controller.compute_twist_stanley(
                    start_lat=self.start_lat,
                    start_lon=self.start_lon,
                    current_lat=self.current_lat,
                    current_lon=self.current_lon,
                    target_lat=self.goal_lat,
                    target_lon=self.goal_lon,
                    current_heading=self.current_heading,
                    linear_vel_x=self.linear_vel_x
                )
                if linear_x == 0 and angular_z == 0:
                    self.is_segment_done = True
                    self.start_lat = None
                    self.start_lon = None
                # self.set_twist(linear_x, angular_z)
                print(f"Navigation: >> linear_x: {linear_x} angular_z: {angular_z}")
            elif self.mode == Mode.CAMERA:
                pass
            elif self.mode == Mode.VELOCITY:
                print(f"Navigation: >> x: {self.control_x} z: {self.control_z}")
                self.set_twist(self.control_x, self.control_z)
        elif self.control == Control.FS_I6:
            print(f"Navigation: >> x: {self.control_x} z: {self.control_z}")
            self.set_twist(self.control_x, self.control_z)

    def clean_up(self):
        self.timer.stop()
        self.navigation_controller.clean_up()
        print("Navigation: >> Navigation stopped")
       
