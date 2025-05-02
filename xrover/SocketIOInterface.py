from PySide6.QtCore import QObject,Signal
from lib.ConstVariable import COMMON
import time
import socketio
import os
import json


sio = socketio.Client(
    reconnection=True,             # bật reconnect
    reconnection_attempts=0,       # 0 = vô hạn
    reconnection_delay=1,          # bắt đầu delay 1s
    reconnection_delay_max=5,      # delay tối đa 5s
)

class SocketIOInterface(QObject):

    connected_signal = Signal()
    disconnected_signal = Signal()
    start_logging_signal = Signal()
    stop_logging_signal = Signal()
    set_mode_signal = Signal(int)
    target_point_signal = Signal(str,str)
    move_delta_signal = Signal(float,float,float)
    delta_go_home_signal = Signal()
    request_delta_status_signal = Signal()
    request_rover_status_signal = Signal()
    freset_signal = Signal()
    save_config_signal = Signal()
    rate_signal = Signal(int)
    rover_vel_signal = Signal(float,float)
    start_signal = Signal()
    stop_signal = Signal()
    

    def __init__(self):
        super().__init__()
        self.sio = sio
        self.isConnected = False
        self.hasRegisteredEvents = False
        if not self.hasRegisteredEvents:
            self.sio.on("connect", self.on_connect, namespace="/rover")
            self.sio.on("path", self.save_path, namespace="/rover")
            self.sio.on("move_rover", self.rover_vel, namespace="/rover")
            self.sio.on("start", self.start, namespace="/rover")
            self.sio.on("stop", self.stop, namespace="/rover")
            self.sio.on("start_logging", self.start_logging, namespace="/rover")
            self.sio.on("stop_logging", self.stop_logging, namespace="/rover")
            self.sio.on("delta_go_home", self.delta_go_home, namespace="/rover")
            self.sio.on("move_delta", self.move_delta, namespace="/rover")
            self.sio.on("disconnect", self.on_disconnect, namespace="/rover")
            self.sio.on("freset", self.freset_callback, namespace="/rover")
            self.sio.on("save_config", self.save_config_callback, namespace="/rover")
            self.sio.on("set_rate", self.rate_callback, namespace="/rover")
            self.sio.on(
                "request_delta_status", self.request_delta_status, namespace="/rover"
            )
            self.sio.on(
                "request_rover_status", self.request_rover_status, namespace="/rover"
            )
            self.sio.on("set_mode", self.set_mode, namespace="/rover")
            self.sio.on("target_point", self.target_point, namespace="/rover")
            self.hasRegisteredEvents = True

    def on_connect(self):
        self.isConnected = True
        print("SocketIOInterface: >> Connected to socketio server")
        self.connected_signal.emit()
    
    def save_path(self, data):
        file_path = COMMON.file_path

        if os.path.exists(file_path):
            print(f"File {file_path} already exists. Updating contents...")
        else:
            print(f"File {file_path} does not exist. Creating new file...")

        with open(file_path, "w") as json_file:
            json.dump(data, json_file, indent=4)

        print(f"Data has been saved to {file_path}")

    def rover_vel(self, data):
        x = data["x"]
        z = data["z"]
        print(f"SocketIOInterface: >> rover vel: x: {x}, z: {z}")
        self.rover_vel_signal.emit(x, z)

    def start(self, data):
        self.start_signal.emit()
        print(f"SocketIOInterface: >> started")

    def stop(self, data):
        self.stop_signal.emit()
        print(f"SocketIOInterface: >> stopped")
    
    def start_logging(self, data):
        self.start_logging_signal.emit()
        print(f"SocketIOInterface: >> started logger {data}")

    def stop_logging(self, data):
        self.stop_logging_signal.emit()
        print(f"SocketIOInterface: >> stopped logger {data}")

    def set_mode(self, data):
        self.set_mode_signal.emit(data)
        print(f"SocketIOInterface: >> set mode: {data}")
    
    def freset_callback(self, data):
        self.freset_signal.emit()
        print(f"SocketIOInterface: >> freset")
        
    def save_config_callback(self, data):
        self.save_config_signal.emit()
        print(f"SocketIOInterface: >> save config")
        
    def rate_callback(self, data):
        self.rate_signal.emit(data)
        print(f"SocketIOInterface: >> set rate: {data}")

    def move_delta(self, data):
        x = float(data["x"])
        y = float(data["y"])
        z = float(data["z"])
        # print(f"delta >> x: {pos.x}, y: {pos.y}, z: {pos.z} ")
        print(f"SocketIOInterface: >> delta >> x: {x}, y: {y}, z: {z} ")
        self.move_delta_signal.emit(x, y, z)

    def delta_go_home(self, data):
        self.delta_go_home_signal.emit()

    def request_delta_status(self, data):
        self.request_delta_status_signal.emit()

    def request_rover_status(self, data):
        self.request_rover_status_signal.emit()
    
    def target_point(self, data):
        latitude = data["latitude"]
        longitude = data["longitude"]
        self.target_point_signal.emit(latitude, longitude)
        print(f"SocketIOInterface: >> target point: {latitude}, {longitude}")


    def on_disconnect(self,*args):
        self.isConnected = False
        print(f"SocketIOInterface: >> Disconnected")
        self.disconnected_signal.emit()


    #--------------------------------
    def gps_position(self,lat: str,lon:str,alt:str):
        self.emit_to_server(name="gps_position",data={"lat":lat,"lon":lon,"alt":alt})

    def heading(self,heading:str):
        self.emit_to_server(name="heading",data=heading)

    def imu(self,data:dict):
        # print(f"SocketIOInterface: >> imu: {data}")
        self.emit_to_server(name="imu",data=data)
    
    def rover_mode(self,mode:int):
        print(f"SocketIOInterface: >> rover mode: {mode}")
        self.emit_to_server(name="rover_mode",data=mode)

    def gps_rate(self,rate:int):
        self.emit_to_server(name="gps_rate",data=rate)
    
    def delta_position(self,x:float,y:float,z:float,w:float):
        self.emit_to_server(name="delta_position",data={"x":x,"y":y,"z":z,"w":w})
        print(f"SocketIOInterface: >> delta position: {x}, {y}, {z}, {w}")
    
    def is_delta_connected(self,data:bool):
        self.emit_to_server(name="is_delta_connected",data=data)
        print(f"SocketIOInterface: >> is delta connected: {data}")
    
    def is_rover_connected(self,data:bool):
        print(f"SocketIOInterface: >> is rover connected: {data}")
        self.emit_to_server(name="is_rover_connected",data=data)
    
    def gps_adrnava(self,data:dict):
        self.emit_to_server(name="adrnav",data=data)
    
    def gps_uniheading(self,data:dict):
        self.emit_to_server(name="uniheading",data=data)
    
    def gps_rtkstatus(self,data:dict):
        self.emit_to_server(name="rtkstatus",data=data)
    
    def gps_rtcmstatus(self,data:dict):
        self.emit_to_server(name="rtcmstatus",data=data)
    
    def gps_bestnav(self,data:dict):
        self.emit_to_server(name="bestnav",data=data)

    #--------------------------------
    def emit_to_server(self,name:str,data):
        if self.isConnected:
            self.sio.emit(name,data,namespace="/rover")
        else:
            print("Not connected to server")


    def runSocketIO(self):
        url = COMMON.server_address
        while True:
            try:
                sio.connect(url)
                # print(f"Connection established to server at {sio.eio}")
                sio.wait()  
            except socketio.exceptions.ConnectionError as e:
                print(f"Connection failed: {e}")
                time.sleep(3)
    
