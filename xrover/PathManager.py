from lib.PathAction import PathAction
from PySide6.QtCore import QObject, Signal, QTimer
from lib.ConstVariable import COMMON
import requests
import json

class PathManager(QObject):
    path_segment_signal = Signal(dict)
    def __init__(self):
        super().__init__()
        self.file_path = COMMON.file_path
        self.path_action = PathAction(file_path=self.file_path)
        self.path_data = self.path_action.get_path_data()
        self.index = 0

        #-------------path logger-------------

        self.lat = None
        self.lon = None
        self.last_lat = None
        self.last_lon = None
        self.file_path_logger = COMMON.file_path_logger
        self.file = None
        self.logger_index = 0
        self.logger_timer_interval = 500
        self.logger_timer = QTimer()
        self.logger_timer.timeout.connect(self.run_logger)

    def get_segment(self,index):
        index_ = index
        self.index = index
        point_1 = self.path_action.get_point(index_)
        start_lat,start_lon = self.path_action.get_coordinate(point_1)
        point_2 = self.path_action.get_point(index_+1)
        end_lat,end_lon = self.path_action.get_coordinate(point_2)
        self.send_segment(start_lat,start_lon,end_lat,end_lon)
        if index_ == len(self.path_data)-2:
            print("PathManager: >> last segment")
            
    def send_segment(self, start_lat, start_lon, end_lat, end_lon):
        segment = {
            "index": self.index,
            "start_lat": start_lat,
            "start_lon": start_lon,
            "end_lat": end_lat,
            "end_lon": end_lon
        }
        self.path_segment_signal.emit(segment)
        print(f"PathManager: >> send segment {start_lat},{start_lon} -> {end_lat},{end_lon}")
    
    def open_file(self):
        try:
            open(self.file_path_logger, "w").close()
            self.file = open(self.file_path_logger, "a")
            print(f"PathManager: >> file opened")
        except Exception:
            print(f"PathManager: >> file not opened")
            
    
    def gps_position(self,lat:str,lon:str):
        self.lat = float(lat)
        self.lon = float(lon)
        print(f"PathManager: >> gps position: {self.lat},{self.lon}")
    
    def start_logging(self,id:int):
        self.logger_index = 0
        self.open_file()
        self.logger_timer.start(self.logger_timer_interval)
        print(f"PathManager: >> start logging")

    def stop_logging(self,id:int):
        self.logger_index = 0
        self.logger_timer.stop()
        self.file.close()
        try:
            with open(self.file_path_logger, "r") as f:
                data = [json.loads(line.strip()) for line in f]
            self.update_path(id, data)
        except Exception as e:
            print(f"PathManager: >> update path {id} error: {e}")

    def run_logger(self):
        if self.lat is None or self.lon is None:
            print("PathManager: >> no gps data")
            return
        if self.last_lat == self.lat and self.last_lon == self.lon:
            print("PathManager: >> no change")
            return
        print(f"PathManager: >> logging")
        entry = {
            "index": self.logger_index,
            "lat": self.lat,
            "lon": self.lon
        }
        self.file.write(f"{json.dumps(entry)}\n")
        self.file.flush()
        self.logger_index += 1
        self.last_lat = self.lat
        self.last_lon = self.lon
        self.lat = None
        self.lon = None

    def update_path(self, id, data):
        try:
            payload = {
                'lines': data
            }
            url = COMMON.api_save_path+str(id)
            response = requests.patch(url, json=payload)
            print(f"PathManager: >> update path {id} response: {response.json()}")
        except Exception as e:
            print(f"PathManager: >> update path {id} error: {e}")
    
    def clean_up(self):
        if self.logger_timer.isActive():
            self.logger_timer.stop()
            print(f"PathManager: >> stop logging")
        if self.file:
            self.file.close()
            print(f"PathManager: >> file closed")

