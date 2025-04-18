import json
from lib.ConstVariable import COMMON
import time
class TestContinuosPath:
    def __init__(self):
        self.path = COMMON.file_path
        self.path_data = None
        self.open_path_file()
        
    def open_path_file(self):
        with open(self.path, "r") as f:
            self.path_data = json.load(f)
            
    def get_path_data(self):
        return self.path_data

    def get_line(self,line_id,path_data):
        for line in path_data:
            if line["lineIndex"] == line_id:
                return line
        return None
    def get_position(self,line):
        for i in range(len(line["points"])):
            if line["points"][i]["position"] == "start":
                start_lat = line["points"][i]["point"]["latitude"]
                start_lon = line["points"][i]["point"]["longitude"]
            elif line["points"][i]["position"] == "end":
                end_lat = line["points"][i]["point"]["latitude"]
                end_lon = line["points"][i]["point"]["longitude"]
        return start_lat,start_lon,end_lat,end_lon
    

if __name__ == "__main__":
    test_continuos_path = TestContinuosPath()
    path_data = test_continuos_path.get_path_data()
    line1_data = test_continuos_path.get_line(1,path_data)
    start_lat,start_lon,end_lat,end_lon = test_continuos_path.get_position(line1_data)
    print(f"Move >> start_lat: {start_lat}, start_lon: {start_lon}, end_lat: {end_lat}, end_lon: {end_lon}")
    last_lat,last_lon = end_lat,end_lon
    for i in range(2,len(path_data)+1):
        line_data = test_continuos_path.get_line(i,path_data)
        start_lat,start_lon,end_lat,end_lon = test_continuos_path.get_position(line_data)
        print(f"Move >> start_lat: {last_lat}, start_lon: {last_lon}, end_lat: {start_lat}, end_lon: {start_lon}")
        print(f"Move >> start_lat: {start_lat}, start_lon: {start_lon}, end_lat: {end_lat}, end_lon: {end_lon}")
        last_lat,last_lon = end_lat,end_lon
            
            
            
            
            
            

        
        
        
        
