import json

class PathAction:
    def __init__(self,file_path=None):
        self.path = file_path
        self.path_data = self.open_path_file()
        self.path_data = self.sort_path(self.path_data)

    def open_path_file(self):
        try:
            with open(self.path, "r") as f:
                path_data = json.load(f)
                return path_data
        except FileNotFoundError:
            print(f"File {self.path} not found.")
        except json.JSONDecodeError as e:
            print(f"Error reading JSON: {e}")
            
    def get_path_data(self):
        return self.path_data

    def get_point(self,index):
        return self.path_data[index]
    
    def get_coordinate(self,point):
        return point["lat"],point["lon"]

    def sort_path(self,path_data):
        for i in range(len(path_data)):
            swapped = False
            for j in range(0,len(path_data)-i-1):
                if path_data[j]["index"] > path_data[j+1]["index"]:
                    path_data[j],path_data[j+1] = path_data[j+1],path_data[j]
                    swapped = True
            if not swapped:
                break
        return path_data
    
    
if __name__ == "__main__":
    path_action = PathAction()
    point = path_action.get_point(1)
    print(point)
    
        
        