import sys
from PySide6.QtCore import QObject, Signal, Slot
from PySide6.QtWidgets import QApplication
# from PySide6.QtSerialPort import QSerialPort
from lib.UM982Lib import GPSDevice
from lib.ConstVariable import COMMON
from lib.ConstVariable import GPS
from VariableManager import instance
import os
import time
import serial
import threading
os.environ["QT_QPA_PLATFORM"] = "offscreen"

class GPSRTK(QObject):

    gps_rate_signal = Signal(int)
    gps_position_signal = Signal(str,str,str)
    heading_signal = Signal(float)
    gps_ecef_signal = Signal(str,str,str)
    adrnav_signal = Signal(dict)
    uniheading_signal = Signal(dict)
    rtkstatus_signal = Signal(dict)
    rtcmstatus_signal = Signal(dict)
    bestnavxyz_signal = Signal(dict)


    def __init__(self,port = None):
        super().__init__()
        
        self.rate = None
        self.gps_port = port
        self.gps_baudrate = GPS.baudrate
        self.gps_device = GPSDevice()
        self.gps_serial = None
        instance.load(COMMON.variable_path)
        self.gps_thread = threading.Thread(target=self.read_gps)
        self.gps_thread.daemon = True
        
        
    def _start_gps(self):
        self.connect_to_gps()
        self.gps_thread.start()
        self.load_variable()
        

    # def connect_to_gps(self):
    #     try:
    #         self.gps_serial = QSerialPort(self.gps_port)
    #         self.gps_serial.setBaudRate(self.gps_baudrate)
    #         if not self.gps_serial.open(QSerialPort.OpenModeFlag.ReadWrite):
    #             print("GPS: >> Open GPS port failed")
    #             return
    #         else:
    #             print("GPS: >> GPS port opened")
    #             self.gps_serial.readyRead.connect(self.handle_gps_data)
    #     except Exception as e:
    #         print(f"GPS: >> Error opening gps port: {e}")
    #         raise

    def connect_to_gps(self):
        try:
            self.gps_serial = serial.Serial(
                port=self.gps_port, 
                baudrate=self.gps_baudrate,
                timeout=1,
                dsrdtr=False,
                rtscts=False,
            )
        except Exception as e:
            print(f"GPS: >> Error opening gps port: {e}")
            raise

    def request_status(self):
        self.gps_rate_signal.emit(self.rate)
    
    def load_variable(self):
        self.rate = int(instance.get("gps.rate",1))
        print(f"GPS: >> gps rate: {self.rate}")
        self.set_gps_rate(self.rate)
        
    def format_hex(self, data: bytes) -> str:
        return " ".join(f"{b:02X}" for b in data)

    def set_rate(self, frequency: int):
        self.set_gps_rate(frequency)
        instance.set("gps.rate", frequency)
        instance.save()
        
    def set_gps_rate(self,rate):
        self.rate = rate
        _rate = 1/int(rate)
        _rate = round(_rate, 2)
        cmd = (
            f"{self.gps_device.ADRNAVA_CMD} {_rate}\r\n"
            f"{self.gps_device.BESTNAVXYZA_CMD} {_rate}\r\n"
            f"{self.gps_device.UNIHEADINGA_CMD} {_rate}\r\n"
            f"{self.gps_device.RTCMSTATUSA_CMD} {self.gps_device.RTCM_CMD}\r\n"
            f"{self.gps_device.RTKSTATUSA_CMD} {_rate}\r\n"
        )
        if self.gps_serial.is_open:
            self.gps_serial.write(cmd.encode())
        else:
            print("GPS: >> gps serial is not open")
        
    
    def freset(self):
        cmd = self.gps_device.FRESET_CMD + "\r\n"
        self.gps_serial.write(cmd.encode())
        print("GPS: >> freset")
    
    def save_config(self):
        cmd = self.gps_device.SAVECONFIG_CMD + "\r\n"
        self.gps_serial.write(cmd.encode())
        print("GPS: >> saved config")

    # send rtcm data to gps
    def handle_rtcm(self, rtcm_data: bytes):
        hex_str = self.format_hex(rtcm_data)
        # print(f"GPS: >> rtcm data: {hex_str}")
        # print("GPS: >> send rtcm data")
        if self.gps_serial.is_open:
            self.gps_serial.write(rtcm_data)
    
    # def handle_gps_data(self):
    #     gps_data = self.gps_serial.readLine().data().decode("utf-8",errors="ignore").strip()
    #     # print(f"GPS: >> gps data: {gps_data}")
        
    #     if gps_data:
    #         gps_data_parsed = self.gps_device.parse_um982_data(gps_data)
    #         if gps_data_parsed is None:
    #             return 
    #         if gps_data_parsed["type"] == "rtk_status_msg":
    #             self.rtkstatus_signal.emit(gps_data_parsed)
    #             # print(f"RTK Status: {gps_data_parsed['pos_type']}")
                
    #         if gps_data_parsed["type"] == "uniheading_msg":
    #             self.uniheading_signal.emit(gps_data_parsed)
    #             heading = gps_data_parsed["heading"]
    #             heading = (float(heading) + COMMON.off_set_heading) % 360
    #             print(f"GPS: >> heading >> {heading}")
    #             self.heading_signal.emit(heading)
                
    #         if gps_data_parsed["type"] == "adrnav_msg":
    #             self.adrnav_signal.emit(gps_data_parsed)
    #             pos_type = gps_data_parsed["pos_type"]
    #             lat = gps_data_parsed["lat"]
    #             lon = gps_data_parsed["lon"]
    #             alt = gps_data_parsed["height"]
    #             print(f"GPS: >> lat >> {lat} lon >> {lon} alt >> {alt}")
    #             self.gps_position_signal.emit(lat, lon, alt)
                
    #         if gps_data_parsed["type"] == "rtcm_status_msg":
    #             self.rtcmstatus_signal.emit(gps_data_parsed)
    #             msg_id = gps_data_parsed["msg_id"]
    #             # print(f"GPS: >> RTCM {msg_id}")
                
    #         if gps_data_parsed["type"] == "bestnavxyz_msg":
    #             self.bestnavxyz_signal.emit(gps_data_parsed)
    #             x = gps_data_parsed["P_X"]
    #             y = gps_data_parsed["P_Y"]
    #             z = gps_data_parsed["P_Z"]
    #             self.gps_ecef_signal.emit(x, y, z)
    #             # print(f"GPS: >> ecef >> {x} \n" f"y >> {y} \n" f"z >> {z} \n")
            
    def read_gps(self):
        try:
            while True:
                gps_data = self.gps_serial.readline().decode(errors="ignore").strip()
                if gps_data:
                    gps_data_parsed = self.gps_device.parse_um982_data(gps_data)
                    if gps_data_parsed is None:
                        continue
                    if gps_data_parsed["type"] == "rtk_status_msg":
                        self.rtkstatus_signal.emit(gps_data_parsed)
                        # print(f"RTK Status: {gps_data_parsed['pos_type']}")
                        
                    if gps_data_parsed["type"] == "uniheading_msg":
                        self.uniheading_signal.emit(gps_data_parsed)
                        heading = gps_data_parsed["heading"]
                        heading = (float(heading) + COMMON.off_set_heading) % 360
                        # print(f"GPS: >> heading >> {heading}")
                        self.heading_signal.emit(heading)
                        
                    if gps_data_parsed["type"] == "adrnav_msg":
                        self.adrnav_signal.emit(gps_data_parsed)
                        pos_type = gps_data_parsed["pos_type"]
                        lat = gps_data_parsed["lat"]
                        lon = gps_data_parsed["lon"]
                        alt = gps_data_parsed["height"]
                        # print(f"GPS: >> lat >> {lat} lon >> {lon} alt >> {alt}")
                        self.gps_position_signal.emit(lat, lon, alt)
                        
                    if gps_data_parsed["type"] == "rtcm_status_msg":
                        self.rtcmstatus_signal.emit(gps_data_parsed)
                        msg_id = gps_data_parsed["msg_id"]
                        # print(f"GPS: >> RTCM {msg_id}")
                        
                    if gps_data_parsed["type"] == "bestnavxyz_msg":
                        self.bestnavxyz_signal.emit(gps_data_parsed)
                        x = gps_data_parsed["P_X"]
                        y = gps_data_parsed["P_Y"]
                        z = gps_data_parsed["P_Z"]
                        self.gps_ecef_signal.emit(x, y, z)
                        
        except Exception:
            pass

    def clean_up(self):
        print(f"GPS: >> closed GPSRTK {self.gps_port}")
        self.gps_serial.close()
        


