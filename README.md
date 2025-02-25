# Xrover Project

## requirememts
- ros2 humble
- python3

## base station F9P: sử dụng ucenter2 cấu hình gps
### cách 1: quick configuration
#### 1: survey-in mode:
- chọn survey in
- nhập minium duration (s):
- position accuracy limit (mm): 
- mở message view: UBX -> NAV -> SVIN: quan sát độ chính xác cải thiện dần 
#### 2: fixed mode:
- sau khi có được vị trí từ surveyin mode:
    + chọn fixed
    + nhập Fixed position accuracy (mm):
    + Position type: 0 - ECEF      1 - LLH
    + nhập tọa độ tương ứng
- config base station phát tín hiệu RTCM3 ra cổng usb:
    + CFG-USBOUTPROT-RTCM3X 1
    + CFG-MSGOUT-RTCM_3X_TYPE1005_USB 1
    + CFG-MSGOUT-RTCM_3X_TYPE1230_USB 1
    + CFG-MSGOUT-RTCM_3X_TYPE1074_USB 1
    + CFG-MSGOUT-RTCM_3X_TYPE1084_USB 1
    + CFG-MSGOUT-RTCM_3X_TYPE1124_USB 1
### cách 2: advanced configuration 
#### 1: survey-in mode:
    - CFG-TMODE-TMODE : value = 1
    - CFG-TMODE-SVIN_MIN_DUR: value: (thời gian tối thiểu thực hiện surveyin)
    - CFG-TMODE-SVIN_ACC_LIMIT: value: (độ chính xác mong muốn)  
    mở message view: UBX -> NAV -> SVIN: quan sát độ chính xác cải thiện dần  
    => đợi gps cải thiện độ chính xác tới mức mong muốn && 2
#### 2: fixed mode:
- config base station phát tín hiệu RTCM3 ra cổng usb:
    + CFG-USBOUTPROT-RTCM3X 1
    + CFG-MSGOUT-RTCM_3X_TYPE1005_USB 1
    + CFG-MSGOUT-RTCM_3X_TYPE1230_USB 1
    + CFG-MSGOUT-RTCM_3X_TYPE1074_USB 1
    + CFG-MSGOUT-RTCM_3X_TYPE1084_USB 1
    + CFG-MSGOUT-RTCM_3X_TYPE1124_USB 1
- fixed mode:
    + CFG-TMODE-POS_TYPE: 1 - LLH
    + CFG-TMODE-LAT: tọa độ
    + CFG-TMODE-LON: tọa độ
    + CFG-TMODE-HEIGHT: độ cao  
    hoặc
    + CFG-TMODE-POS_TYPE: 0 - ECEF
    + CFG-TMODE-ECEF_X: x
    + CFG-TMODE-ECEF_Y: y
    + CFG-TMODE-ECEF_Z: z
    ------------------------------------------------
    + CFG-TMODE-MODE: 2 - fixed

## rover UM982: dùng phần mềm UPRECISE cấu hình gps
trong mục receiver configurations: tìm & cấu hình các tham số  
- $CONFIG, ANTENNA, CONFIG ANTENNA POWERON*7A
- $CONFIG, NMEAVERSION, CONFIG NMEAVERSION V410*47
- $CONFIG, RTK, CONFIG RTK TIMEOUT 600*69
- $CONFIG, RTK, CONFIG RTK RELIABILITY 3 1*76
- $CONFIG, PPP, CONFIG PPP TIMEOUT 300*6C
- $CONFIG, HEADING, CONFIG HEADING RELIABILITY 3*67
- $CONFIG, HEADING, CONFIG HEADING TRACTOR*69
- $CONFIG, HEADING, CONFIG HEADING LENGTH 0.00 0.00*38
- $CONFIG, DGPS, CONFIG DGPS TIMEOUT 600*69
- $CONFIG, RTCMB1CB2A, CONFIG RTCMB1CB2A ENABLE*25
- $CONFIG, ANTENNADELTAHEN, CONFIG ANTENNADELTAHEN 0.0000 0.0000 0.0000*3A
- $CONFIG,PPS, CONFIG PPS ENABLE GPS POSITIVE 500000 1000 0 0*6E
- $CONFIG, SIGNALGROUP, CONFIG SIGNALGROUP 4 5*05
- $CONFIG, ANTIJAM, CONFIG ANTIJAM AUTO*2B
- $CONFIG,AGNSS, CONFIG AGNSS DISABLE*70

## Mục lục

xrover/  
├── launch/  
├── xrover/  
├── package.xml  
├── README.MD  
├── setup.cfg  
├── setup.py  
  
xrover/xrover/ 
├── /lib -------------------------- (thư viện liên quan)  
├── ConnectServer.py -------------- (kết nối nestjs server)  
├── Delta.py ---------------------- (điều khiển robot delta)  
├── ExecuteProgram.py ------------- (gửi tọa độ đích)  
├── IMU.py ------------------------ (đọc cảm biến IMU H30 && publish)  
├── MotorController.py ------------ (điều khiển động cơ)  
├── Navigation.py ----------------- (di chuyển A --> B )  
├── S21C.py ----------------------- (đọc cảm biến tiệm cận && publish)  
├── UM982.py ---------------------- (đọc gps && publish)  
  

## danh sách topic
/program_cmd (String) -------------- thực hiện chương trình vd: cmd = { "program_id": id, "cmd": "run",}  
/gps/goal (NavSatFix) -------------- tọa độ điểm đích  
/gps/fix (NavSatFix)  -------------- tọa độ robot  
/gps/heading (Float32) ------------- hướng địa lý robot  
/imu/data (Imu) -------------------- dữ liệu cảm biến IMU  
/rover/vel (Twist) ----------------- di chuyển rover  
/delta/move (Point) ---------------- di chuyển robot delta  
/delta/request_status (String)------ yêu cầu trả về trạng thái robot delta tại topic /status  
/status (String) ------------------- trạng thái robot  
/sensor/A (Range) ------------------ cảm biến tiệm cận  
/sensor/B (Range)  
/sensor/C (Range)  
/sensor/D (Range)  
/sensor/E (Range)  
/sensor/F (Range)  
  


## chi tiết các node:
1. connect_server_node
File: ConnectServer.py
chức năng chính: Giao tiếp 2 chiều với server thông qua Socket.IO, gửi dữ liệu đến server, nhận và thực thi lệnh từ server.

2. delta_node
File: Delta.py
chức năng chính: điều khiển robot delta.

3. execute_program_node:
File: ExecuteProgram.py
Chức năng chính: Nhận và thực thi chương trình từ server, gửi tọa độ đích đến robot.

4. imu_node
File: IMU.py
Chức năng chính: Đọc dữ liệu từ cảm biến IMU H30 và publish dữ liệu IMU.

5. motor_controller_node
File: Motorcontroller.py
Chức năng chính: Nhận lệnh điều khiển &&  điều khiển bánh xe.

6. navigator_node
File: Navigation.py
Chức năng chính: Điều hướng robot đi từ điểm A đến điểm B theo tọa độ gps, tránh vật cản.

7. s21c_sensors_node
File: S21c.py
Chức năng chính: Đọc dữ liệu cảm biến tiệm cận và publish dữ liệu khoảng cách.

8. um982_node
File: UM982.py
Chức năng chính: Đọc dữ liệu GPS từ module UM982 và publish dữ liệu GPS và heading.

## Cài đặt

