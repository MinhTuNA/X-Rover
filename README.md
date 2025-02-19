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
├── ConnectServer.py -------------- (kết nối nestjs server)  
├── const.py ---------------------- (biến)  
├── Driver.py --------------------- (giao tiếp driver điều khiển động cơ)  
├── ExecuteProgram.py ------------- (gửi tọa độ đích)  
├── H30IMU.py --------------------- (thư viện cảm biến IMU H30)  
├── IMU.py ------------------------ (đọc cảm biến H30 && publish)  
├── MotionController.py ----------- (điều khiển chuyển động)  
├── MotorController.py ------------ (tính vận tốc && điều khiển động cơ qua driver)  
├── Navigation.py ----------------- (di chuyển A --> B )  
├── PathPlaner.py ----------------- (tính khoảng cách & hướng giữa 2 điểm)
├── S21C.py ----------------------- (đọc cảm biến tiệm cận && publish)  
├── UM982.py ---------------------- (đọc gps && publish)  
  

## danh sách topic
/program_cmd (String) ---------- thực hiện chương trình vd: cmd = { "program_id": id, "cmd": "run",}  
/gps/goal (NavSatFix) ---------- tọa độ điểm đích  
/gps/fix (NavSatFix)  ---------- tọa độ robot  
/compass/heading (Float32) ----- hướng địa lý of robot  
/imu/data (Imu) ---------------- dữ liệu cảm biến IMU  
/rover/vel (Twist) ------------- di chuyển rover  
/delta/move (Point) ------------ di chuyển robot delta  
/status (String) --------------- trạng thái robot  
/sensor/A (Range) -------------- cảm biến tiệm cận  
/sensor/B (Range)  
/sensor/C (Range)  
/sensor/D (Range)  
/sensor/E (Range)  
/sensor/F (Range)  
  


## chi tiết các node:
1. connect_server_node
File: connect_server.py
- Subscriptions: (NavSatFix, '/gps/fix')
- Subscriptions: (Float32, '/compass/heading')
- Publications: (String,'/program_cmd')
chức năng chính: Giao tiếp 2 chiều với server thông qua Socket.IO, gửi dữ liệu đến server, nhận và thực thi lệnh từ server.

2. execute_program_node:
File: execute_program.py
- Subscriptions: (String, '/status')
- Subscriptions: (String, '/program_cmd')
- Publications: (NavSatFix, '/gps/goal')
Chức năng chính: Nhận và thực thi chương trình từ server, gửi tọa độ đích đến robot.

3. imu_node
File: imu_node.py
- Publications: (Imu, '/imu/data')
Chức năng chính: Đọc dữ liệu từ cảm biến IMU H30 và publish dữ liệu IMU.

4. motor_controller_node
File: motor_controller.py
- Subscriptions: (Twist, '/cmd_vel')
Chức năng chính: Nhận lệnh điều khiển vận tốc và tính toán tốc độ bánh xe.

5. navigator_node
File: navigation.py
- Subscriptions: (NavSatFix, '/gps/fix')
- Subscriptions: (Float32, '/compass/heading')
- Subscriptions: (NavSatFix, '/gps/goal')
- Subscriptions: (Imu, '/imu/data')
- Publications: (Twist, '/cmd_vel')
- Publications: (String, '/status')
Chức năng chính: Điều hướng robot từ điểm A đến điểm B, tránh vật cản.

6. s21c_sensors_node
File: s21c_node.py
- Publications: (Range, '/sensor/A')
- Publications: (Range, '/sensor/B')
- Publications: (Range, '/sensor/C')
- Publications: (Range, '/sensor/D')
- Publications: (Range, '/sensor/E')
- Publications: (Range, '/sensor/F')
Chức năng chính: Đọc dữ liệu cảm biến tiệm cận và publish dữ liệu khoảng cách.

7. um982_node
File: um982.py
- Publications: (NavSatFix, '/gps/fix')
- Publications: (Float32, '/compass/heading')
Chức năng chính: Đọc dữ liệu GPS từ module UM982 và publish dữ liệu GPS và heading.

## Cài đặt

