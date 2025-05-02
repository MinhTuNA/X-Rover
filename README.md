# Xrover Project

## Mục lục

xrover/  
├── launch/  
├── xrover/  
├── global_variable.ini  
├── package.xml  
├── path_logger.jsonl (path đã log)  
├── path.json (path từ database)  
├── README.MD  
├── setup.cfg  
├── setup.py  
├── requirements.txt  
 
xrover/xrover/ 
├── /lib -------------------------- (thư viện liên quan)  
├── Delta.py ---------------------- (robot delta)  
├── FS_I6.py ---------------------- (tay điều khiển)  
├── GPSRTK.py --------------------- (cảm biến GPS)  
├── IMU.py ------------------------ (cảm biến IMU)  
├── main.py ------------------------ (chương trình chính)  
├── Navigation.py ----------------- (di chuyển)  
├── ros2Interface.py -------------- (giao tiếp với các node khác)  
├── PathLogger.py ----------------- (ghi lại hành trình)  
├── RTCMReceiver.py --------------- (tín hiệu hiệu chỉnh RTCM)  
├── S21C.py ----------------------- (cảm biến tiệm cận)  
├── SignalLight.py ---------------- (đèn tín hiệu)  
├── SocketIOInterface.py ---------- (kết nối với web)  
  

## danh sách topic
điều khiển bánh xe (rpm) >> "navigation/rpm" - Float32MultiArray


