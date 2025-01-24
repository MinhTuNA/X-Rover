# Server IP
SERVER_IP = "http://192.168.1.11:8901"

# IMU
H30_IMU_BAUDRATE = 460800
H30_IMU_PORT = "/dev/ttyACM1"
H30_UART_RX_BUF_LEN = 512
CNT_PER_SECOND = 1000


# ultrasonic sensor
FIELD_OF_VIEW = 1.047 # (rad)
MIN_RANGE = 0.03      # (m)
MAX_RANGE = 4.5       # (m)
ULTRASONIC_BAUDRATE = 115200
ULTRASONIC_PORT = "/dev/ttyACM0"

# rtk gps um982
UM982_BAUDRATE = 115200
UM982_PORT = "/dev/ttyUSB0"

# RS485
RS485_BAUDRATE = 9600
RS485_PORT = ""
m_left_kp = 1000
m_right_kp = 1000
m_left_ki = 2000
m_right_ki = 2000
speed_mode = 0x41
torque_mode = 0x01
READ_REGISTER = 0x03
WRITE_REGISTER = 0x10

Ki = 0.1
Kp = 0.1
Kd = 0.1

rover_speed = 0.1
distance_threshold = 0.1
angle_threshold = 20

