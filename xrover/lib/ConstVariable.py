class COMMON:
    z_safe = -831.2
    rover_speed = 0.3
    distance_threshold = 0.05
    angle_threshold = 5
    wheel_radius = 0.2
    wheel_base = 0.75
    off_set_heading = 90
    server_address = "http://192.168.50.249:8901/rover"
    api_save_path = "http://192.168.50.249:8901/api/path/"
    tcp_server_ip = "192.168.50.249"
    tcp_server_port = 8765
    motorKi = 0.1
    motorKp = 0.1
    motorKd = 0.1
    file_path_logger = "/home/deltax/xrover_ws/src/x-rover/path_logger.jsonl"
    file_path = "/home/deltax/xrover_ws/src/x-rover/path.json"
    variable_path = "/home/deltax/xrover_ws/src/x-rover/global_variable.ini"
    angleKi = 0.1
    angleKp = 0.1
    angleKd = 0.1

class DELTA:
    baudrate = 115200
    serial_number = "15341270"

class IMU:
    baudrate = 460800
    serial_number = "5919000639"
    uart_buf_len = 512
    cnt_per_seconds = 1000


class GPS:
    baudrate = 115200
    serial_number = "DU0D6VH3"

class RF_MODULE:
    baudrate = 57600
    serial_number = "DU0D66DB"


class ULTRASONIC:
    baudrate = 115200
    serial_number = "5761037141"
    min_range = 0.03  # m
    max_range = 4.5  # m


class RS485:
    baudrate = 115200
    serial_number = "B003LK5S"
    read_mode = 0x03
    write_mode = 0x10


class FS_I6:
    serial_number = "0001"


class WHEEL:
    m_left_kp = 1000
    m_right_kp = 1000
    m_left_ki = 2000
    m_right_ki = 2000
    speed_mode = 0x41
    torque_mode = 0x01
    address_start = 0x00
    num_byte = 0x0009
    ratio = 5
    torque = 255
