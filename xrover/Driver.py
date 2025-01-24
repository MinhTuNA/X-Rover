from .Const import *
import serial
class Driver:
    def __init__(self):
        super().__init__()
        self.rs485 = None
        self.rs485_port = RS485_PORT
        self.rs485_baudrate = RS485_BAUDRATE
        
        self.write = WRITE_REGISTER
        self.read = READ_REGISTER
        self.speed_mode = speed_mode
        self.torque_mode = torque_mode
        self.left_kp = m_left_kp
        self.right_kp = m_right_kp
        self.left_ki = m_left_ki
        self.right_ki = m_right_ki
        
        
    
    def connect(self):
        try:
            self.rs485 = serial.Serial(
                port=self.rs485_port, baudrate=self.rs485_baudrate, timeout=1
            )
            self.get_logger().info("Successfully connected to motor driver")
        except Exception as e:
            self.get_logger().error(f"Connection to motor driver failed: {e}")
            raise
    
    def send_to_driver(self, header= [], payload=[]):
        crc_l, crc_h = self.calculate_crc16_modbus(header+payload)

        data = bytes(header + payload + [crc_l] + [crc_h])
        print(f"data: {data.hex().upper()}")
        self.rs485.write(data)
        
    def read_from_driver(self,num_registers=0):
        try:
            response = self.rs485.read(5 + 2 * num_registers)
            if len(response) <5 :
                print("Incomplete response received")
                return None
            if response[1] != self.read:
                print("Unexpected function code in response")
                return None
            byte_count = response[2]
            data = response[3:3 + byte_count]
            crc_received_l, crc_received_h = response[-2:]
            crc_calculated_l, crc_calculated_h = self.calculate_crc16_modbus(response[:-2])
            crc_received = (crc_received_h << 8) | crc_received_l
            crc_calculated = (crc_calculated_h << 8) | crc_calculated_l
            if crc_received != crc_calculated:
                print("CRC mismatch in response")
                return None
            
            return data
        except Exception as e:
                print(f"Error reading from driver: {e}")
                return None    
    
    def split_into_bytes(self,value):
        high_byte = (value >> 8) & 0xFF  
        low_byte = value & 0xFF
        return high_byte, low_byte
    def calculate_crc16_modbus(self,data):
        crc = 0xFFFF 
        poly = 0xA001

        for byte in data:
            crc ^= byte 
            for _ in range(8):
                if crc & 0x0001: 
                    crc = (crc >> 1) ^ poly
                else:
                    crc >>= 1
        return crc & 0xFF, (crc >> 8) & 0xFF
    
    def motor_controller(self,device_address = 1, left_rpm = 0, right_rpm = 0, left_torque = 100, right_torque = 100):
        if(device_address <= 0 or device_address > 127):
            self.get_logger().error("address invalid")
            return
        left_rpm_high, left_rpm_low = self.split_into_bytes(left_rpm)
        right_rpm_high, right_rpm_low = self.split_into_bytes(right_rpm)
        left_torque_high, left_torque_low = self.split_into_bytes(left_torque)
        right_torque_high, right_torque_low = self.split_into_bytes(right_torque)
        left_kp_high, left_kp_low = self.split_into_bytes(self.left_kp)
        right_kp_high, right_kp_low = self.split_into_bytes(self.right_kp)
        left_ki_high, left_ki_low = self.split_into_bytes(self.left_ki)
        right_ki_high, right_ki_low = self.split_into_bytes(self.right_ki)
        
        header = [device_address,self.write,0x00, 0x00, 0x00, 0x09, 0x12,self.left_mode, self.right_mode]
        payload = [
            left_rpm_high, left_rpm_low,
            right_rpm_high, right_rpm_low,
            left_torque_high, left_torque_low,
            right_torque_high, right_torque_low,
            left_kp_high, left_kp_low,
            right_kp_high, right_kp_low,
            left_ki_high, left_ki_low,
            right_ki_high, right_ki_low,
        ]
        self.send_to_driver(header=header, payload=payload)
        
    def request_data_from_driver(self, device_address = 1, start_address = 0, num_register = 0):
        if(device_address <= 0 or device_address > 127):
            self.get_logger().error("address invalid")
            return
        start_address_h, start_address_l = self.split_into_bytes(start_address)
        num_register_h, num_register_l = self.split_into_bytes(num_register)
        header = [device_address, self.read]
        payload = [ start_address_h, start_address_l,
                    num_register_h, num_register_l]
        self.send_to_driver(header=header,payload=payload)
        data=self.read_from_driver(num_registers=num_register)
        value = self.parse_register_values(data=data)
        self.get_logger().info(f"{value}")
        
    def parse_register_values(self, data):
        if not data:
            print("No response data received.")
            return []

        register_values = []
        for i in range(0, len(data), 2):
            value = (data[i] << 8) | data[i + 1]
            if value >= (1 << 15):
                value -= (1 << 16)
            register_values.append(value)
        return register_values
    