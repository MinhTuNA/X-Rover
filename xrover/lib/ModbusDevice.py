from .SerialDeviceScanner import DevicePortScanner
from .ConstVariable import RS485, WHEEL
from .ModbusDeviceScanner import DeviceModbusScanner
import serial


class ModbusDevice:
    is_first_connect = True

    def __init__(self, timeout=1, parity="N", stopbits=1, bytesize=8):
        self.device = DevicePortScanner()
        self.rs485_port = self.device.find_rs485_port()
        self.rs485_baudrate = RS485.baudrate
        # self.modbus_scanner = DeviceModbusScanner(
        #     port=self.rs485_port, baudrate=self.rs485_baudrate)
        self.write_mode = RS485.write_mode
        self.read_mode = RS485.read_mode
        if ModbusDevice.is_first_connect:
            ModbusDevice.is_first_connect = False
            self.connect()

    def connect(self):
        try:
            self.rs485 = serial.Serial(
                port=self.rs485_port, baudrate=self.rs485_baudrate, timeout=1
            )
            print("Successfully connected to motor driver")
        except Exception as e:
            print(f"Connection to motor driver failed: {e}")
            raise

    def send_to_driver(self, header=[], payload=[]):
        crc_l, crc_h = self.calculate_crc16_modbus(header + payload)

        data = bytes(header + payload + [crc_l] + [crc_h])
        print(f"data: {data.hex().upper()}")
        self.rs485.write(data)

    def split_into_bytes(self, value):
        high_byte = (value >> 8) & 0xFF
        low_byte = value & 0xFF
        return high_byte, low_byte

    def calculate_crc16_modbus(self, data):
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


class Driver(ModbusDevice):
    def __init__(self, timeout=1, parity="N", stopbits=1, bytesize=8):
        super().__init__(timeout, parity, stopbits, bytesize)
        # self.driver_address = self.modbus_scanner.find_driver_device()
        self.driver_address = 0x01
        self.is_driver_connect = False
        self.speed_mode = WHEEL.speed_mode
        self.torque_mode = WHEEL.torque_mode

    def set_motor(
        self,
        left_rpm=0,
        right_rpm=0,
        left_torque=100,
        right_torque=100,
        left_mode=0x41,
        right_mode=0x41,
    ):
        if self.driver_address <= 0 or self.driver_address > 127:
            print("address invalid")
            return

        # header
        start_address_high, start_address_low = self.split_into_bytes(
            WHEEL.address_start
        )
        num_byte_high, num_byte_low = self.split_into_bytes(WHEEL.num_byte)
        total_byte = WHEEL.num_byte * 2

        # payload
        left_rpm_high, left_rpm_low = self.split_into_bytes(left_rpm)
        right_rpm_high, right_rpm_low = self.split_into_bytes(right_rpm)
        left_torque_high, left_torque_low = self.split_into_bytes(left_torque)
        right_torque_high, right_torque_low = self.split_into_bytes(right_torque)
        left_kp_high, left_kp_low = self.split_into_bytes(self.left_kp)
        right_kp_high, right_kp_low = self.split_into_bytes(self.right_kp)
        left_ki_high, left_ki_low = self.split_into_bytes(self.left_ki)
        right_ki_high, right_ki_low = self.split_into_bytes(self.right_ki)

        header = [
            self.driver_address,
            self.write_mode,
            start_address_high,
            start_address_low,
            num_byte_high,
            num_byte_low,
            total_byte,
            left_mode,
            right_mode,
        ]
        payload = [
            left_rpm_high,
            left_rpm_low,
            right_rpm_high,
            right_rpm_low,
            left_torque_high,
            left_torque_low,
            right_torque_high,
            right_torque_low,
            left_kp_high,
            left_kp_low,
            right_kp_high,
            right_kp_low,
            left_ki_high,
            left_ki_low,
            right_ki_high,
            right_ki_low,
        ]
        self.send_to_driver(header=header, payload=payload)
        self.wait_response(timeout=1)

    def request_data_from_driver(self, start_address=0, num_register=0):
        if self.driver_address <= 0 or self.driver_address > 127:
            print("address invalid")
            return
        start_address_h, start_address_l = self.split_into_bytes(start_address)
        num_register_h, num_register_l = self.split_into_bytes(num_register)
        header = [self.driver_address, self.read]
        payload = [start_address_h, start_address_l, num_register_h, num_register_l]
        self.send_to_driver(header=header, payload=payload)
        data = self.wait_response(timeout=1)
        value = self.parse_register_values(data=data)
        print(f"value >> {value}")

    def wait_response(self, timeout=1):
        data_from_driver = self.rs485.readline(timeout=timeout)
        if data_from_driver is None:
            print("No response data received.")
            return []
        if data_from_driver[1] == self.read_mode:  # dữ liệu trả về từ yêu cầu đọc
            byte_count = data_from_driver[2]
            status_data = data_from_driver[3 : 3 + byte_count]
            crc_received_l, crc_received_h = data_from_driver[-2:]  # kiếm tra checksum
            crc_calculated_l, crc_calculated_h = self.calculate_crc16_modbus(
                data_from_driver[:-2]
            )
            crc_received = (crc_received_h << 8) | crc_received_l
            crc_calculated = (crc_calculated_h << 8) | crc_calculated_l
            if crc_received == crc_calculated:
                return status_data
            else:
                print("CRC check failed.")
                return []

        elif data_from_driver[1] == self.write_mode:  # Dữ liệu trả về từ yêu cầu ghi
            start_address = (data_from_driver[2] << 8) | data_from_driver[3]
            register_count = (data_from_driver[4] << 8) | data_from_driver[5]

            crc_received_l, crc_received_h = data_from_driver[-2:]
            crc_calculated_l, crc_calculated_h = self.calculate_crc16_modbus(
                data_from_driver[:-2]
            )
            crc_received = (crc_received_h << 8) | crc_received_l
            crc_calculated = (crc_calculated_h << 8) | crc_calculated_l

            if crc_received == crc_calculated:
                return {
                    "start_address": start_address,
                    "register_count": register_count,
                }
            else:
                print("CRC check failed.")
                return {}

        elif data_from_driver[1] == 0x80:
            error_code = data_from_driver[2]
            return {"error_code": error_code}

        else:
            print("Unexpected function code in response.")
            return {}

    def parse_register_values(self, data):
        if not data:
            print("No response data received.")
            return []

        register_values = []
        for i in range(0, len(data), 2):
            value = (data[i] << 8) | data[i + 1]
            if value >= (1 << 15):
                value -= 1 << 16
            register_values.append(value)
        return register_values


class Battery(ModbusDevice):
    def __init__(self, timeout=1, parity="N", stopbits=1, bytesize=8):
        super().__init__(timeout, parity, stopbits, bytesize)


# if __name__ == "__main__":
#     driver_device = Driver()
#     battery_device = Battery()
