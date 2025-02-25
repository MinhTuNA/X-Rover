from pymodbus.client import ModbusSerialClient


class DeviceModbusScanner():
    def __init__(self, port="/dev/ttyUSB0", baudrate=9600, timeout=1, parity='N', stopbits=1, bytesize=8):
        self.modbus_port = port
        self.modbus_baudrate = baudrate
        self.err_code_register = 0x00
        self.mx_driver_err_code = [0, 1, 5, 6, 7, 8, 11, 12, 13, 14, 27, 28]
        self.battery_err_code = []
        self.is_connect = False
        self.client = ModbusSerialClient(
            port=self.modbus_port,
            baudrate=self.modbus_baudrate,
            timeout=timeout,
            parity=parity,
            stopbits=stopbits,
            bytesize=bytesize,
        )
        self.devices_address = self.list_modbus_device()
        # self.mx_driver_address = self.find_driver_device()
        # self.battery_address = self.find_battery_device()

    def list_modbus_device(self):
        list_devices_address = []
        if self.client.connect():
            self.is_connect = True
            for address in range(1, 128):
                response = self.client.read_holding_registers(
                    address=self.err_code_register,
                    count=1,
                    slave=address
                )
                if response and not response.isError():
                    list_devices_address.append(address)
            self.client.close()
        else:
            self.is_connect = False

        return list_devices_address

    def find_driver_device(self):
        self.client.connect()
        for address in self.devices_address:
            try:
                res = self.client.read_holding_registers(
                    address=self.err_code_register,
                    count=1,
                    slave=address
                )
                if res and not res.isError():
                    if res.registers[0] in self.mx_driver_err_code:
                        self.client.close()
                        return address
            except:
                self.client.close()
        self.client.close()
        return None

    def find_battery_device(self):
        self.client.connect()
        for address in self.devices_address:
            try:
                res = self.client.read_holding_registers(
                    address=self.battery_err_code,
                    count=1,
                    slave=address
                )
                if res and not res.isError():
                    if res.registers[0] in self.mx_driver_err_code:
                        self.client.close()
                        return address
            except:
                self.client.close()
        self.client.close()
        return None
