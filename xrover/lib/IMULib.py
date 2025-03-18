import struct
import serial
import time
from collections import deque
import os
from .ConstVariable import IMU


DATA_BUF_SIZE = 1024
PROTOCOL_FIRST_BYTE = 0x59
PROTOCOL_SECOND_BYTE = 0x53
PROTOCOL_MIN_LEN = 7
CNT_PER_SECOND = 1000
UART_RX_BUF_LEN = 128
DEG_TO_RAD_FACTOR = 57.29577952383886

# Define constants
SENSOR_TEMP_ID = 0x01
ACCEL_ID = 0x10
GYRO_ID = 0x20
MAGNETIC_NORM_ID = 0x30
RAW_MAGNETIC_ID = 0x31
EULER_ID = 0x40
QUATERNION_ID = 0x41
UTC_ID = 0x50
SAMPLE_TIMESTAMP_ID = 0x51
DATA_READY_TIMESTAMP_ID = 0x52
LOCATION_ID = 0x68
SPEED_ID = 0x70
STATUS_ID = 0x80

SENSOR_TEMP_FACTOR = 0.01
ACC_DATA_FACTOR = 0.000001
GYRO_DATA_FACTOR = 0.000001
MAG_NORM_DATA_FACTOR = 0.000001
MAG_RAW_DATA_FACTOR = 0.001
EULER_DATA_FACTOR = 0.000001
QUATENION_DATA_FACTOR = 0.000001
HIGH_RES_LONG_LAT_DATA_FACTOR = 0.0000000001
ALT_DATA_FACTOR = 0.001
VEL_DATA_FACTOR = 0.001


class YesenseDecoder:
    def __init__(self):
        self.decode_data = bytearray(DATA_BUF_SIZE)
        self.decode_buf_len = 0
        self.cmd_info = {"flg": 0, "result": 0x00, "data_class": 0x00}
        self.std_out_decoder = StdOutDecoder()
        self.msg_count = 0
        self.timing_count = 0
        self.msg_rate = 0

    def data_proc(self, data: bytes, result: dict):
        if len(data) + self.decode_buf_len > DATA_BUF_SIZE:
            self.clear_buf_data(0, self.decode_buf_len - 1)
            return "BUF_FULL"

        self.decode_data[self.decode_buf_len : self.decode_buf_len + len(data)] = data
        self.decode_buf_len += len(data)

        ret = self.std_out_decoder.data_proc(
            self.decode_data[: self.decode_buf_len], result
        )
        if ret == "ANALYSIS_OK":
            msg_idx = self.std_out_decoder.msg_idx_obt()
            self.clear_buf_data(msg_idx["st_idx"], msg_idx["end_idx"])
        elif ret in ["NO_HEADER", "DATA_LEN_ERR", "CRC_ERR"]:
            msg_idx = self.std_out_decoder.msg_idx_obt()
            print(
                f"Decoding Error: {ret}. Clearing buffer from {msg_idx['st_idx']} to {msg_idx['end_idx']}"
            )
            self.clear_buf_data(msg_idx["st_idx"], msg_idx["end_idx"])

        return ret

    def clear_buf_data(self, st_idx: int, end_idx: int):
        if st_idx > end_idx or st_idx >= self.decode_buf_len:
            print(f"Invalid indices for clear_buf_data: start={st_idx}, end={end_idx}")
            return "PARA_ERR"

        cnt = end_idx - st_idx + 1
        if cnt >= self.decode_buf_len:
            self.decode_data = bytearray(DATA_BUF_SIZE)
            self.decode_buf_len = 0
        else:
            remaining_data = self.decode_data[end_idx + 1 : self.decode_buf_len]
            self.decode_data[: len(remaining_data)] = remaining_data
            self.decode_buf_len -= cnt

        return "ANALYSIS_OK"

    def read_from_uart(self, port: str, baudrate: int, timeout: float = 1.0):
        try:
            with serial.Serial(port, baudrate, timeout=timeout) as ser:
                ser.flushInput()
                last_time = time.time()
                consecutive_errors = 0
                while True:
                    data = ser.read(UART_RX_BUF_LEN)
                    if data:
                        result = {}
                        ret = self.data_proc(data, result)
                        if ret == "ANALYSIS_OK":
                            self.msg_count += 1
                            consecutive_errors = 0
                            tid = result.get("tid")
                            acc = result.get("acc")
                            gyro = result.get("gyro")
                            euler = result.get("euler")
                            quat = result.get("quat")
                            print(
                                # f"\ntid >> {tid}"
                                # f"\nacc >> {acc}"
                                # f"\ngyro >> {gyro}"
                                f"\neuler >> {int(euler[0])} {int(euler[1])} {int(euler[2])}"
                                # f"\nquat >> {quat}"
                            )

                        elif ret == "BUF_FULL":
                            consecutive_errors += 1
                            if consecutive_errors >= 3:
                                print(
                                    "Too many buffer full errors, stopping processing."
                                )
                                break
                        elif ret != "NO_HEADER":
                            print(f"Decoding Error: {ret}")
                    current_time = time.time()
                    elapsed_time = (current_time - last_time) * 1000
                    self.timing_count += elapsed_time

                    if self.timing_count >= CNT_PER_SECOND:
                        self.msg_rate = self.msg_count
                        self.msg_count = 0
                        self.timing_count = 0
                        print(f"Message Rate: {self.msg_rate} msgs/sec")

                    last_time = current_time

        except serial.SerialException as e:
            print(f"UART Error: {e}")

    def read_from_uart_(self, ser, uart_rx_buf_len):
        try:
            data = ser.read(uart_rx_buf_len)
            if data:
                result = {}
                ret = self.data_proc(data, result)
                if ret == "ANALYSIS_OK":
                    self.msg_count += 1
                    tid = result.get("tid")
                    acc = result.get("acc")
                    gyro = result.get("gyro")
                    euler = result.get("euler")
                    quat = result.get("quat")
                    return {
                        "tid": tid,
                        "acc": acc,
                        "gyro": gyro,
                        "euler": euler,
                        "quat": quat,
                    }
                elif ret == "BUF_FULL":
                    return "BUF_FULL"
                elif ret != "NO_HEADER":
                    return f"Decoding Error: {ret}"
        except serial.SerialException as e:
            return f"UART Error: {e}"


class StdOutDecoder:
    def __init__(self):
        self.msg_idx = {"st_idx": 0, "end_idx": 0}

    def crc_calc(self, data: bytes):
        ck1, ck2 = 0, 0
        for byte in data:
            ck1 = (ck1 + byte) % 256
            ck2 = (ck2 + ck1) % 256
        return (ck2 << 8) | ck1

    def data_proc(self, data: bytes, result: dict):
        ptr = 0
        cnt = len(data)

        while cnt > 1:
            if (
                data[ptr] == PROTOCOL_FIRST_BYTE
                and data[ptr + 1] == PROTOCOL_SECOND_BYTE
            ):
                break
            ptr += 1
            cnt -= 1

        if cnt <= 1:
            return "NO_HEADER"

        self.msg_idx["st_idx"] = ptr

        if cnt <= PROTOCOL_MIN_LEN:
            return "DATA_LEN_ERR"

        header_len = data[ptr + 4]
        if header_len + PROTOCOL_MIN_LEN > cnt:
            return "DATA_LEN_ERR"

        crc_calculated = self.crc_calc(data[ptr + 2 : ptr + 2 + header_len + 3])
        crc_received = struct.unpack_from("<H", data, ptr + 2 + header_len + 3)[0]

        if crc_calculated != crc_received:
            return "CRC_ERR"

        self.msg_idx["end_idx"] = ptr + header_len + PROTOCOL_MIN_LEN - 1
        result["tid"] = struct.unpack_from("<H", data, ptr + 2)[0]

        payload_start = ptr + 5
        payload_len = header_len
        while payload_len > 0:
            payload_info = self.parse_payload(data[payload_start:], result)
            if payload_info is None:
                payload_start += 1
                payload_len -= 1
            else:
                data_len = payload_info["data_len"] + 2
                payload_start += data_len
                payload_len -= data_len

        return "ANALYSIS_OK"

    def parse_payload(self, payload: bytes, result: dict):
        if len(payload) < 2:
            return None

        data_id = payload[0]
        data_len = payload[1]
        data = payload[2 : 2 + data_len]

        if data_id == SENSOR_TEMP_ID:
            result["sensor_temp"] = (
                struct.unpack_from("<h", data)[0] * SENSOR_TEMP_FACTOR
            )
        elif data_id == ACCEL_ID:
            result["acc"] = [
                struct.unpack_from("<i", data, offset)[0] * ACC_DATA_FACTOR
                for offset in range(0, 12, 4)
            ]
        elif data_id == GYRO_ID:
            result["gyro"] = [
                struct.unpack_from("<i", data, offset)[0] * GYRO_DATA_FACTOR
                for offset in range(0, 12, 4)
            ]
        elif data_id == MAGNETIC_NORM_ID:
            result["mag_norm"] = [
                struct.unpack_from("<i", data, offset)[0] * MAG_NORM_DATA_FACTOR
                for offset in range(0, 12, 4)
            ]
        elif data_id == RAW_MAGNETIC_ID:
            result["mag_raw"] = [
                struct.unpack_from("<i", data, offset)[0] * MAG_RAW_DATA_FACTOR
                for offset in range(0, 12, 4)
            ]
        elif data_id == EULER_ID:
            result["euler"] = [
                struct.unpack_from("<i", data, offset)[0] * EULER_DATA_FACTOR
                for offset in range(0, 12, 4)
            ]
        elif data_id == QUATERNION_ID:
            result["quat"] = [
                struct.unpack_from("<i", data, offset)[0] * QUATENION_DATA_FACTOR
                for offset in range(0, 16, 4)
            ]
        elif data_id == LOCATION_ID:
            result["location"] = {
                "latitude": struct.unpack_from("<q", data)[0]
                * HIGH_RES_LONG_LAT_DATA_FACTOR,
                "longitude": struct.unpack_from("<q", data, 8)[0]
                * HIGH_RES_LONG_LAT_DATA_FACTOR,
                "altitude": struct.unpack_from("<i", data, 16)[0] * ALT_DATA_FACTOR,
            }
        elif data_id == SPEED_ID:
            result["speed"] = [
                struct.unpack_from("<i", data, offset)[0] * VEL_DATA_FACTOR
                for offset in range(0, 12, 4)
            ]
        elif data_id == STATUS_ID:
            result["status"] = data[0]
        elif data_id == SAMPLE_TIMESTAMP_ID:
            result["sample_timestamp"] = struct.unpack_from("<I", data)[0]
        elif data_id == DATA_READY_TIMESTAMP_ID:
            result["data_ready_timestamp"] = struct.unpack_from("<I", data)[0]
        else:
            return None

        return {"data_id": data_id, "data_len": data_len}

    def msg_idx_obt(self):
        return self.msg_idx


def main():
    imu = YesenseDecoder()  # Replace with your actual class
    port = "/dev/ttyACM0"
    baudrate = 460800
    timeout = 1.0
    consecutive_errors = 0
    last_time = time.time()

    try:
        with serial.Serial(port, baudrate, timeout=timeout) as ser:
            ser.flushInput()
            while True:
                data = imu.read_from_uart_(ser, 1024)
                if isinstance(data, dict):
                    tid = data.get("tid")
                    acc = data.get("acc")
                    gyro = data.get("gyro")
                    euler = data.get("euler")
                    quat = data.get("quat")
                    print(
                        f"\ntid >> {tid}"
                        f"\nacc >> {acc}"
                        f"\ngyro >> {gyro}"
                        f"\neuler >> {euler}"
                        f"\nquat >> {quat}"
                    )
                    consecutive_errors = 0
                    # Process data as needed
                elif data == "BUF_FULL":
                    consecutive_errors += 1
                    if consecutive_errors >= 3:
                        print("Too many buffer full errors")
                        consecutive_errors = 0
                        # pass
                        break
                        # pass
                elif "Error" in data:
                    print(data)
                    # pass
                    # break

                current_time = time.time()
                elapsed_time = (current_time - last_time) * 1000
                imu.timing_count += elapsed_time

                if imu.timing_count >= CNT_PER_SECOND:
                    imu.msg_rate = imu.msg_count
                    imu.msg_count = 0
                    imu.timing_count = 0
                    print(f"Message Rate: {imu.msg_rate} msgs/sec")

                last_time = current_time
    except serial.SerialException as e:
        print(f"UART Error: {e}")


if __name__ == "__main__":
    main()
decoder = YesenseDecoder()
decoder.read_from_uart("COM10", 460800)
