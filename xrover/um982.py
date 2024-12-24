import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32
import serial
import re
import numpy as np


class UM982Node(Node):
    def __init__(self, uart_port="/dev/ttyUSB1", baudrate=115200):
        super().__init__("um982_node")
        self.buffer = ""
        self.gps_publisher = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.heading_publisher = self.create_publisher(Float32, "/compass/heading", 10)

        self.uart_port = uart_port
        self.baudrate = baudrate
        self.get_logger().info(
            f"Connecting to UART at {uart_port} with baudrate {baudrate}"
        )
        try:
            self.serial_port = serial.Serial(
                port=self.uart_port, baudrate=self.baudrate, timeout=1
            )
            self.get_logger().info("Successfully connected to UART")
        except Exception as e:
            self.get_logger().error(f"Failed to open UART port: {e}")
            raise

        self.uart_timer = self.create_timer(0.1, self.read_uart_and_publish)

    def read_uart_and_publish(self):
        try:
            # Đọc dữ liệu từ UART
            data = self.serial_port.readline()
            if data and b"\x00" not in data:
                um982_data = data.decode("utf-8", errors="ignore").strip()
                # self.get_logger().info(um982_data)
                self.parse_um982_data(um982_data)
        except Exception as e:
            self.get_logger().error(f"Error while reading from UART: {e}")

    def parse_um982_data(self, data):
        hash_messages = None
        dollar_messages = None
        if data.startswith("#"):
            hash_messages = data
            parsed_msg = re.split(r"[,;]", hash_messages)

            # self.get_logger().info(f"{parsed_msg}")
            if parsed_msg[0] == "#RTKSTATUSA":
                rtk_status = self.parse_rtkstatus(parsed_msg)
                if rtk_status is not None:
                    status = rtk_status.get("position_type")
                    self.get_logger().info(f"status >> {status}")

            # if(parsed_msg[0] == '#BESTNAVXYZA'):
            #     lat,lon,alt = self.get_coordinates(parsed_msg)
            #     self.get_logger().info(f"\nRTK \nlat >> {lat} \nlon >> {lon} \nalt >> {alt} ")
            if parsed_msg[0] == "#ADRNAVA":
                lat = self.parse_adrnav(parsed_msg).get("latitude")
                lon = self.parse_adrnav(parsed_msg).get("longitude")
                height = self.parse_adrnav(parsed_msg).get("height")
                # self.get_logger().info(str(parsed_msg))
                self.get_logger().info(
                    f"\nRTK \nlat >> {lat} \nlon >> {lon} \nalt >> {height} "
                )
                self.publish_coordinates_data(lat=lat, lon=lon, alt=height)
            if parsed_msg[0] == "#UNIHEADINGA":
                heading = self.parse_uniheading(parsed_msg).get("heading")
                self.get_logger().info(f"heading >> {heading} ")
                self.publish_heading(heading=heading)
            if parsed_msg[0] == "#RTCMSTATUSA":
                self.get_logger().info(f"{parsed_msg}")
                # rctm_status = self.parse_rtcmstatus(parsed_msg)
                # if(rctm_status is not None):
                #     status = rctm_status.get('l4_num')
                #     self.get_logger().info(f"RCTM: {status}")

        elif data.startswith("$"):
            dollar_messages = data
            parsed_msg = re.split(r"[,;*]", dollar_messages)
            # self.get_logger().info(f"Message: {msg}")
            # self.get_logger().info(f"{parsed_msg}")
            # if(parsed_msg[0]=='$GNGLL'):
            #     lat,lon = self.parse_gps_GNGLL(parsed_msg)
            #     if lat is not None and lon is not None:
            #         self.get_logger().info(f"\nGNGLL: \nlat >> {lat} \nlon >> {lon}")

    def publish_coordinates_data(self, lat=None, lon=None, alt=None):
        gps_msg = NavSatFix()
        gps_msg.latitude = lat
        gps_msg.longitude = lon
        gps_msg.altitude = alt
        self.gps_publisher.publish(gps_msg)

    def publish_heading(self, heading=None):
        heading_msg = Float32()
        heading_msg.data = heading
        self.heading_publisher.publish(heading_msg)

    def get_coordinates(self, data):
        if len(data) > 14:
            x, y, z = map(float, (data[12], data[13], data[14]))
            return self.ecef_to_latlon(x, y, z)
        return None

    def ecef_to_latlon(self, x: float, y: float, z: float):
        """
        Chuyển đổi tọa độ ECEF (x, y, z) sang latitude, longitude, altitude (hệ WGS84).

        Args:
            x (float): Tọa độ X (mét)
            y (float): Tọa độ Y (mét)
            z (float): Tọa độ Z (mét)

        Returns:
            tuple: (latitude, longitude, altitude) trong đơn vị (độ, độ, mét)
        """
        # Hằng số của elipsoid WGS84
        a = 6378137.0
        e = 8.1819190842622e-2

        lon = np.arctan2(y, x)

        r = np.sqrt(x**2 + y**2)
        lat = np.arctan2(z, r)

        for _ in range(10):
            N = a / np.sqrt(1 - e**2 * np.sin(lat) ** 2)
            lat = np.arctan2(z + e**2 * N * np.sin(lat), r)

        N = a / np.sqrt(1 - e**2 * np.sin(lat) ** 2)
        alt = r / np.cos(lat) - N

        lat_deg = np.degrees(lat)
        lon_deg = np.degrees(lon)

        return lat_deg, lon_deg, alt

    def parse_gps_GNGLL(self, data):
        if len(data) < 5:
            return None

        try:
            # Lấy dữ liệu Latitude
            lat_deg = float(data[1][:2])
            lat_min = float(data[1][2:])
            latitude = lat_deg + lat_min / 60
            if data[2] == "S":
                latitude = -latitude

            # Lấy dữ liệu Longitude
            lon_deg = float(data[3][:3])
            lon_min = float(data[3][3:])
            longitude = lon_deg + lon_min / 60
            if data[4] == "W":
                longitude = -longitude

            return latitude, longitude

        except (ValueError, IndexError):
            return None

    def parse_rtkstatus(self, data):
        if len(data) >= 21:
            # Extract values based on their positions in the data list
            result = {
                "header": data[0],
                "message_length": int(data[1]),
                "gnss_system": data[2],
                "solution_status": data[3],
                "time_of_week": int(data[4]),
                "gps_time": data[5],  # GPS time as string (not converted to int)
                "reserved1": int(data[6]),
                "reserved2": int(data[7]),
                "satellites_tracked": int(data[8]),
                "satellites_used": int(data[9]),
                "gps_source": data[10],
                "reserved3": data[11],
                "bds_source1": data[12],
                "bds_source2": data[13],
                "reserved4": data[14],
                "glo_source": data[15],
                "reserved5": data[16],
                "reserved6": data[17],
                "reserved7": data[18],
                "reserved8": data[19],
                "reserved9": data[20],
                "position_type": data[21],
            }
            # if len(data) > 21:
            #     result["calculate_status"]= int(data[22]) if data[22].isdigit() else data[22]  # Handle string or int
            #     result["ion_detected"]= data[23]  # Not a numerical value
            #     result["dual_rtk_flag"]= data[24]
            #     result["reserved10"] = int(data[25])
            # if len(data) > 26:
            #     result["checksum"] = data[26]
            return result
        return None

    def parse_adrnav(self, data):
        if len(data) >= 39:
            # Extract values based on their positions in the data list
            result = {
                "header": data[0],
                "message_length": int(data[1]),
                "gnss_system": data[2],
                "solution_status": data[3],
                "time_of_week": int(data[4]),
                "gps_time": int(data[5]),
                "reserved1": int(data[6]),
                "reserved2": int(data[7]),
                "satellites_tracked": int(data[8]),
                "satellites_used": int(data[9]),
                "solution_computation": data[10],
                "position_type": data[11],
                "latitude": float(data[12]),
                "longitude": float(data[13]),
                "height": float(data[14]),
                "undulation": float(data[15]),
                "datum": data[16],
                "lat_std_dev": float(data[17]),
                "lon_std_dev": float(data[18]),
                "height_std_dev": float(data[19]),
                "base_station_id": data[20],
                "differential_age": float(data[21]),
                "solution_age": float(data[22]),
                "#svs": int(data[23]),
                "#solnsvs": int(data[24]),
                "reserved3": int(data[25]),
                "reserved4": int(data[26]),
                "reserved5": int(data[27]),
                "reserved6": int(data[28]),
                "galileo_bds3_signal_mask": data[29],
                "gps_glonass_bds2_signal_mask": data[30],
                "solution_status_2": data[31],
                "position_type2": data[32],
                "horizontal_speed": float(data[33]),
                "vertical_speed": float(data[34]),
                "vertical_speed_std_dev": float(data[35]),
                "track_ground": float(data[36]),
                "checksum": data[38],
            }
            return result
        return None

    def parse_uniheading(self, data):
        if len(data) > 15:
            # Extract values based on their positions in the data list
            result = {
                "header": data[0],
                "message_length": int(data[1]),
                "gnss_system": data[2],
                "solution_status": data[3],
                "time_of_week": int(data[4]),
                "gps_time": int(data[5]),
                "reserved1": int(data[6]),
                "reserved2": int(data[7]),
                "satellites_tracked": int(data[8]),
                "satellites_used": int(data[9]),
                "solution_computation": data[10],
                "position_type": data[11],
                "baseline_length": float(data[12]),
                "heading": float(data[13]),
                "pitch": float(data[14]),
                "roll": float(data[15]),
                "heading_std_dev": float(data[16]),
                "pitch_std_dev": float(data[17]),
                "base_station_id": data[18],
                "age_of_corrections": int(data[19]),
                "observations": int(data[20]),
                "multi_l2": int(data[21]),
                "reserved_field1": int(data[22]),
                "reserved_field2": int(data[23]),
                "extended_solution_status": int(data[24]),
                "software_version": int(data[25]),
                "checksum": data[26],
            }
            return result
        return None

    def parse_rtcmstatus(self, data):
        if len(data) >= 20:
            result = {
                "message": data[0],  # Tên thông điệp (ví dụ: RTCMSTATUSA)
                "cpu_idle": int(data[1]),  # CPU idle (Uchar)
                "time_ref": int(
                    data[2]
                ),  # Tham chiếu thời gian (Uchar: GPST hoặc BDST)
                "time_status": data[3],  # Trạng thái thời gian (Fine/Unknown)
                "gps_week_number": int(data[4]),  # Số tuần GPS (Ushort)
                "gps_seconds_of_week": int(data[5]),  # GPS seconds of week (ms) (Ulong)
                "reserved": int(data[6]),  # Trường reserved
                "version": int(data[7]),  # Phiên bản định dạng Unicore (Uchar)
                "leap_seconds": int(data[8]),  # Leap seconds (Uchar)
                "output_delay": int(data[9]),  # Output delay (Ushort)
                # RTCMSTATUS message body
                "msg_id": int(data[10]),  # ID của thông điệp MSM
                "msg_num": int(data[11]),  # Số lượng thông điệp
                "base_id": int(data[12]),  # Base station ID
                "satellite_num": int(data[13]),  # Số vệ tinh hiện tại
                "l1_num": int(data[14]),  # Số tín hiệu L1
                "l2_num": int(data[15]),  # Số tín hiệu L2
                "l3_num": int(data[16]),  # Số tín hiệu L3
                "l4_num": int(data[17]),  # Số tín hiệu L4
                "l5_num": int(data[18]),  # Số tín hiệu L5
                "l6_num": int(data[19]),  # Số tín hiệu L6
                "checksum": data[-1],  # Checksum
            }
            return result
        return None

    def destroy_node(self):
        super().destroy_node()
        self.serial_port.close()
        self.get_logger().info("UART connection closed.")


def main(args=None):
    rclpy.init(args=args)
    um982_node = UM982Node()

    try:
        rclpy.spin(um982_node)
    except KeyboardInterrupt:
        um982_node.get_logger().info("Shutting down.")
    finally:
        um982_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
