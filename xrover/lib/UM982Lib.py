import math
import re
import os


class GPSDevice:
    def __init__(self):
        self.ADRNAVA_CMD = "ADRNAVA"
        self.RTKSTATUSA_CMD = "RTKSTATUSA"
        self.UNIHEADINGA_CMD = "UNIHEADINGA"
        self.RTCMSTATUSA_CMD = "RTCMSTATUSA"
        self.BESTNAVXYZA_CMD = "BESTNAVXYZA"
        self.FRESET_CMD = "FRESET"
        self.SAVECONFIG_CMD = "SAVECONFIG"
        self.RTCM_CMD = "ONCHANGED"
        
        self.valid_keywords = [
            "#ADRNAVA",
            "#RTKSTATUSA",
            "#UNIHEADINGA",
            "#RTCMSTATUSA",
            "#BESTNAVXYZA",
        ]
        self.rtcm_status = None

    def filter_rtk_data(self, data):
        valid_keywords = self.valid_keywords
        for keyword in valid_keywords:
            index = data.find(keyword)
            if index != -1:
                return data[index:]
        return None

    def parse_um982_data(self, data):
        hash_messages = None
        dollar_messages = None
        # print(f"data >> {data}")
        data_filtered = self.filter_rtk_data(data)
        if data_filtered is None:
            return
        # print(f"data >> {data_filtered}")
        if data_filtered.startswith("#"):
            hash_messages = data_filtered
            parsed_msg = re.split(r"[;]", hash_messages)
            heading = re.split(r"[,]", parsed_msg[0])
            main_data = re.split(r"[,]", parsed_msg[1])

            if heading[0] == "#RTKSTATUSA":
                rtk_status = self.parse_rtkstatus(main_data)
                if rtk_status is None:
                    return
                return rtk_status
                # status = rtk_status["pos_type"]

            if heading[0] == "#ADRNAVA":
                adrnav_data = self.parse_adrnav(main_data)
                if adrnav_data is None:
                    return
                return adrnav_data
                # print("ADRNAV")
                # for key, value in adrnav_data.items():
                #     print(f"{key}: {value}")

            if heading[0] == "#UNIHEADINGA":
                uniheading_data = self.parse_uniheading(main_data)
                if uniheading_data is None:
                    return
                return uniheading_data
                # for key, value in uniheading_data.items():
                #     print(f"{key}: {value}")

            if heading[0] == "#RTCMSTATUSA":
                rtcm_status_data = self.parse_rtcmstatus(main_data)
                # print(f"RTCM >> {rtcm_status_data}")
                if rtcm_status_data is None:
                    return
                return rtcm_status_data
                # status = rtcm_status_data["l4_num"]
                # self.rtcm_status = status
                # self.get_logger().info(f"RCTM: {status}")
            if heading[0] == "#BESTNAVXYZA":
                best_nav_xyz_data = self.parse_bestnavxyz(main_data)
                if best_nav_xyz_data is None:
                    return
                return best_nav_xyz_data

    def ECEF_to_WGS84(self, x=None, y=None, z=None, tolerance=1e-12):
        a = 6378137.0
        e2 = 0.00669438

        # kinh độ
        _lon = math.atan2(y, x)
        # khoảng cách p trên mặt phẳng x y
        p = math.sqrt(x**2 + y**2)
        # ước tính ban đầu cho vĩ độ
        fi0 = math.atan2(z, p * (1 - e2))

        while True:
            sin_fi = math.sin(fi0)
            N = a / math.sqrt(1 - e2 * sin_fi**2)
            new_fi = math.atan2(z + e2 * N * sin_fi, p)
            if abs(new_fi - fi0) < tolerance:
                fi = new_fi
                break
            fi0 = new_fi

        N = a / math.sqrt(1 - e2 * math.sin(fi) ** 2)
        # độ cao
        alt = p / math.cos(fi) - N

        lat = math.degrees(fi)
        lon = math.degrees(_lon)

        return lat, lon, alt

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
        if len(data) == 17:
            result = {
                "type": "rtk_status_msg",
                "gps_source": data[0],
                "reserved1": data[1],
                "bds_source1": data[2],
                "bds_source2": data[3],
                "reserved2": data[4],
                "glo_source": data[5],
                "reserved3": data[6],
                "gal_source1": data[7],
                "gal_source2": data[8],
                "qzss_source": data[9],
                "reserved4": data[10],
                "pos_type": data[11],
                "calculate_status": data[12],
                "ion_detected": data[13],
                "dual_rtk_flag": data[14],
                "adr_number": data[15],
                "reserved5": data[16],
            }
            return result
        return None

    def parse_adrnav(self, data):
        if len(data) == 30:
            # Extract values based on their positions in the data list
            result = {
                "type": "adrnav_msg",
                "sol_status": data[0],
                "pos_type": data[1],
                "lat": data[2],
                "lon": data[3],
                "height": data[4],
                "undulation": data[5],
                "datum_id": data[6],
                "lat_std_dev": data[7],
                "lon_std_dev": data[8],
                "height_std_dev": data[9],
                "stn_id": data[10],
                "diff_age": data[11],
                "sol_age": data[12],
                "num_sat_tracked": data[13],
                "num_sat_used": data[14],
                "reserved0": data[15],
                "reserved1": data[16],
                "reserved2": data[17],
                "extended_sol_status": data[18],
                "galileo_bds3_sig_mask": data[19],
                "gps_glonass_bds2_sig_mask": data[20],
                "sol_status": data[21],
                "vel_type": data[22],
                "latency": data[23],
                "age": data[24],
                "hor_spd": data[25],
                "trk_gnd": data[26],
                "vert_spd": data[27],
                "verspd_std": data[28],
                "horspd_std": data[29],
            }
            return result
        return None

    def parse_uniheading(self, data):
        if len(data) == 17:
            # Extract values based on their positions in the data list
            result = {
                "type": "uniheading_msg",
                "sol_status": data[0],
                "pos_type": data[1],
                "length": data[2],
                "heading": data[3],
                "pitch": data[4],
                "reserved0": data[5],
                "hdgstddev": data[6],
                "ptchstddev": data[7],
                "base_station_id": data[8],
                "satellites_tracked": data[9],
                "satellites_used": data[10],
                "obs": data[11],
                "multi": data[12],
                "Reserved1": data[13],
                "ext_sol_stat": data[14],
                "galileo_bds3_signal_mask": data[15],
                "gps_glonass_bds2_signal_mask": data[16],
            }
            return result
        return None

    def parse_rtcmstatus(self, data):
        if len(data) >= 9:
            result = {
                "type": "rtcm_status_msg",
                "msg_id": data[0],
                "msg_num": data[1],
                "base_id": data[2],
                "state_lites_num": data[3],
                "l1_num": data[4],
                "l2_num": data[5],
                "l3_num": data[6],
                "l4_num": data[7],
                "l5_num": data[8],
                "l6_num": data[9],
            }
            return result
        return None

    def parse_bestnavxyz(self, data):
        if len(data) >= 25:
            result = {
                "type": "bestnavxyz_msg",
                "P_sol_status": data[0],  # solution status
                "pos_type": data[1],  # position type
                "P_X": data[2],  # X-coordinate of position, m
                "P_Y": data[3],  # Y-coordinate of position, m
                "P_Z": data[4],  # Z-coordinate of position, m
                "P_X_sigma": data[5],  # Standard deviation of P-X, m
                "P_Y_sigma": data[6],  # Standard deviation of P-Y, m
                "P_Z_sigma": data[7],  # Standard deviation of P-Z, m
                "V_sol_status": data[8],  # solution status
                "vel_type": data[9],  # Velocity type
                "V_X": data[10],  # Velocity along X-axis, m/s
                "V_Y": data[11],  # Velocity along Y-axis, m/s
                "V_Z": data[12],  # Velocity along Z-axis, m/s
                "V_X_sigma": data[13],  # Standard deviation of V-X, m/s
                "V_Y_sigma": data[14],  # Standard deviation of V-Y, m/s
                "V_Z_sigma": data[15],  # Standard deviation of V-Z, m/s
                "stn_id": data[16],  # Base Station ID, default = 0
                "V_latency": data[
                    17
                ],  # A measure of latency in the velocity time tag, in seconds. Subtracting latency from epoch time gives accurate velocity
                "diff_age": data[18],  # Differential age, s
                "sol_age": data[19],  # Solution age, s
                "num_sat_tracked": data[20],
                "num_sat_used": data[21],  # .....
            }
            return result
        return None
