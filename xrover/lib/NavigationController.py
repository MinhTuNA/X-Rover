import math
from PySide6.QtCore import QObject, QTimer
from PySide6.QtWidgets import QApplication
from .SerialDeviceScanner import DevicePortScanner
from .ModbusDevice import Driver
from .ConstVariable import COMMON, WHEEL
import os
import sys
import signal

os.environ["QT_QPA_PLATFORM"] = "offscreen"

class NavigationController(QObject):

    def __init__(self,port = None):
        super().__init__()
        self.scanner = DevicePortScanner()
        self.rs485_port = port
        self.driver = Driver(self.rs485_port)
        self.linear_velocity = 0
        self.angular_velocity = 0
        self.ratio = WHEEL.ratio
        self.kp = COMMON.motorKp
        self.ki = COMMON.motorKi
        self.kd = COMMON.motorKd
        self.wheel_radius = COMMON.wheel_radius
        self.wheel_base = COMMON.wheel_base
        self.mode = WHEEL.speed_mode

    def wheel_speeds(self, linear_velocity, angular_velocity):
        omega_left = (
            linear_velocity + (angular_velocity * self.wheel_base / 2)
        ) / self.wheel_radius
        omega_right = (
            linear_velocity - (angular_velocity * self.wheel_base / 2)
        ) / self.wheel_radius
        return omega_left, omega_right

    "RPM"

    def motor_speeds(self, omega_left, omega_right):
        f_left = (omega_left / (2 * math.pi)) * 60
        f_right = (omega_right / (2 * math.pi)) * 60
        return f_left, f_right
    
    def set_rpm(self, left_rpm=None, right_rpm=None):
        if left_rpm is None or right_rpm is None:
            return
        left_rpm = left_rpm*self.ratio
        right_rpm = right_rpm*self.ratio*-1
        # print(f"Navigation [DRIVER]: >> left_rpm: {int(left_rpm)} | right_rpm: {int(right_rpm)}")
        self.driver.set_motor(
            left_rpm=int(left_rpm),
            right_rpm=int(right_rpm),
            left_torque=WHEEL.torque,
            right_torque=WHEEL.torque,
            left_mode=WHEEL.speed_mode,
            right_mode=WHEEL.speed_mode,
        )
    
    def set_velocity(self, linear_velocity=None, angular_velocity=None):
        if linear_velocity is None or angular_velocity is None:
            return
        self.linear_velocity = linear_velocity
        self.angular_velocity = angular_velocity
        if self.mode == WHEEL.speed_mode:
            if self.linear_velocity == 0 and self.angular_velocity == 0:
                self.driver.set_motor(
                    left_rpm=0,
                    right_rpm=0,
                    left_torque=150,
                    right_torque=150,
                    right_mode=WHEEL.speed_mode,
                    left_mode=WHEEL.speed_mode,
                )
                # print(f"Navigation [DRIVER]: >> left_rpm: {0} | right_rpm: {0}")
                

            elif self.linear_velocity != 0 or self.angular_velocity != 0:
                omega_left, omega_right = self.wheel_speeds(
                    linear_velocity=self.linear_velocity,
                    angular_velocity=self.angular_velocity,
                )
                f_left, f_right = self.motor_speeds(
                    omega_left=omega_left, omega_right=omega_right
                )
                f_left = round(f_left, 4) * self.ratio
                f_right = round(f_right, 4) * self.ratio*-1
                print(f"Navigation [DRIVER]: >> left_rpm: {int(f_left)} | right_rpm: {int(f_right)}")
                self.driver.set_motor(
                    left_rpm=int(f_left),
                    right_rpm=int(f_right),
                    right_torque=WHEEL.torque,
                    left_torque=WHEEL.torque,
                    left_mode=WHEEL.speed_mode,
                    right_mode=WHEEL.speed_mode,
                )
            else:
                print("Navigation [DRIVER]: >> invalid velocity")
        elif self.mode == WHEEL.torque_mode:
            pass
    
    def clean_up(self):
        self.driver.clean_up()
        

    @staticmethod
    def compute_twist_stanley(
        start_lat,
        start_lon,
        current_lat,
        current_lon,
        target_lat,
        target_lon,
        current_heading,
        linear_vel_x = 0.3,
        k=1.0,
        epsilon=1e-5,
        max_angular_z=0.5,
    ):
        if current_heading < 0 or current_heading > 360:
          print("invalid heading")
          return
        
        ref_lat, ref_lon = start_lat, start_lon
        start_x, start_y = NavigationController.latlon_to_local(start_lat, start_lon, ref_lat, ref_lon)
        current_x, current_y = NavigationController.latlon_to_local(current_lat, current_lon, ref_lat, ref_lon)
        target_x, target_y = NavigationController.latlon_to_local(target_lat, target_lon, ref_lat, ref_lon)
        
        delta_east = target_x - start_x 
        delta_north = target_y - start_y  

        desired_bearing = (math.degrees(math.atan2(delta_east, delta_north)) + 360) % 360
        desired_heading_rad = math.radians(desired_bearing)
        current_heading_rad = math.radians(current_heading)
        heading_error = desired_heading_rad - current_heading_rad
        heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

        AB_x = target_x - start_x
        AB_y = target_y - start_y
        
        AC_x = current_x - start_x
        AC_y = current_y - start_y
        
        lambda_val = (AC_x * AB_x + AC_y * AB_y) / (AB_x**2 + AB_y**2)
        proj_x = start_x + lambda_val * AB_x
        proj_y = start_y + lambda_val * AB_y
        
        error_distance = math.hypot(current_x - proj_x, current_y - proj_y)
        cross = AB_x * AC_y - AB_y * AC_x
        sign = 1.0 if cross > 0 else -1.0 if cross < 0 else 0.0
        cross_track_error = sign * error_distance
        
        dist_to_target = math.hypot(target_x - current_x, target_y - current_y)
        
        if lambda_val >= 1 or dist_to_target < 0.15:
            linear_x = 0.0
            angular_z = 0.0
            return linear_x, angular_z
        
        v = linear_vel_x
        
        steering_angle = heading_error + math.atan2(k * cross_track_error, v + epsilon)

        linear_x = v
        angular_z = steering_angle
        if angular_z >= max_angular_z: 
            angular_z = max_angular_z

        return round(linear_x, 2), round(angular_z, 2)
    
    
    @staticmethod
    def latlon_to_local(lat, lon, ref_lat, ref_lon):
        """
        Chuyển đổi tọa độ lat/lon sang hệ tọa độ local (x, y) sử dụng phép chiếu equirectangular.
        Với:
            x = R * Δlon * cos(ref_lat)
            y = R * Δlat
        Đầu ra tính theo mét.
        """
        lat, lon, ref_lat, ref_lon = map(float, (lat, lon, ref_lat, ref_lon))

        R = 6371000  # bán kính Trái Đất (m)
        x = R * math.radians(lon - ref_lon) * math.cos(math.radians(ref_lat))
        y = R * math.radians(lat - ref_lat)
        return x, y

    @staticmethod
    def haversine(lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = (
            math.sin(delta_phi / 2.0) ** 2
            + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2
        )
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c
    
    @staticmethod
    def ecef_to_geodetic(X, Y, Z):
        """
        Chuyển đổi tọa độ ECEF sang tọa độ địa lý (vĩ độ, kinh độ, độ cao) theo chuẩn WGS84.
        
        Đầu vào:
            X, Y, Z: Tọa độ ECEF (mét)
        Đầu ra:
            (lat_deg, lon_deg, h): (Vĩ độ, kinh độ tính theo độ, độ cao (mét)
        """
        # Tham số elipsoid WGS84
        a = 6378137.0
        f = 1 / 298.257223563
        e_sq = f * (2 - f)
        b = a * (1 - f)  # bán kính theo trục phụ

        # Tính khoảng cách từ trục Z (trên mặt phẳng XY)
        p = math.sqrt(X * X + Y * Y)

        # Tính kinh độ
        lon = math.atan2(Y, X)

        # Ước lượng ban đầu cho vĩ độ sử dụng công thức Bowring
        theta = math.atan2(Z * a, p * b)
        lat = math.atan2(Z + e_sq * b * math.sin(theta)**3,
                        p - e_sq * a * math.cos(theta)**3)

        # Tính bán kính cong N và độ cao ban đầu
        N = a / math.sqrt(1 - e_sq * math.sin(lat)**2)
        h = p / math.cos(lat) - N

        # Lặp lại cho đến khi hội tụ (ngưỡng sai số 1e-12 rad, tương đương cm)
        tol = 1e-12  
        lat_prev = 0.0
        while abs(lat - lat_prev) > tol:
            lat_prev = lat
            N = a / math.sqrt(1 - e_sq * math.sin(lat)**2)
            h = p / math.cos(lat) - N
            lat = math.atan2(Z, p * (1 - e_sq * N / (N + h)))

        # Chuyển đổi kết quả vĩ độ và kinh độ từ radian sang độ
        lat_deg = math.degrees(lat)
        lon_deg = math.degrees(lon)
        
        return lat_deg, lon_deg, h
    
    @staticmethod
    def geodetic_to_ecef(lat_deg, lon_deg, h):
        """
        Chuyển đổi tọa độ địa lý (vĩ độ, kinh độ, độ cao) theo chuẩn WGS84 sang tọa độ ECEF.
        
        Đầu vào:
            lat_deg: Vĩ độ (độ)
            lon_deg: Kinh độ (độ)
            h: Độ cao (mét)
        Đầu ra:
            (X, Y, Z): Tọa độ ECEF (mét)
        """
        # Tham số elipsoid WGS84
        a = 6378137.0               # Bán kính theo trục chính (m)
        f = 1 / 298.257223563       # Độ dẹt của elipsoid
        e_sq = f * (2 - f)          # Bình phương độ lệch tâm

        # Chuyển đổi vĩ độ và kinh độ từ độ sang radian
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)

        # Tính bán kính cong (radius of curvature) theo phương ngang
        N = a / math.sqrt(1 - e_sq * math.sin(lat)**2)

        X = (N + h) * math.cos(lat) * math.cos(lon)
        Y = (N + h) * math.cos(lat) * math.sin(lon)
        Z = (N * (1 - e_sq) + h) * math.sin(lat)

        return X, Y, Z

    @staticmethod
    def calculate_bearing(lat1, lon1, lat2, lon2):
        dLon = math.radians(lon2 - lon1)
        lat1, lat2 = math.radians(lat1), math.radians(lat2)
        x = math.sin(dLon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(
            lat2
        ) * math.cos(dLon)
        bearing = math.atan2(x, y)
        return (math.degrees(bearing) + 180) % 360 - 180
    
def main():
    app = QApplication(sys.argv)
    nav = NavigationController()
    nav.set_velocity(linear_velocity=0.5, angular_velocity=0)
    def handle_destroy(signum, frame):
        print("Navigation: >> Closing...")
        nav.clean_up()
        app.quit()

    signal.signal(signal.SIGINT, handle_destroy)
    app.exec()
    
if __name__ == "__main__":
    main()
