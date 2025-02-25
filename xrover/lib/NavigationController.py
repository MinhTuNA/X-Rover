import math
from geometry_msgs.msg import Twist
from .ConstVariable import COMMON


class NavigationController:

    @staticmethod
    def compute_twist(current_lat, current_lon, target_lat, target_lon, current_heading):
        """
        Tính toán lệnh vận tốc dựa trên vị trí hiện tại, mục tiêu và hướng robot.
        :param current_lat: Vĩ độ hiện tại
        :param current_lon: Kinh độ hiện tại
        :param target_lat: Vĩ độ mục tiêu
        :param target_lon: Kinh độ mục tiêu
        :param current_heading: Hướng hiện tại của robot (độ)
        :return: geometry_msgs/Twist chứa lệnh vận tốc
        """
        twist = Twist()
        distance_to_target = NavigationController.haversine(
            current_lat, current_lon, target_lat, target_lon)
        target_bearing = NavigationController.calculate_bearing(
            current_lat, current_lon, target_lat, target_lon)
        angle_error = target_bearing - current_heading
        angle_error = (angle_error + 180) % 360 - 180

        Kp_angular = 0.02
        Kp_linear = 0.5
        max_angular_speed = 0.5
        max_linear_speed = 1
        distance_threshold = COMMON.distance_threshold
        angle_threshold = COMMON.angle_threshold
        if abs(angle_error) > angle_threshold:
            twist.angular.z = max(
                -max_angular_speed,
                min(
                    Kp_angular * math.radians(angle_error),
                    max_angular_speed
                )
            )
            twist.linear.x = 0.0
        elif distance_to_target > distance_threshold:
            twist.angular.z = max(
                -max_angular_speed,
                min(
                    Kp_angular * math.radians(angle_error),
                    max_angular_speed
                )
            )
            twist.linear.x = max(
                0.0,
                min(
                    Kp_linear * distance_to_target,
                    max_linear_speed
                )
            )
        else:
            twist.angular.z = 0.0
            twist.linear.x = 0.0  # Đã đến đích, dừng robot
        return twist

    @staticmethod
    def haversine(lat1, lon1, lat2, lon2):
        R = 6371000
        phi1, phi2 = math.radians(lat1), math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        a = (math.sin(delta_phi / 2.0) ** 2 +
             math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        return R * c

    @staticmethod
    def calculate_bearing(lat1, lon1, lat2, lon2):
        dLon = math.radians(lon2 - lon1)
        lat1, lat2 = math.radians(lat1), math.radians(lat2)
        x = math.sin(dLon) * math.cos(lat2)
        y = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * \
            math.cos(lat2) * math.cos(dLon)
        bearing = math.atan2(x, y)
        return (math.degrees(bearing) + 180) % 360 - 180
