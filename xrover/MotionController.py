#!/usr/bin/env python3
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from .PathPlaner import PathPlaner
from .Const import *
import json
from .KalmanFilter import KalmanFilter
class MotionController:
    def __init__(self, node, sensor_manager):
        self.kalman_filter = KalmanFilter(process_noise=0.1, measurement_noise=0.5)
        self.node = node
        self.sensor_manager = sensor_manager
        self.error_integral = 0
        self.previous_angle_error = 0
        self.last_twist = Twist()
        self.cmd_vel_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.status_publisher = self.create_publisher(String, "/status", 10)
        self.last_time = self.node.get_clock().now()
        
    def compute_twist(self, target_lat, target_lon):
        current_time = self.node.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9 
        self.last_time = current_time
        gyro_rate = self.sensor_manager.gyro_rate[2]
        compass_angle = self.sensor_manager.compass_heading
        ax = self.sensor_manager.accel_angle[0]
        ay = self.sensor_manager.accel_angle[1]
        az = self.sensor_manager.accel_angle[2]
        accel_angle = KalmanFilter.calculate_accel_angle(ax, ay, az)
        
        distance = PathPlaner.haversine(self.sensor_manager.current_lat, self.sensor_manager.current_lon, target_lat, target_lon)
        target_bearing = PathPlaner.calculate_bearing(self.sensor_manager.current_lat, self.sensor_manager.current_lon, target_lat, target_lon)
        
        self.kalman_filter.predict(gyro_rate, dt)
        combined_angle = (compass_angle + accel_angle) / 2
        self.kalman_filter.update(combined_angle)
        current_angle = self.kalman_filter.get_angle()
        
        angle_error = target_bearing - current_angle
        angle_error = (angle_error + 180) % 360 - 180
        
        # PID
        self.error_integral += angle_error
        error_derivative = angle_error - self.previous_angle_error
        self.previous_angle_error = angle_error
        twist = Twist()
        if distance > distance_threshold:    
            if abs(angle_error) > angle_threshold:
                twist.linear.x = 0.0
                twist.angular.z = Kp * angle_error + Ki * self.error_integral + Kd * error_derivative
            else:
                twist.linear.x = max(0.05, min(rover_speed, distance / distance_threshold * rover_speed))
                twist.angular.z = Kp * angle_error + Ki * self.error_integral + Kd * error_derivative
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.send_status("done")

        return twist
    
    def publish_twist(self, twist):
        if twist and (twist.linear.x != self.last_twist.linear.x or twist.angular.z != self.last_twist.angular.z):
            self.cmd_vel_pub.publish(twist)
            self.last_twist = twist
    
    def send_status(self, status):
        cmd = {
            "status": status,
        }
        msg_json = json.dumps(cmd)
        msg = String()
        msg.data = msg_json
        self.status_publisher.publish(msg)