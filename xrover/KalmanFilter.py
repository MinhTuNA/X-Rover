import math
import time
class KalmanFilter:
    def __init__(self, process_noise=0.1, measurement_noise=1.0):
        self.angle = 0.0
        self.P = 1.0 
        self.Q = process_noise
        self.R = measurement_noise 

    def normalize_angle(self, angle):
        return (angle + 180) % 360 - 180

    def predict(self, gyro_rate, dt):
        self.angle += gyro_rate * dt
        self.angle = self.normalize_angle(self.angle)
        self.P += self.Q

    def update(self, measured_angle):
        angle_error = self.normalize_angle(measured_angle - self.angle)
        K = self.P / (self.P + self.R)
        self.angle += K * angle_error
        self.angle = self.normalize_angle(self.angle)
        self.P *= (1 - K)

    def get_angle(self):
        return self.angle

    def calculate_accel_angle(ax, ay, az):
        return math.degrees(math.atan2(ay, az))
   