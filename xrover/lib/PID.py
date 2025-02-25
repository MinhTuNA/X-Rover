import math


class PIDController:
    def __init__(self, kp, ki, kd, output_limits=(None, None)):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.output_limits = output_limits

    def compute(self, error, dt):
        if dt <= 0:
            dt = 1e-6
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        P = self.kp * error
        I = self.ki * self.integral
        D = self.kd * derivative
        self.prev_error = error
        output = P + I + D
        if self.output_limits[0] is not None:
            output = max(self.output_limits[0], output)
        if self.output_limits[1] is not None:
            output = min(self.output_limits[1], output)
        return output

    def reset(self):
        self.prev_error = 0
        self.integral = 0
