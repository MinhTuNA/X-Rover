import Jetson.GPIO as GPIO
import time


class SignalLightLib:
    def __init__(self):
        self.red = 29
        self.yellow = 31
        self.green = 33
        self.buzz = 32
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.red, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.yellow, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.green, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.buzz, GPIO.OUT, initial=GPIO.LOW)

    def test_signal_light(self):
        GPIO.output(self.red, GPIO.HIGH)
        GPIO.output(self.yellow, GPIO.HIGH)
        GPIO.output(self.green, GPIO.HIGH)
        GPIO.output(self.buzz, GPIO.HIGH)
        print("high")
        time.sleep(1)
        GPIO.output(self.red, GPIO.LOW)
        GPIO.output(self.yellow, GPIO.LOW)
        GPIO.output(self.green, GPIO.LOW)
        GPIO.output(self.buzz, GPIO.LOW)
        print("low")
        time.sleep(1)
    
    def _validate_value(self, value):
        if value not in [GPIO.HIGH, GPIO.LOW]:
            raise ValueError("Value must be GPIO.HIGH or GPIO.LOW")

    def set_red(self, value):
        self._validate_value(value)
        GPIO.output(self.red, value)
        
    def set_yellow(self, value):
        self._validate_value(value)
        GPIO.output(self.yellow, value)
        
    def set_green(self, value):
        self._validate_value(value)
        GPIO.output(self.green, value)

    def set_buzz(self, value):
        self._validate_value(value)
        GPIO.output(self.buzz, value)
        
    def cleanup(self):
        self.set_red(0)
        self.set_yellow(0)
        self.set_green(0)
        self.set_buzz(0)
        GPIO.cleanup()
        


if __name__ == "__main__":
    signal_light_lib = SignalLightLib()
    try:
        while True:
            signal_light_lib.test_signal_light()
            
    except KeyboardInterrupt:
        signal_light_lib.cleanup()


# import gpiod
# import time


# class SignalLightLib:
#     def __init__(self):
#         self.red = 1
#         self.yellow = 0
#         self.green = 8
#         self.blue = 2

#         self.chip = gpiod.Chip("gpiochip1")
#         self.red_line = self.chip.get_line(self.red)
#         self.yellow_line = self.chip.get_line(self.yellow)
#         self.green_line = self.chip.get_line(self.green)
#         self.blue_line = self.chip.get_line(self.blue)

#         self.red_line.request(consumer="SignalLightLib", type=gpiod.LINE_REQ_DIR_OUT)
#         self.yellow_line.request(consumer="SignalLightLib", type=gpiod.LINE_REQ_DIR_OUT)
#         self.green_line.request(consumer="SignalLightLib", type=gpiod.LINE_REQ_DIR_OUT)
#         self.blue_line.request(consumer="SignalLightLib", type=gpiod.LINE_REQ_DIR_OUT)

#     def test_signal_light(self):
#         print("high")
#         self.red_line.set_value(1)
#         self.yellow_line.set_value(1)
#         self.green_line.set_value(1)
#         self.blue_line.set_value(1)
#         time.sleep(1)
#         print("low")
#         self.red_line.set_value(0)
#         self.yellow_line.set_value(0)
#         self.green_line.set_value(0)
#         self.blue_line.set_value(0)
#         time.sleep(1)


# if __name__ == "__main__":
#     signal_light_lib = SignalLightLib()
#     while True:
#         signal_light_lib.test_signal_light()
