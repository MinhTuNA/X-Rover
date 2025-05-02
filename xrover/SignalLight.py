from lib.SignalLightLib import SignalLightLib
from PySide6.QtCore import QObject, QTimer
import sys
from PySide6.QtWidgets import QApplication
import signal
import os

os.environ["QT_QPA_PLATFORM"] = "offscreen"

class SignalLight(QObject):
    def __init__(self):
        super().__init__()
        self.on = 0
        self.off = 1
        self.signal_light_lib = SignalLightLib()
        self.signal_light_lib.set_red(self.off)
        self.signal_light_lib.set_yellow(self.off)
        self.signal_light_lib.set_green(self.off)
        self.signal_light_lib.set_buzz(self.off)
        
    def red(self, state: bool):
        if state:
            self.signal_light_lib.set_red(self.on)
        else:
            self.signal_light_lib.set_red(self.off)

    def yellow(self, state: bool):
        if state:
            self.signal_light_lib.set_yellow(self.on)
        else:
            self.signal_light_lib.set_yellow(self.off)

    def green(self, state: bool):
        if state:
            self.signal_light_lib.set_green(self.on)
        else:
            self.signal_light_lib.set_green(self.off)

    def buzz(self, state: bool):
        if state:
            self.signal_light_lib.set_buzz(self.on)
        else:
            self.signal_light_lib.set_buzz(self.off)

    def clean_up(self):
        self.signal_light_lib.cleanup()


def main():
    app = QApplication(sys.argv)
    signal_light = SignalLight()
    signal_light.red(True)
    signal_light.yellow(True)
    signal_light.green(True)
    signal_light.buzz(True)
    
    timer = QTimer()
    timer.start(100)  # mỗi 100ms
    timer.timeout.connect(lambda: None)

    def handle_sigint(sig, frame):
        signal_light.clean_up()
        app.quit()
    signal.signal(signal.SIGINT, handle_sigint)
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
