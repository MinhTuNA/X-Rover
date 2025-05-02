from lib.ibus import IBusBM
from PySide6.QtCore import QObject,Signal,QTimer
import signal
import time


class FS_I6(QObject):
    ch0_signal = Signal(int)
    ch1_signal = Signal(int)
    ch2_signal = Signal(int)
    ch3_signal = Signal(int)
    vra_signal = Signal(int)
    vrb_signal = Signal(int)
    swa_signal = Signal(int)
    swb_signal = Signal(int)
    swc_signal = Signal(int)
    swd_signal = Signal(int)
    def __init__(self,port):
        super().__init__()
        self.ibus = IBusBM(port)
    
    def init_ibus(self):
        self.ibus.reset_ibus_port()
        time.sleep(3)
        self.ibus.run_ibus()
        self.timer_loop = QTimer()
        self.timer_loop.timeout.connect(self.ibus.loop)
        self.timer_loop.start(50)
        self.timer_read_chanel = QTimer()
        self.timer_read_chanel.timeout.connect(self.read_chanel)
        self.timer_read_chanel.start(50)

    def read_chanel(self):
        ch0 = self.ibus.readChannel(0)
        ch1 = self.ibus.readChannel(1)
        ch2 = self.ibus.readChannel(2)
        ch3 = self.ibus.readChannel(3)
        vra = self.ibus.readChannel(4)
        vrb = self.ibus.readChannel(5)
        swa = self.ibus.readChannel(6)
        swb = self.ibus.readChannel(7)
        swc = self.ibus.readChannel(8)
        swd = self.ibus.readChannel(9)
        # print(f"FS_I6: >> ch0: {ch0}")
        # print(f"FS_I6: >> ch1: {ch1}")
        # print(f"FS_I6: >> ch2: {ch2}")
        # print(f"FS_I6: >> ch3: {ch3}")
        # print(f"FS_I6: >> vra: {vra}")
        # print(f"FS_I6: >> vrb: {vrb}")
        # print(f"FS_I6: >> swa: {swa}")
        # print(f"FS_I6: >> swb: {swb}")
        # print(f"FS_I6: >> swc: {swc}")
        # print(f"FS_I6: >> swd: {swd}")
        
        self.ch0_signal.emit(ch0)
        self.ch1_signal.emit(ch1)
        self.ch2_signal.emit(ch2)
        self.ch3_signal.emit(ch3)
        self.vra_signal.emit(vra)
        self.vrb_signal.emit(vrb)
        self.swa_signal.emit(swa)
        self.swb_signal.emit(swb)
        self.swc_signal.emit(swc)
        self.swd_signal.emit(swd)

    def clean_up(self):
        
        self.timer_loop.stop()
        self.timer_read_chanel.stop()
        self.ibus.ibus_destroy()
        print(f"FS_I6: >> Closed FS_I6 {self.ibus.ibus_port}")
