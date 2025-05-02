import sys
from PySide6.QtWidgets import QApplication
from PySide6.QtCore import QTimer, QThread
from SocketIOInterface import SocketIOInterface
from S21C import UltrasonicSensors
from RTCMReceiver import RTCMReceiver
from lib.SerialDeviceScanner import DevicePortScanner
from GPSRTK import GPSRTK
from Delta import Delta
from FS_I6 import FS_I6
from SignalLight import SignalLight
from IMU import IMUReader
from Navigation import Navigation
from Ros2Interface import Ros2Interface,rclpy
from PathManager import PathManager
import signal
import os
import threading


os.environ["QT_QPA_PLATFORM"] = "offscreen"


if __name__ == "__main__":
    app = QApplication(sys.argv)

    scanner = DevicePortScanner()
    ports = scanner.list_serial_ports()
    gps_port = scanner.find_um982_port(ports)
    s21c_port = scanner.find_s21c_port(ports)
    rf_module_port = scanner.find_rf_module_port(ports)
    delta_port = scanner.find_delta_x_port(ports)
    fs_i6_port = scanner.find_fs_i6_port(ports)
    imu_port = scanner.find_imu_port(ports)
    rs485_port = scanner.find_rs485_port(ports)
    print(f"GPS: >> port {gps_port}")
    print(f"S21C: >> port {s21c_port}")
    print(f"RF Module: >> port {rf_module_port}")
    print(f"Delta: >> port {delta_port}")
    print(f"FS_I6: >> port {fs_i6_port}")
    print(f"IMU: >> port {imu_port}")
    print(f"RS485: >> port {rs485_port}")
    
    socket_io_interface = SocketIOInterface()
    thread_socket_io = threading.Thread(target=socket_io_interface.runSocketIO)
    thread_socket_io.daemon = True
    thread_socket_io.start()

    ultrasonic_sensors = UltrasonicSensors(s21c_port)
    ultrasonic_thread = threading.Thread(target=ultrasonic_sensors.read_uart_data)
    ultrasonic_thread.daemon = True
    ultrasonic_thread.start()
    
    rtcm_receiver = RTCMReceiver(rf_module_port)
    thread_rtcm_receiver = QThread()
    rtcm_receiver.moveToThread(thread_rtcm_receiver)
    thread_rtcm_receiver.started.connect(rtcm_receiver.init_rtcm)
    thread_rtcm_receiver.finished.connect(rtcm_receiver.clean_up)
    thread_rtcm_receiver.start()

    gps = GPSRTK(gps_port)
    gps_thread = QThread()
    gps.moveToThread(gps_thread)
    gps_thread.started.connect(gps._start_gps)
    gps_thread.finished.connect(gps.clean_up)
    gps_thread.start()

    ros2_interface = Ros2Interface()
    ros2_interface_thread = QThread()
    ros2_interface.moveToThread(ros2_interface_thread)
    ros2_interface_thread.started.connect(ros2_interface.run)
    ros2_interface_thread.finished.connect(ros2_interface.clean_up)
    ros2_interface_thread.start()

    delta = Delta(delta_port)
    delta_thread = QThread()
    delta.moveToThread(delta_thread)
    delta_thread.started.connect(delta.start_delta)
    delta_thread.finished.connect(delta.handle_destroy)
    delta_thread.start()

    fs_i6 = FS_I6(fs_i6_port)
    fs_i6_thread = QThread()
    fs_i6.moveToThread(fs_i6_thread)
    fs_i6_thread.started.connect(fs_i6.init_ibus)
    fs_i6_thread.finished.connect(fs_i6.clean_up)
    fs_i6_thread.start()

    imu = IMUReader(imu_port)
    imu_thread = QThread()
    imu.moveToThread(imu_thread)
    imu_thread.started.connect(imu.connect_imu)
    imu_thread.finished.connect(imu.clean_up)
    imu_thread.start()

    signal_light = SignalLight()
    signal_light_thread = QThread()
    signal_light.moveToThread(signal_light_thread)
    signal_light_thread.started.connect(signal_light.green(True))
    signal_light_thread.finished.connect(signal_light.clean_up)
    signal_light_thread.start()

    navigation = Navigation(rs485_port)
    navigation_thread = QThread()
    navigation.moveToThread(navigation_thread)
    navigation_thread.started.connect(navigation.run)
    navigation_thread.finished.connect(navigation.clean_up)
    navigation_thread.start()

    path_manager = PathManager()
    path_manager_thread = QThread()
    path_manager.moveToThread(path_manager_thread)
    path_manager_thread.finished.connect(path_manager.clean_up)
    path_manager_thread.start()

    # #-----------delta-----------
    delta.position_signal.connect(socket_io_interface.delta_position)
    delta.isconnected_signal.connect(socket_io_interface.is_delta_connected)

    socket_io_interface.move_delta_signal.connect(delta.move_delta)
    socket_io_interface.delta_go_home_signal.connect(delta.go_home)
    socket_io_interface.request_delta_status_signal.connect(delta.status_callback)

    # #-----------RTK GPS-----------
    gps.gps_rate_signal.connect(socket_io_interface.gps_rate)
    gps.gps_position_signal.connect(socket_io_interface.gps_position)
    gps.heading_signal.connect(socket_io_interface.heading)
    # gps.gps_ecef_signal.connect(socket_io_interface.gps_ecef)
    gps.adrnav_signal.connect(socket_io_interface.gps_adrnava)
    gps.uniheading_signal.connect(socket_io_interface.gps_uniheading)

    rtcm_receiver.rtcm_data_signal.connect(gps.handle_rtcm)
    socket_io_interface.request_rover_status_signal.connect(gps.request_status)
    socket_io_interface.rate_signal.connect(gps.set_rate)
    socket_io_interface.save_config_signal.connect(gps.save_config)
    socket_io_interface.freset_signal.connect(gps.freset)

    # #-----------imu-----------

    imu.imu_signal.connect(socket_io_interface.imu)

    # #-----------navigation-----------
    navigation.is_rover_connected_signal.connect(socket_io_interface.is_rover_connected)
    navigation.rover_mode_signal.connect(socket_io_interface.rover_mode)
    socket_io_interface.request_rover_status_signal.connect(navigation.rover_status)
    fs_i6.ch0_signal.connect(navigation.control_z_callback)
    fs_i6.ch1_signal.connect(navigation.control_x_callback)
    fs_i6.swa_signal.connect(navigation.swa_callback)
    gps.gps_position_signal.connect(navigation.gps_callback)
    gps.heading_signal.connect(navigation.heading_callback)
    ros2_interface.rpm_signal.connect(navigation.set_rpm)
    socket_io_interface.set_mode_signal.connect(navigation.set_mode)
    socket_io_interface.rover_vel_signal.connect(navigation.set_velocity)
    # #-----------path manager-----------
    socket_io_interface.start_logging_signal.connect(path_manager.start_logging)
    socket_io_interface.stop_logging_signal.connect(path_manager.stop_logging)

    timer = QTimer()
    timer.start(100)  # mỗi 100ms
    timer.timeout.connect(lambda: None)

    def handle_sigint(sig, frame):
        print("Received SIGINT - Closing...")
        ultrasonic_sensors.clean_up()
        thread_rtcm_receiver.quit()
        thread_rtcm_receiver.wait()
        gps_thread.quit()
        gps_thread.wait()
        signal_light_thread.quit()
        signal_light_thread.wait()
        delta_thread.quit()
        delta_thread.wait()
        fs_i6_thread.quit()
        fs_i6_thread.wait()
        imu_thread.quit()
        imu_thread.wait()
        navigation_thread.quit()
        navigation_thread.wait()
        path_manager_thread.quit()
        path_manager_thread.wait()
        if rclpy.ok():
            rclpy.shutdown()
        ros2_interface_thread.quit()
        ros2_interface_thread.wait()
        app.quit()

    signal.signal(signal.SIGINT, handle_sigint)

    sys.exit(app.exec())

