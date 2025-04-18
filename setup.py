from setuptools import setup, find_packages
import os
from glob import glob

package_name = "xrover"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="deltax",
    maintainer_email="viminhtu.101002@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "connect_server_node = xrover.ConnectServer:main",
            "navigation_node = xrover.Navigation:main",
            "execute_program_node = xrover.ExecuteProgram:main",
            "motor_controller_node = xrover.MotorController:main",
            "um982_node = xrover.UM982:main",
            "imu_node = xrover.IMU:main",
            "s21c_node = xrover.S21C:main",
            "cmd_vel_node = xrover.test_cmd_vel:main",
            "delta_node = xrover.Delta:main",
            "rtcm_receiver_node = xrover.RTCMReceiver:main",
            "program_reader_node = xrover.ProgramReader:main",
            "fs_i6_node = xrover.FS_I6:main",
            "signal_light_node = xrover.SignalLight:main",
            "load_path_node = xrover.LoadPath:main",
            "path_logger_node = xrover.PathLogger:main",
        ],
    },
)
