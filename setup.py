from setuptools import setup,find_packages
import os
from glob import glob

package_name = 'xrover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='deltax',
    maintainer_email='viminhtu.101002@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = xrover.sensor:main',
            'connect_server_node = xrover.connect_server:main',
            'navigator_node = xrover.navigation:main',
            'execute_program_node = xrover.execute_program:main',
            'motor_controller_node = xrover.motor_controller:main',
            'rtcm3_publisher = xrover.connect_base_station:main',
            'um982_node = xrover.um982:main',
            'imu_node = xrover.imu_node:main',
            's21c_node = xrover.s21c_node:main',
        ],
    },
)
