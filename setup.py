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
    install_requires=["setuptools",
                    #   "pyserial",
                    #   "pyside6",
                    #   "python-socketio[client]>=5.13.0",
                    #   "numpy",
                    #   "opencv-python",
                    #   "cv-bridge",
                      ],
    zip_safe=True,
    maintainer="deltax",
    maintainer_email="viminhtu.101002@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            
        ],
    },
)
