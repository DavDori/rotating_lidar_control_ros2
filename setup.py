from setuptools import find_packages, setup
import os
from glob import glob

package_name = "rotating_lidar_ctrl"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="DavDori",
    maintainer_email="davidedorigoni@gmail.com",
    description="Transform a 2D LiDAR into a 3D LiDAR by acting on the stepper angle.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "turret = rotating_lidar_ctrl.turret:main",
        ],
    },
)
