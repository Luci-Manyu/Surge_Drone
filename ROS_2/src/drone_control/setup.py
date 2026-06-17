import os
from glob import glob

from setuptools import find_packages, setup

package_name = "drone_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="abhimanyu",
    maintainer_email="ee1251591@iitd.ac.in",
    description="MAVROS bridge + keyboard OFFBOARD velocity control for PX4 SITL.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "offboard_keyboard = drone_control.offboard_keyboard:main",
            "mapping_sweep = drone_control.mapping_sweep:main",
        ],
    },
)
