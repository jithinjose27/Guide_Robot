import os
from glob import glob
from setuptools import find_packages, setup

package_name = "glad_simulation_teleop"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="jithin27j@gmail.com",
    description="Keyboard teleop for GLAD robot",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "teleop_node=glad_simulation_teleop.teleop_node:main",
        ],
    },
)
