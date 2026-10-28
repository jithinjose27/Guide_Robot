from setuptools import find_packages, setup
import os
from glob import glob

package_name = "glad_simulation_sensor_filter"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="root",
    maintainer_email="jithin27j@gmail.com",
    description="Sensor fusion and filtering package for G.L.A.D robot",
    license="TODO: License declaration",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "glad_ekf_node = glad_simulation_sensor_filter.glad_ekf_node:main",
        ],
    },
)

