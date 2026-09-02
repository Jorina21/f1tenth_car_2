from setuptools import setup
import os
from glob import glob

package_name = "simple_driver"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="arc",
    maintainer_email="arc@example.com",
    description="Simple LiDAR wall-centering driver for F1TENTH.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "simple_driver = simple_driver.simple_driver_node:main",
        ],
    },
)