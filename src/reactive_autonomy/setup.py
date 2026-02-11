from setuptools import setup
import os
from glob import glob

package_name = "reactive_autonomy"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "params"), glob("params/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@todo.todo",
    description="Reactive LiDAR autonomy for F1TENTH (Follow-the-Gap + TTC + adaptive speed)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "reactive_autonomy = reactive_autonomy.reactive_autonomy_node:main",
        ],
    },
)
