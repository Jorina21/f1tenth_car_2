#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("simple_driver")

    config_file = os.path.join(
        pkg_share,
        "config",
        "simple_driver.yaml"
    )

    simple_driver_node = Node(
        package="simple_driver",
        executable="simple_driver",
        name="simple_driver",
        output="screen",
        parameters=[config_file]
    )

    return LaunchDescription([
        simple_driver_node
    ])