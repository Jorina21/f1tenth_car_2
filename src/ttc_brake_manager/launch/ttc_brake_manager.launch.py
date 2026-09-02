from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("ttc_brake_manager")

    params_file = os.path.join(
        pkg_share,
        "params",
        "ttc_brake_manager.yaml",
    )

    return LaunchDescription([
        Node(
            package="ttc_brake_manager",
            executable="ttc_brake_manager",
            name="ttc_brake_manager",
            output="screen",
            parameters=[params_file],
        )
    ])