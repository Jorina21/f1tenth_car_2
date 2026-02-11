from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("reactive_autonomy")
    params_file = os.path.join(pkg_share, "params", "reactive_autonomy.yaml")

    return LaunchDescription([
        Node(
            package="reactive_autonomy",
            executable="reactive_autonomy",
            name="reactive_autonomy",
            output="screen",
            parameters=[params_file],
        )
    ])
