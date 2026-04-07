from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = 'pure_pursuit'
    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'pure_pursuit_params.yaml'
    )

    return LaunchDescription([
        Node(
            package=package_name,
            executable='pure_pursuit',
            name='pure_pursuit',
            output='screen',
            parameters=[config_file]
        )
    ])