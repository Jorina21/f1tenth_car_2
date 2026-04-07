from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = 'follow_the_gap'
    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'follow_the_gap_params.yaml'
    )

    return LaunchDescription([
        Node(
            package=package_name,
            executable='follow_the_gap',
            name='follow_the_gap',
            output='screen',
            parameters=[config_file]
        )
    ])