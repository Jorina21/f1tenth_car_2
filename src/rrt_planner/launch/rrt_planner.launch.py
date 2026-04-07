from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = 'rrt_planner'
    config_file = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'rrt_params.yaml'
    )

    return LaunchDescription([
        Node(
            package=package_name,
            executable='rrt_planner',
            name='rrt_planner',
            output='screen',
            parameters=[config_file]
        )
    ])