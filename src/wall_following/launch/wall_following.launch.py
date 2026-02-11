from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('wall_following')
    params_file = os.path.join(pkg_share, 'params', 'wall_following.yaml')

    return LaunchDescription([
        Node(
            package='wall_following',
            executable='wall_following_node',
            name='wall_following',
            output='screen',
            parameters=[params_file],
        )
    ])
