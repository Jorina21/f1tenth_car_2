from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wall_follow_2',
            executable='wall_follow',
            name='PID',
            parameters=[{
                'target_distance': 1.1,
                'look_ahead': 1.0,
                'beam_a_id': 480.0,#400.0, # Original = 340.0
                'beam_b_id': 179.0,
                'K_p': 1.5, # Original 0.5 0.2 1.0
                'K_i': 0.0,
                'K_d': 2.4,#1.8, #3.6,
            }]
        ),
    ])