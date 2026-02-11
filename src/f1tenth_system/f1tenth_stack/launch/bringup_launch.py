from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    joy_teleop_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'joy_teleop.yaml'
    )
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc.yaml'
    )
    sensors_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'sensors.yaml'
    )
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mux.yaml'
    )

    # Launch arguments
    joy_la = DeclareLaunchArgument('joy_config', default_value=joy_teleop_config)
    vesc_la = DeclareLaunchArgument('vesc_config', default_value=vesc_config)
    sensors_la = DeclareLaunchArgument('sensors_config', default_value=sensors_config)
    mux_la = DeclareLaunchArgument('mux_config', default_value=mux_config)

    ld = LaunchDescription([joy_la, vesc_la, sensors_la, mux_la])

    # JOYSTICK
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )

    # THROTTLE MIXING NODE
    throttle_mix_node = Node(
        package='f1tenth_stack',
        executable='throttle_mix',
        name='throttle_mix',
        output='screen',
        remappings=[
            ('/ackermann_cmd', '/ackermann_cmd')
        ]
    )

    # VESC NODES
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )

    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )

    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )

    # LD19 LIDAR
    ld19_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        parameters=[LaunchConfiguration('sensors_config')]
    )

    # MUX
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[
            ('ackermann_cmd_out', 'ackermann_cmd')
        ]
    )

    # LASER TF
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.27', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )

    # Add Actions
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(throttle_mix_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(ld19_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(static_tf_node)

    return ld
