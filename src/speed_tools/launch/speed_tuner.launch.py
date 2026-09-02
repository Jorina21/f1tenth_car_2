from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='/qualifier_node',
        description='Target node with speed_scale parameter',
    )

    param_name_arg = DeclareLaunchArgument(
        'param_name',
        default_value='speed_scale',
        description='Parameter to tune',
    )

    min_scale_arg = DeclareLaunchArgument(
        'min_scale',
        default_value='0.6',
        description='Minimum speed scale',
    )

    max_scale_arg = DeclareLaunchArgument(
        'max_scale',
        default_value='5.0',  #1.2
        description='Maximum speed scale',
    )

    step_arg = DeclareLaunchArgument(
        'step',
        default_value='0.5', #0.1
        description='Speed scale step',
    )

    start_scale_arg = DeclareLaunchArgument(
        'start_scale',
        default_value='0.6',
        description='Starting speed scale',
    )

    speed_tuner = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'speed_tools',
            'speed_tuner',
            '--node-name',
            LaunchConfiguration('node_name'),
            '--param-name',
            LaunchConfiguration('param_name'),
            '--min-scale',
            LaunchConfiguration('min_scale'),
            '--max-scale',
            LaunchConfiguration('max_scale'),
            '--step',
            LaunchConfiguration('step'),
            '--start-scale',
            LaunchConfiguration('start_scale'),
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        node_name_arg,
        param_name_arg,
        min_scale_arg,
        max_scale_arg,
        step_arg,
        start_scale_arg,
        speed_tuner,
    ])