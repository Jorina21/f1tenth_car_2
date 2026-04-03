from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    map_yaml_file = LaunchConfiguration("map")
    amcl_params_file = LaunchConfiguration("amcl_params_file")

    declare_map_arg = DeclareLaunchArgument(
        "map",
        description="Full path to map yaml file"
    )

    declare_amcl_params_arg = DeclareLaunchArgument(
        "amcl_params_file",
        default_value="/home/arc/f1tenth_ws/src/f1tenth_localization/config/amcl.yaml",
        description="Full path to AMCL params file"
    )

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_yaml_file},
            {"use_sim_time": False}
        ]
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[amcl_params_file]
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {"node_names": ["map_server", "amcl"]}
        ]
    )

    return LaunchDescription([
        declare_map_arg,
        declare_amcl_params_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
    ])