from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def workspace_root_from_share(package_share: str) -> str:
    return os.path.abspath(os.path.join(package_share, "..", "..", "..", ".."))


def default_map_yaml_file(package_share: str) -> str:
    return os.path.join(
        workspace_root_from_share(package_share),
        "src",
        "LidarBridge",
        "simulation",
        "maps",
        "simulation_map.yaml",
    )


def default_simulator_params_file(package_share: str) -> str:
    return os.path.join(
        workspace_root_from_share(package_share),
        "src",
        "LidarBridge",
        "simulation",
        "config",
        "ackermann_simulator_params.yaml",
    )


def generate_launch_description():
    package_share = get_package_share_directory("simulation")
    simulation_with_map_launch = os.path.join(
        package_share,
        "launch",
        "simulation_with_map.launch.py",
    )

    workspace_root = os.path.abspath(
        os.path.join(package_share, "..", "..", "..", "..")
    )
    default_rviz_config = os.path.join(
        workspace_root,
        "src",
        "LidarBridge",
        "projectConfigs",
        "simulationConifg.rviz",
    )

    rviz_config = LaunchConfiguration("rviz_config")
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    simulator_params_file = LaunchConfiguration("simulator_params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz2 config file",
            ),
            DeclareLaunchArgument(
                "map_yaml_file",
                default_value=default_map_yaml_file(package_share),
                description="Nav2-compatible map yaml file to publish as /map",
            ),
            DeclareLaunchArgument(
                "simulator_params_file",
                default_value=default_simulator_params_file(package_share),
                description="Ackermann simulator parameter yaml",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation_with_map_launch),
                launch_arguments={
                    "map_yaml_file": map_yaml_file,
                    "simulator_params_file": simulator_params_file,
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
