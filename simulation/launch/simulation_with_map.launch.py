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
    simulator_launch = os.path.join(package_share, "launch", "ackermann_simulator.launch.py")

    map_yaml_file = LaunchConfiguration("map_yaml_file")
    simulator_params_file = LaunchConfiguration("simulator_params_file")

    return LaunchDescription(
        [
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
                PythonLaunchDescriptionSource(simulator_launch),
                launch_arguments={"params_file": simulator_params_file}.items(),
            ),
            Node(
                package="simulation",
                executable="nav2_map_publisher.py",
                name="nav2_map_publisher_node",
                output="screen",
                parameters=[
                    {
                        "map_yaml_file": map_yaml_file,
                        "map_topic": "/map",
                        "map_metadata_topic": "/map_metadata",
                        "map_frame_id": "map",
                        "republish_period_sec": 2.0,
                    }
                ],
            ),
        ]
    )
