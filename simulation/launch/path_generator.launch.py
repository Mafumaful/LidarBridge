from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def workspace_root_from_share(package_share: str) -> str:
    return os.path.abspath(os.path.join(package_share, "..", "..", "..", ".."))


def default_config_file(package_share: str) -> str:
    return os.path.join(
        workspace_root_from_share(package_share),
        "src",
        "LidarBridge",
        "simulation",
        "config",
        "path_generator_params.yaml",
    )


def generate_launch_description():
    package_share = get_package_share_directory("simulation")

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config_file(package_share),
                description="Path generator config yaml",
            ),
            Node(
                package="simulation",
                executable="path_generator.py",
                name="path_generator",
                arguments=["--config", config_file],
                output="screen",
            ),
        ]
    )
