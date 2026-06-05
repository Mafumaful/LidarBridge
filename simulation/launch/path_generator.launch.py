from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("simulation")
    default_config = os.path.join(package_share, "config", "path_generator_params.yaml")

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
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
