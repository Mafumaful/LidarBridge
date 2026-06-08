from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def workspace_root_from_share(package_share: str) -> str:
    return os.path.abspath(os.path.join(package_share, "..", "..", "..", ".."))


def default_params_file(package_share: str) -> str:
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
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file(package_share),
                description="Ackermann simulator parameter yaml",
            ),
            Node(
                package="simulation",
                executable="ackermann_simulator_node",
                name="ackermann_simulator_node",
                output="screen",
                parameters=[params_file],
            )
        ]
    )
