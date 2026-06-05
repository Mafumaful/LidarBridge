from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("yuyi_controller")
    default_params = os.path.join(package_share, "config", "yuyi_controller_params.yaml")

    workspace_root = os.path.abspath(
        os.path.join(package_share, "..", "..", "..", "..")
    )
    default_path_file = os.path.join(
        workspace_root,
        "src",
        "LidarBridge",
        "simulation",
        "paths",
        "generated_path.yaml",
    )

    params_file = LaunchConfiguration("params_file")
    path_file = LaunchConfiguration("path_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Controller parameter yaml",
            ),
            DeclareLaunchArgument(
                "path_file",
                default_value=default_path_file,
                description="Generated path yaml or csv file",
            ),
            Node(
                package="yuyi_controller",
                executable="yuyi_controller_node",
                name="yuyi_controller_node",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "path_file": path_file,
                    },
                ],
            ),
        ]
    )
