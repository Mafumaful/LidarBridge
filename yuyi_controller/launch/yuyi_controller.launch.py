from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
        "yuyi_controller",
        "config",
        "yuyi_controller_params.yaml",
    )


def build_node_parameters(params_file: str, path_file: str):
    parameters = [params_file]
    if path_file:
        parameters.append({"path_file": path_file})
    return parameters


def _create_controller_node(context):
    params_file = LaunchConfiguration("params_file").perform(context)
    path_file = LaunchConfiguration("path_file").perform(context)

    return [
        Node(
            package="yuyi_controller",
            executable="yuyi_controller_node",
            name="yuyi_controller_node",
            output="screen",
            parameters=build_node_parameters(params_file, path_file),
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory("yuyi_controller")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file(package_share),
                description="Controller parameter yaml",
            ),
            DeclareLaunchArgument(
                "path_file",
                default_value="",
                description="Optional generated path yaml or csv file override",
            ),
            OpaqueFunction(function=_create_controller_node),
        ]
    )
