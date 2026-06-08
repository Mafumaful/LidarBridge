from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def build_controller_launch_arguments(controller_params_file: str, path_file: str):
    launch_arguments = {
        "params_file": controller_params_file,
    }
    if path_file:
        launch_arguments["path_file"] = path_file
    return launch_arguments


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


def default_controller_params_file(package_share: str) -> str:
    return os.path.join(
        workspace_root_from_share(package_share),
        "src",
        "LidarBridge",
        "yuyi_controller",
        "config",
        "yuyi_controller_params.yaml",
    )


def generate_launch_description():
    simulation_share = get_package_share_directory("simulation")
    controller_share = get_package_share_directory("yuyi_controller")

    simulation_with_rviz_launch = os.path.join(
        simulation_share,
        "launch",
        "simulation_with_rviz.launch.py",
    )
    controller_launch = os.path.join(
        controller_share,
        "launch",
        "yuyi_controller.launch.py",
    )

    workspace_root = os.path.abspath(
        os.path.join(simulation_share, "..", "..", "..", "..")
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
    path_file = LaunchConfiguration("path_file")
    controller_params_file = LaunchConfiguration("controller_params_file")
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
                default_value=default_map_yaml_file(simulation_share),
                description="Nav2-compatible map yaml file to publish as /map",
            ),
            DeclareLaunchArgument(
                "path_file",
                default_value="",
                description="Optional generated path yaml or csv file for the controller",
            ),
            DeclareLaunchArgument(
                "simulator_params_file",
                default_value=default_simulator_params_file(simulation_share),
                description="Ackermann simulator parameter yaml",
            ),
            DeclareLaunchArgument(
                "controller_params_file",
                default_value=default_controller_params_file(controller_share),
                description="Controller parameter yaml",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation_with_rviz_launch),
                launch_arguments={
                    "rviz_config": rviz_config,
                    "map_yaml_file": map_yaml_file,
                    "simulator_params_file": simulator_params_file,
                }.items(),
            ),
            OpaqueFunction(
                function=lambda context: [
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(controller_launch),
                        launch_arguments=build_controller_launch_arguments(
                            controller_params_file.perform(context),
                            path_file.perform(context),
                        ).items(),
                    )
                ]
            ),
        ]
    )
