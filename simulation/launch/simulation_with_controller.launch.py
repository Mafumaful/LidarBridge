from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


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
    default_map_yaml = os.path.join(simulation_share, "maps", "simulation_map.yaml")
    default_path_file = os.path.join(
        workspace_root,
        "src",
        "LidarBridge",
        "simulation",
        "paths",
        "generated_path.yaml",
    )
    default_controller_params = os.path.join(
        controller_share,
        "config",
        "yuyi_controller_params.yaml",
    )

    rviz_config = LaunchConfiguration("rviz_config")
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    path_file = LaunchConfiguration("path_file")
    controller_params_file = LaunchConfiguration("controller_params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz2 config file",
            ),
            DeclareLaunchArgument(
                "map_yaml_file",
                default_value=default_map_yaml,
                description="Nav2-compatible map yaml file to publish as /map",
            ),
            DeclareLaunchArgument(
                "path_file",
                default_value=default_path_file,
                description="Generated path yaml or csv file for the controller",
            ),
            DeclareLaunchArgument(
                "controller_params_file",
                default_value=default_controller_params,
                description="Controller parameter yaml",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulation_with_rviz_launch),
                launch_arguments={
                    "rviz_config": rviz_config,
                    "map_yaml_file": map_yaml_file,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(controller_launch),
                launch_arguments={
                    "params_file": controller_params_file,
                    "path_file": path_file,
                }.items(),
            ),
        ]
    )
