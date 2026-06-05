from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("simulation")
    simulator_launch = os.path.join(package_share, "launch", "ackermann_simulator.launch.py")
    default_map_yaml = os.path.join(package_share, "maps", "simulation_map.yaml")

    map_yaml_file = LaunchConfiguration("map_yaml_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_yaml_file",
                default_value=default_map_yaml,
                description="Nav2-compatible map yaml file to publish as /map",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulator_launch)
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
