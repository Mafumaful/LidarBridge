from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("simulation")
    simulator_launch = os.path.join(package_share, "launch", "ackermann_simulator.launch.py")

    workspace_root = os.path.abspath(
        os.path.join(package_share, "..", "..", "..", "..")
    )
    rviz_config = os.path.join(
        workspace_root,
        "src",
        "LidarBridge",
        "projectConfigs",
        "simulationConifg.rviz",
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(simulator_launch)
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
