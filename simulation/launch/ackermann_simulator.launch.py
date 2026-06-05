from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("simulation"),
        "config",
        "ackermann_simulator_params.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="simulation",
                executable="ackermann_simulator_node",
                name="ackermann_simulator_node",
                output="screen",
                parameters=[params_file],
            )
        ]
    )
