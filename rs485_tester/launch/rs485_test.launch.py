from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rs485_tester",
            executable="rs485_test_node",
            name="rs485_test_node",
            output="screen",
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare("rs485_tester"), "config", "rs485_params.yaml"]
                )
            ],
        )
    ])
