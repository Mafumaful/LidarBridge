from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="lio_scan_monitor",
            executable="lio_scan_monitor_node",
            name="lio_scan_monitor_node",
            output="screen",
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare("lio_scan_monitor"), "config", "lio_scan_monitor_params.yaml"]
                )
            ],
        )
    ])
