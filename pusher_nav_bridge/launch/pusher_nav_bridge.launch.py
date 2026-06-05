from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare("pusher_nav_bridge"),
        "config",
        "pusher_nav_bridge_params.yaml",
    ])

    return LaunchDescription([
        Node(
            package="pusher_nav_bridge",
            executable="pusher_nav_bridge_node",
            name="pusher_nav_bridge_node",
            output="screen",
            parameters=[params],
        )
    ])
