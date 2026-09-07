import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("hipnuc_imu"), "config", "can.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("interface", default_value="can0"),
            DeclareLaunchArgument("node_id", default_value="8"),
            Node(
                package="hipnuc_imu",
                executable="can_node",
                name="hipnuc_can",
                output="screen",
                parameters=[
                    config,
                    {"interface": LaunchConfiguration("interface"), "node_id": LaunchConfiguration("node_id")},
                ],
            ),
        ]
    )
