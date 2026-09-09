import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory("hipnuc_imu"), "config", "serial.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("baudrate", default_value="115200"),
            DeclareLaunchArgument("frame_id", default_value="imu_link"),
            Node(
                package="hipnuc_imu",
                executable="serial_node",
                name="hipnuc_serial",
                output="screen",
                parameters=[
                    config,
                    {
                        "port": LaunchConfiguration("port"),
                        "baudrate": ParameterValue(LaunchConfiguration("baudrate"), value_type=int),
                        "frame_id": LaunchConfiguration("frame_id"),
                    },
                ],
            ),
        ]
    )
