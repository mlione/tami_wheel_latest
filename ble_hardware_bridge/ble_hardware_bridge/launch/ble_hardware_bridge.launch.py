from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = (
        Path(get_package_share_directory("ble_hardware_bridge"))
        / "config"
        / "ble_hardware_bridge.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_config),
                description="Path to the BLE hardware bridge parameter file.",
            ),
            Node(
                package="ble_hardware_bridge",
                executable="ble_hardware_bridge_node",
                name="ble_hardware_bridge_node",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
