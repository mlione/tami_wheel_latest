"""Offline RGB-D -> perception -> DWA -> LQR validation; no hardware bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('wheel_perception')
    dataset_path = LaunchConfiguration('dataset_path')
    fps = LaunchConfiguration('fps')
    loop = LaunchConfiguration('loop')

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'run_launch.py')
        )
    )
    player = Node(
        package='wheel_perception',
        executable='zed_dataset_player.py',
        name='zed_dataset_player',
        output='screen',
        parameters=[{
            'dataset_path': dataset_path,
            'fps': ParameterValue(fps, value_type=float),
            'loop': ParameterValue(loop, value_type=bool),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('dataset_path'),
        DeclareLaunchArgument('fps', default_value='10.0'),
        DeclareLaunchArgument('loop', default_value='false'),
        navigation,
        # TensorRT engine construction and lifecycle activation take several
        # seconds. Starting replay immediately silently drops the first frames.
        TimerAction(period=8.0, actions=[player]),
    ])
