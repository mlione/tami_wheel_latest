"""Opt-in ZED-only indoor DWA test. Never launches the BLE bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    main_launch = os.path.join(
        get_package_share_directory('wheel_perception'), 'launch', 'run_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='visualize',
                              description='visualize (no motion) or drive (unmanned low speed)'),
        DeclareLaunchArgument('allow_motion', default_value='false',
                              description='Must be true together with mode:=drive to enable motion'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(main_launch),
            launch_arguments={
                'use_dataset_mode': 'false',
                'indoor_mode': LaunchConfiguration('mode'),
                'indoor_allow_motion': LaunchConfiguration('allow_motion'),
            }.items(),
        ),
    ])
