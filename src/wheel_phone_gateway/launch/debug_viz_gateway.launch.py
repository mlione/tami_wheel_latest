from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """启动 `/debug/viz` 手机图像网关。"""
    arguments = [
        DeclareLaunchArgument('image_topic', default_value='/debug/viz'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('stream_fps', default_value='15.0'),
        DeclareLaunchArgument('jpeg_quality', default_value='55'),
        DeclareLaunchArgument('output_width', default_value='640'),
        DeclareLaunchArgument('output_height', default_value='360'),
    ]

    gateway = Node(
        package='wheel_phone_gateway',
        executable='debug_viz_gateway',
        name='debug_viz_gateway',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'port': ParameterValue(
                LaunchConfiguration('port'), value_type=int
            ),
            'stream_fps': ParameterValue(
                LaunchConfiguration('stream_fps'), value_type=float
            ),
            'jpeg_quality': ParameterValue(
                LaunchConfiguration('jpeg_quality'), value_type=int
            ),
            'output_width': ParameterValue(
                LaunchConfiguration('output_width'), value_type=int
            ),
            'output_height': ParameterValue(
                LaunchConfiguration('output_height'), value_type=int
            ),
        }],
    )

    return LaunchDescription([*arguments, gateway])
