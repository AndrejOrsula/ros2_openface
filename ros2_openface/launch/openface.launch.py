"""Launch OpenFace"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():

    config_openface = LaunchConfiguration('config_openface', default=os.path.join(get_package_share_directory(
        'openface'), 'config', 'openface.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_openface',
            default_value=config_openface,
            description='Path to config for openface'),

        Node(
            package='openface',
            node_executable='openface',
            node_name='openface',
            node_namespace="",
            output='screen',
            parameters=[config_openface],
            remappings=[('camera/image_raw', 'camera/color/image_raw'),
                        ('camera/camera_info', 'camera/color/camera_info'),
                        ('openface/faces', 'faces')],
        ),
    ])
