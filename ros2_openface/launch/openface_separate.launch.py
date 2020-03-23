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

    config_openface_separate = LaunchConfiguration('config_openface_separate', default=os.path.join(get_package_share_directory(
        'openface'), 'config', 'openface_separate.yaml'))

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_openface_separate',
            default_value=config_openface_separate,
            description='Path to config for openface'),

        Node(
            package='openface',
            node_executable='openface_separate',
            node_name='openface_separate',
            node_namespace='',
            output='screen',
            parameters=[config_openface_separate],
            remappings=[('camera/image_raw', 'camera/color/image_raw'),
                        ('camera/camera_info', 'camera/color/camera_info'),
                        ('openface/landmarks', 'landmarks'),
                        ('openface/landmarks_visible', 'landmarks_visible'),
                        ('openface/landmarks_3d', 'landmarks_3d'),
                        ('openface/head_pose', 'head_pose'),
                        ('openface/eye_landmarks', 'eye_landmarks'),
                        ('openface/eye_landmarks_visible', 'eye_landmarks_visible'),
                        ('openface/eye_landmarks_3d', 'eye_landmarks_3d'),
                        ('openface/gaze_left', 'gaze_left'),
                        ('openface/gaze_right', 'gaze_right'),
                        ('openface/gaze_compound', 'gaze_compound'),
                        ('openface/action_units', 'action_units')],
        ),
    ])
