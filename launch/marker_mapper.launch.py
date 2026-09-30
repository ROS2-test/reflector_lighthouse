#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('reflector_lighthouse')
    params_file = LaunchConfiguration('params_file').perform(context)
    output_file = LaunchConfiguration('output_file').perform(context)

    return [
        Node(
            package='reflector_lighthouse',
            executable='marker_mapper_node',
            name='marker_mapper_node',
            output='screen',
            parameters=[
                params_file,
                {'output_file': output_file},
            ],
        )
    ]


def generate_launch_description():
    pkg = get_package_share_directory('reflector_lighthouse')
    default_params = os.path.join(pkg, 'config', 'params.yaml')
    # Пишем в src, чтобы файл был виден в репозитории
    default_out = os.path.expanduser(
        '~/ros2_ws/src/reflector_lighthouse/config/markers.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('output_file', default_value=default_out),
        OpaqueFunction(function=_launch_setup),
    ])
