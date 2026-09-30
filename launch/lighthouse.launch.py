#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('reflector_lighthouse')
    default_params = os.path.join(pkg, 'config', 'params.yaml')
    default_markers = os.path.join(pkg, 'config', 'markers_example.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('markers_file', default_value=default_markers),
        Node(
            package='reflector_lighthouse',
            executable='lighthouse_node',
            name='lighthouse_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'markers_file': LaunchConfiguration('markers_file')},
            ],
        ),
    ])
