#!/usr/bin/env python3
"""Marker mapper. build markers.yaml from /scan_light while driving."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    lighthouse_dir = get_package_share_directory('reflector_lighthouse')
    default_params = os.path.join(lighthouse_dir, 'config', 'params.yaml')
    default_out = '/home/user/ros2_ws/src/reflector_lighthouse/config/markers.yaml'

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    params_file = LaunchConfiguration('params_file')
    output_file = LaunchConfiguration('output_file')

    reflector_mapper_cmd_group = GroupAction([
        PushROSNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace,
        ),
        Node(
            package='reflector_lighthouse',
            executable='marker_mapper_node',
            name='marker_mapper_node',
            output='screen',
            parameters=[
                params_file,
                {'output_file': ParameterValue(output_file, value_type=str)},
            ],
        ),
    ])

    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('namespace', default_value=''))
    ld.add_action(DeclareLaunchArgument('use_namespace', default_value='false'))
    ld.add_action(DeclareLaunchArgument(
        'params_file', default_value=default_params,
    ))
    ld.add_action(DeclareLaunchArgument(
        'output_file',
        default_value=default_out,
        description='Where to write markers.yaml (autosave on shutdown)',
    ))
    ld.add_action(reflector_mapper_cmd_group)
    return ld
