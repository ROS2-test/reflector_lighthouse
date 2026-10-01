#!/usr/bin/env python3
"""Lighthouse. reflective-marker pose from /scan_light."""

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
    default_markers = os.path.join(lighthouse_dir, 'config', 'markers.yaml')
    if not os.path.isfile(default_markers):
        default_markers = os.path.join(lighthouse_dir, 'config', 'markers_example.yaml')

    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    params_file = LaunchConfiguration('params_file')
    markers_file = LaunchConfiguration('markers_file')

    lighthouse_cmd_group = GroupAction([
        PushROSNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace,
        ),
        Node(
            package='reflector_lighthouse',
            executable='lighthouse_node',
            name='lighthouse_node',
            output='screen',
            parameters=[
                params_file,
                {'markers_file': ParameterValue(markers_file, value_type=str)},
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
        'markers_file',
        default_value=default_markers,
        description='YAML with reflective markers',
    ))
    ld.add_action(lighthouse_cmd_group)
    return ld
