#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

this_pkg_name = 'pure_python_pkg'

def generate_launch_description():
    param_file = PathJoinSubstitution([
        get_package_share_directory(this_pkg_name),
        'config',
        'talker_params.yaml'
    ])

    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for nodes'
        ),
        Node(
            package=this_pkg_name,
            executable='talker_py',
            name='talker_py',
            namespace=namespace,
            parameters=[param_file],
            output='screen'
        ),
        Node(
            package=this_pkg_name,
            executable='listener_py',
            name='listener_py',
            namespace=namespace,
            output='screen'
        )
    ])
