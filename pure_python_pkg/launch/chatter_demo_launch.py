#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

this_pkg_name = 'pure_python_pkg'

def generate_launch_description():
    param_file = PathJoinSubstitution([
        get_package_share_directory(this_pkg_name),
        'config',
        'talker_params.yaml'
    ])
    # param_file = '/home/oscar/ros_ws/tutorials_ws/install/pure_python_pkg/share/pure_python_pkg/config/talker_params.yaml'

    return LaunchDescription([
        Node(
            package=this_pkg_name,
            executable='talker_py',
            name='talker_py',
            parameters=[param_file],
            # parameters=[{'node_frequency': 5.0}],
            output='screen'
        ),
        Node(
            package=this_pkg_name,
            executable='listener_py',
            name='listener_py',
            output='screen'
        )
    ])
