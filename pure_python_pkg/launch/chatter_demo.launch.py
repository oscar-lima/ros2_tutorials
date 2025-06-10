#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pure_python_pkg',
            executable='talker_py',
            name='talker_py',
            output='screen'
        ),
        Node(
            package='pure_python_pkg',
            executable='listener_py',
            name='listener_py',
            output='screen'
        )
    ])
