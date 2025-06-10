#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hybrid_pkg',
            executable='talker_cpp',
            name='talker',
            output='screen'
        ),
        Node(
            package='hybrid_pkg',
            executable='listener_cpp',
            name='listener',
            output='screen'
        )
    ])
