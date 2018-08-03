#!/usr/bin/env python3

from launch.exit_handler import default_exit_handler, restart_exit_handler
from ros2run.api import get_executable_path


def launch(launch_descriptor, argv):
    ld = launch_descriptor
    
    # talker
    ld.add_process(
        cmd=[get_executable_path(package_name='ros2_tutorials', executable_name='talker')],
        name='talker',
        exit_handler=restart_exit_handler,
    )
    
    # listener
    ld.add_process(
        cmd=[get_executable_path(package_name='ros2_tutorials', executable_name='listener')],
        name='listener',
        exit_handler=restart_exit_handler,
    )

    return ld
