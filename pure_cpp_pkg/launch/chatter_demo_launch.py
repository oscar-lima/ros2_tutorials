#!/usr/bin/env python3

from launch.exit_handler import default_exit_handler, restart_exit_handler
from ros2run.api import get_executable_path

def launch(launch_descriptor, argv):
    ld = launch_descriptor

    # talker
    ld.add_process(
        cmd=[get_executable_path(package_name='pure_cpp_pkg', executable_name='talker_cpp')],
        name='talker',
        exit_handler=restart_exit_handler,
    )

    # listener
    ld.add_process(
        cmd=[get_executable_path(package_name='pure_cpp_pkg', executable_name='listener_cpp')],
        name='listener',
        exit_handler=restart_exit_handler,
    )

    return ld
