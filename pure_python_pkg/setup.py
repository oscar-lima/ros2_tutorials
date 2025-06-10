import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'pure_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Oscar.Lima',
    maintainer_email='oscar.lima@dfki.de',
    description='Example of a ROS2 pkg in python',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener_py = pure_python_pkg.listener_py:main',
	        'talker_py = pure_python_pkg.talker_py:main',
        ],
    },
)
