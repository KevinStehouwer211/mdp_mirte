from setuptools import setup
import os
from glob import glob
from setuptools import find_packages

package_name = 'gh_twin'

dirs = ['config', 'launch', 'models', 'worlds', 'rviz', 'maps']

data_files = [
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name]
    ),
    (
        'share/' + package_name,
        ['package.xml']
    ),
]

for directory in dirs:

    for root, _, files in os.walk(directory):

        if files:

            install_path = os.path.join(
                'share',
                package_name,
                root
            )

            file_paths = [
                os.path.join(root, f)
                for f in files
            ]

            data_files.append(
                (
                    install_path,
                    file_paths
                )
            )

setup(
    name=package_name,
    version='0.22.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    author='Abhijith Prakash',
    author_email='abhijithprakas@tudelft.nl',
    maintainer='Abhijith Prakash',
    maintainer_email='abhijithprakas@tudelft.nl',
    keywords=['ROS'],
    description='Navigation action server and client for ROS2',
    license='Apache License, Version 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nav2_interface_node = gh_twin.nav_to_pose:main',
            'publish_waypoint = gh_twin.publish_waypoint:main',
            'joystick_control = gh_twin.joystick_control:main',
        ],
    },
)
