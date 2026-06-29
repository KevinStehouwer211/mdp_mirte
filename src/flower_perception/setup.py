import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'flower_perception'

data_files = [
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name],
    ),
    (
        'share/' + package_name,
        ['package.xml'],
    ),
    ('share/' + package_name + '/launch', ['launch/flower_perception.launch.py']),
]

for directory in ['launch', 'models', 'rviz', 'datasets']:
    if not os.path.exists(directory):
        continue

    for root, _, files in os.walk(directory):
        if files:
            install_path = os.path.join(
                'share',
                package_name,
                root,
            )

            file_paths = [
                os.path.join(root, filename)
                for filename in files
            ]

            data_files.append(
                (
                    install_path,
                    file_paths,
                )
            )


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kevin Stehouwer',
    maintainer_email='kstehouwer@tudelft.nl',
    description='Flower perception package for detecting flowers and pests',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'flower_detector = flower_perception.flower_detector_node:main',
            'flower_detector_tracker = flower_perception.flower_detector_tracker_node:main',
            'lidar_to_camera_projection = flower_perception.lidar_to_camera_projection_node:main',
            'flower_height_estimator = flower_perception.flower_height_estimator_node:main',
            'flower_triangulation = flower_perception.flower_triangulation_node:main',
            'flower_bbox_projection = flower_perception.flower_bbox_projection_node:main',
        ],
    },
)