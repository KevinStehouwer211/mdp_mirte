from setuptools import setup, find_packages
from glob import glob

package_name = 'mirte_indicator_devices'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/launch',
            glob('launch/*.py')
        ),
        (
            'share/' + package_name + '/sounds',
            glob('sounds/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'audio_node = mirte_indicator_devices.audio_node:main',
            'led_controller_node = mirte_indicator_devices.led_controller_node:main',
        ],
    },
)