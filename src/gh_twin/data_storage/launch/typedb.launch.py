from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gh_twin_data_storage',
            executable='typedb_node',
            name='typedb_storage_node',
            output='screen',
        )
    ])
