from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gh_twin_data_storage',
            executable='typedb_node',
            name='typedb_storage_node',
            output='screen',
            parameters=[{
            'typedb_address': 'localhost:1729',
            'database_name': 'greenhouse',
            'map_id': 'greenhouse-map-001',
            'robot_id': 'robot1',
            }]
        )
    ])
