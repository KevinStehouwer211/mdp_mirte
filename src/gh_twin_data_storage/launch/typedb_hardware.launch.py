from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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
            'reset_waypoints_on_start': True,
            "waypoints_file": PathJoinSubstitution([FindPackageShare('gh_twin_data_storage'), 'config', 'waypoints_hardware.yaml']),
            }]
        )
    ])
