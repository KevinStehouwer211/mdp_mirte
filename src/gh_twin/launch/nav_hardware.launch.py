from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = FindPackageShare('gh_twin')
    nav2_bringup_launch_file_dir = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )
    
    rviz_config = PathJoinSubstitution(
            [FindPackageShare("gh_twin"), 'config', 'rviz_config.rviz']
    )
    
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    filter_file = LaunchConfiguration('filter_file')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([pkg_dir, 'maps', 'map_hardware.yaml']),
        description='Full path to map yaml file to load',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_dir, 'config', 'navigation_hardware.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )
    
    declare_laser_filter_file_cmd = DeclareLaunchArgument(
        'filter_file',
        default_value=PathJoinSubstitution([pkg_dir, 'config', 'laser_filter.yaml']),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )


    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_bringup_launch_file_dir]),
        launch_arguments={
            'map': map_yaml_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]
        )

    laser_filter_node = Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            parameters=[filter_file, {'use_sim_time': use_sim_time}],
            remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered')]
    )
    cmd_vel_relay = Node(
    package='topic_tools',
    executable='relay',
    name='cmd_vel_relay',
    arguments=[
        '/cmd_vel_nav',
        '/mirte_base_controller/cmd_vel'
    ]
   )
    return LaunchDescription(
        [
            declare_map_yaml_cmd,
            declare_params_file_cmd,
            declare_use_sim_time_cmd,
            declare_laser_filter_file_cmd,
            nav2_bringup_launch,
            laser_filter_node,
            cmd_vel_relay,
            rviz_node
        ]
    )
