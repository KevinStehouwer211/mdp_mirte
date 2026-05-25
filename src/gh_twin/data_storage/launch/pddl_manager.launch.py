from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    world_file = LaunchConfiguration('world_file', default='')
    problem_file = LaunchConfiguration('problem_file', default='')
    planner_cmd = LaunchConfiguration('planner_cmd', default='')

    declare_world_file = DeclareLaunchArgument(
        'world_file',
        default_value=world_file,
        description='Path to an existing world.pddl file',
    )

    declare_problem_file = DeclareLaunchArgument(
        'problem_file',
        default_value=problem_file,
        description='Path to an existing problem.pddl file',
    )

    declare_planner_cmd = DeclareLaunchArgument(
        'planner_cmd',
        default_value=planner_cmd,
        description='Optional external planner command for replan',
    )

    pddl_node = Node(
        package='gh_twin_data_storage',
        executable='pddl_manager_node',
        name='pddl_manager_node',
        output='screen',
        arguments=[
            '--world-file', world_file,
            '--problem-file', problem_file,
            '--planner-cmd', planner_cmd,
        ],
    )

    return LaunchDescription([
        declare_world_file,
        declare_problem_file,
        declare_planner_cmd,
        pddl_node,
    ])
