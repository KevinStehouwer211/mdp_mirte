import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def mode_is(mode_name):
    return IfCondition(PythonExpression(["'", LaunchConfiguration("mode"), "' == '", mode_name, "'"]))

def generate_launch_description():
    mode = LaunchConfiguration("mode")
    use_sim_time = LaunchConfiguration("use_sim_time")
    domain_file = LaunchConfiguration("domain_file")
    problem_file = LaunchConfiguration("problem_file")
    typedb_address = LaunchConfiguration("typedb_address")
    database_name = LaunchConfiguration("database_name")
    battery_topic = LaunchConfiguration("battery_topic")
    battery_threshold = LaunchConfiguration("battery_threshold")
    recharge_waypoint = LaunchConfiguration("recharge_waypoint")
    conda_prefix = os.environ.get("CONDA_PREFIX", "")
    current_ld_library_path = os.environ.get("LD_LIBRARY_PATH", "")
    pixi_library_path = os.path.join(conda_prefix, "lib") if conda_prefix else ""
    runtime_library_path = (
        f"{pixi_library_path}:{current_ld_library_path}"
        if pixi_library_path
        else current_ld_library_path
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gh_twin"), "launch", "slam.launch.py"])
        ),
        condition=mode_is("manual_slam"),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("gh_twin"), "launch", "nav_hardware.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
        }.items(),
    )

    typedb_node = Node(
        package="gh_twin_data_storage",
        executable="typedb_node",
        name="typedb_storage_node",
        output="screen",
        parameters=[
            {
                "typedb_address": typedb_address,
                "database_name": database_name,
                "map_id": "greenhouse-map-001",
                "robot_id": "robot1",
                "current_wp_id": "",
                "reset_waypoints_on_start": True,
                "waypoints_file": PathJoinSubstitution([
                    FindPackageShare("gh_twin_data_storage"),
                    "config",
                    "waypoints_hardware.yaml",
                ]),
            }
        ],
    )

    plan_executor = Node(
        package="gh_twin_data_storage",
        executable="plan_executor_node",
        name="plan_executor",
        output="screen",
        arguments=[
            "--domain-file",
            domain_file,
            "--problem-file",
            problem_file,
        ],
        parameters=[
            {
                "operating_mode_topic": "/operating_mode",
                "use_sim_time": use_sim_time,
                "battery_topic": battery_topic,
                "battery_threshold": battery_threshold,
                "recharge_waypoint": recharge_waypoint,
                "typedb_address": typedb_address,
                "database_name": database_name,
                "robot_heartbeat_topic": "/robot_heartbeat",
                "arm_poses_file": PathJoinSubstitution([
                    FindPackageShare("gh_twin_data_storage"),
                    "config",
                    "arm_poses.yml",
                ]),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="manual_slam",
                choices=["manual_slam", "autonomous"],
                description="manual_slam records waypoints; autonomous executes the PDDL/Nav2 plan.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation/Gazebo clock.",
            ),
            DeclareLaunchArgument(
                "domain_file",
                default_value="DomainGreenhouse.pddl",
                description="PDDL domain file used by the autonomous executor.",
            ),
            DeclareLaunchArgument(
                "problem_file",
                default_value="problem_generated.pddl",
                description="Generated PDDL problem file.",
            ),
            DeclareLaunchArgument(
                "typedb_address",
                default_value="localhost:1729",
                description="TypeDB server address.",
            ),
            DeclareLaunchArgument(
                "database_name",
                default_value="greenhouse",
                description="TypeDB database name.",
            ),
            DeclareLaunchArgument(
                "battery_topic",
                default_value="io/power/power_watcher",
                description="BatteryState topic published by the MIRTE INA226 power watcher.",
            ),
            DeclareLaunchArgument(
                "battery_threshold",
                default_value="0.25",
                description="Return to recharge when battery percentage is below this fraction.",
            ),
            DeclareLaunchArgument(
                "recharge_waypoint",
                default_value="wp_0",
                description="Waypoint used as the recharge location.",
            ),
            SetEnvironmentVariable("LD_LIBRARY_PATH", runtime_library_path),
            typedb_node,
            #slam_launch,
            nav_launch,
            plan_executor,
        ]
    )
