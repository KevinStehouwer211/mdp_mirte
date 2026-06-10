"""Demo launch: brings up the full teleop demo stack in one command.

Equivalent to running, by hand:
    typedb server
    ros2 launch gh_twin_data_storage task_scheduler_hardware.launch.py
    ros2 launch lupin_greenhouse_bridge greenhouse_bridge.launch.py
    python3 src/gh_twin/gh_twin/apriltag_detector.py
    python3 src/gh_twin/gui/main.py
    ros2 launch flower_perception flower_perception.launch.py

`typedb console` is intentionally NOT started here: it is an interactive REPL,
which doesn't belong in a launch file. Open it in its own terminal if you want
to inspect the DB:  typedb console --core=localhost:1729

TypeDB needs a few seconds to start accepting connections, so the components
that talk to it (the task scheduler and the GUI) are delayed by TYPEDB_WARMUP
seconds. Bump that if the scheduler logs a TypeDB connection error on startup.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


# Seconds to wait for `typedb server` to be ready before starting DB clients.
TYPEDB_WARMUP = 8.0

# Workspace source root, where the two plain python scripts live. Override with
# the GH_TWIN_WS env var if your checkout isn't at ~/ro47007_mirte_ws.
WS = os.environ.get("GH_TWIN_WS", os.path.expanduser("~/ro47007_mirte_ws"))


def _include(package, launch_file):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", launch_file)))


def generate_launch_description():
    # 1. TypeDB server (must come up first).
    typedb_server = ExecuteProcess(
        cmd=["typedb", "server"], output="screen")

    # 2-3. ROS launches that don't depend on TypeDB: greenhouse bridge + flower perception.
    greenhouse_bridge = _include(
        "lupin_greenhouse_bridge", "greenhouse_bridge.launch.py")
    flower_perception = _include(
        "flower_perception", "flower_perception.launch.py")

    # 4. AprilTag detector (plain script, run from its own directory).
    apriltag_detector = ExecuteProcess(
        cmd=["python3", "apriltag_detector.py"],
        cwd=os.path.join(WS, "src/gh_twin/gh_twin"),
        output="screen")

    # 5. GUI (plain script, run from its own directory). Talks to TypeDB -> delayed.
    gui = ExecuteProcess(
        cmd=["python3", "main.py"],
        cwd=os.path.join(WS, "src/gh_twin/gui"),
        output="screen")

    # 6. Task scheduler (connects to TypeDB on startup) -> delayed.
    task_scheduler = _include(
        "gh_twin_data_storage", "task_scheduler_hardware.launch.py")

    return LaunchDescription([
        typedb_server,
        greenhouse_bridge,
        flower_perception,
        apriltag_detector,
        # Give TypeDB time to start accepting connections before its clients launch.
        TimerAction(period=TYPEDB_WARMUP, actions=[task_scheduler, gui]),
    ])
