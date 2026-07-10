# 🌱 gh_twin

ROS 2 package for the greenhouse digital twin: the Gazebo simulation world, MIRTE Master bringup, SLAM, Nav2 autonomous navigation, joystick teleoperation, and the NiceGUI-based operator dashboard.

This is the "hub" package — most other packages in this workspace (`gh_twin_data_storage`, `flower_perception`) either launch through it or call into its Python classes (`ArmController`, `VisionController`, `NavToPose`) as libraries.

---

# 🎥 Demos

### Autonomous Navigation

![Autonomous navigation](../../docs/media/nav_sidebyside.gif)

Nav2 driving the robot to a goal pose through the greenhouse costmap.

### SLAM

![SLAM](../../docs/media/slam_gui.gif)

Mapping the greenhouse with `slam_toolbox` while teleoperating.

### Teleoperation

![Teleoperation](../../docs/media/teleop.gif)

Manual driving with the PS4 joystick node (`joystick_control`).

---

# 📦 Features

* Gazebo simulation of the greenhouse (crates + walls) with MIRTE Master spawned inside it
* `slam_toolbox`-based mapping (simulation and hardware variants)
* Nav2 autonomous navigation (simulation and hardware variants) with AMCL localization
* PS4-controller teleoperation of both the base (holonomic-capable) and the arm
* `twist_mux`-arbitrated `/cmd_vel` between navigation, teleop and a safety zero
* NiceGUI operator dashboard: live map with robot pose, camera feed, flower/pest/sensor overlays, operating-mode control, alerts/log panels, and optional LLM-based data analysis
* Reusable Python helper classes (`ArmController`, `VisionController`, `NavToPose`) consumed by `gh_twin_data_storage`'s task scheduler

---

# 🚀 Quick Start

```bash
cd ~/ro47007_mirte_ws
pixi shell
source install/setup.bash

ros2 launch gh_twin greenhouse_launch.xml
```

This starts Gazebo with the MIRTE Master spawned in the greenhouse world. See [Launch Files](#-launch-files) below for SLAM, navigation, and joystick control.

---

# 🚦 Launch Files

All paths below are relative to `src/gh_twin/launch/`.

| Launch file | Purpose | Key args (default) |
|---|---|---|
| `greenhouse_launch.xml` | Starts Gazebo + spawns MIRTE Master, controller spawners, and `twist_mux` | `world` (`worlds/greenhouse.world`), `gui` (`true`), `twist_mux_config` (`config/twist_mux.yaml`) |
| `slam.launch.py` | Simulation SLAM: `async_slam_toolbox_node`, keyboard teleop (xterm), RViz | `slam_params_file` (`config/slam_config.yaml`), `use_sim_time` (`false`) |
| `slam_hardware.launch.py` | Hardware SLAM: `sync_slam_toolbox_node` + laser filter + keyboard teleop + RViz | `filter_file` (`config/laser_filter.yaml`), `slam_params_file` (`config/slam_hardware.yaml`) |
| `slam_local.launch.py` | Hardware SLAM without the laser filter stage | `slam_params_file` (`config/slam_hardware.yaml`) |
| `nav.launch.py` | Simulation Nav2 bring-up + laser filter + RViz | `map` (`maps/map.yaml`), `params_file` (`config/navigation.yaml`), `use_sim_time` (`true`) |
| `nav_hardware.launch.py` | Hardware Nav2 bring-up; adds a `/cmd_vel_nav → /mirte_base_controller/cmd_vel` relay that bypasses `twist_mux` | `map` (`maps/map_hardware.yaml`), `params_file` (`config/navigation_hardware.yaml`) |
| `amcl.launch.py` | Standalone localization-only launch (map server + AMCL + keyboard teleop), no full Nav2 stack | `map_file` (`maps/map.yaml`), `amcl_params_file` (`config/amcl_config.yaml`) |
| `joystick_control.launch.py` | `joy_node` + the `joystick_control` teleop node | `cmd_vel_topic` (`/mirte_base_controller/cmd_vel`) |
| `demo_teleop.launch.py` | Full-stack demo: TypeDB server, greenhouse bridge, flower perception, AprilTag detector, task scheduler (hardware), and the GUI — reads workspace root from `$GH_TWIN_WS` | n/a |

> [!IMPORTANT]
> `slam.launch.py`/`amcl.launch.py` remap keyboard teleop to `/mirte_base_controller/cmd_vel_unstamped` (goes through `twist_mux`), while `slam_hardware.launch.py`/`slam_local.launch.py` remap to `/mirte_base_controller/cmd_vel` directly. Only close one teleop source before starting another to avoid two nodes fighting over the topic.

### Navigate to a goal

After launching `nav.launch.py` and RViz has opened:

1. Set **2D Pose Estimate** at the robot's actual pose (matches Gazebo/AMCL's known start).
2. Set **Nav2 Goal** anywhere on the map — the robot should start moving.

---

# 🤖 Nodes

### Launchable (`console_scripts`)

| Executable | Node name | File |
|---|---|---|
| `joystick_control` | `joy_control` | `joystick_control.py` |
| `nav2_interface_node` | `nav2_interface_node` | `nav_to_pose.py` |
| `publish_waypoint` | `nav_client` | `publish_waypoint.py` |

> [!WARNING]
> `publish_waypoint` is legacy scaffolding: it uses the ROS2-tutorial demo action `action_tutorials_interfaces/action/Nav` (not a gh_twin action) and its `main()` sends a hardcoded goal. It is not part of the working navigation pipeline — use `nav2_interface_node` / `nav.launch.py` instead.

### `joystick_control` — PS4 controller teleop

Parameters: `cmd_vel_topic` (default `/mirte_base_controller/cmd_vel`), `joint_trajectory_topic` (default `/mirte_master_arm_controller/joint_trajectory`, not exposed as a launch arg).

Publishes `geometry_msgs/Twist` on `cmd_vel_topic` and `trajectory_msgs/JointTrajectory` on `joint_trajectory_topic`. Subscribes `sensor_msgs/Joy` on `/joy` and `control_msgs/JointTrajectoryControllerState` on `/mirte_master_arm_controller/state` (keeps arm joint positions in sync).

| Mode | Buttons | Behavior |
|---|---|---|
| Dead-man switch | Hold **L1** | Required for any input to be processed |
| Base | (default) | Left stick vertical → `linear.x` (max 0.4 m/s), right stick horizontal → `angular.z` (max 1.0 rad/s) |
| Base + holonomic | Hold **R1** | Adds left stick horizontal → `linear.y` |
| Toggle mode | Press **PS** | Switches between `base` and `arm` |
| Arm | (after PS toggle) | Left stick (horizontal/vertical) and right stick (vertical/horizontal) incrementally jog `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_joint` by 0.002 rad/msg, clamped to `[-1.57, 1.57]` rad |

### `nav2_interface_node` — Nav2 wrapper

Wraps `nav2_simple_commander.BasicNavigator`. Subscribes `geometry_msgs/PoseWithCovarianceStamped` on `amcl_pose`. `nav_set_goal(pose)` does a dry-run `getPath` and only commits via `goToPose` if a path exists (120 s timeout). Used programmatically by `gh_twin_data_storage`'s `plan_executor_node`, not launched standalone.

### Library classes (not `console_scripts`, imported by other nodes)

| Class | File | Used by |
|---|---|---|
| `ArmController` | `arm_controller.py` | `gh_twin_data_storage`'s `plan_executor_node` (arm poses from a YAML config: `move_arm_to_pose(name)`, `execute_spray_sequence()`) |
| `VisionController` | `vision_controller.py` | `plan_executor_node` (calls `flower_perception`'s `/flower_triangulation/{start,finish,reset}` services, publishes `/pest_detected`) |

---

# 🖥️ Operator GUI (`gui/`)

A [NiceGUI](https://nicegui.io/) single-page dashboard for monitoring and controlling the robot.

```bash
cd src/gh_twin/gui/
python3 main.py
```

> [!NOTE]
> The GUI is a plain Python script, not a ROS2 `console_scripts` entry — run it directly as shown above.

* **Login**: file-based auth against `gui/users.txt` (JSON `username → sha256(password)`).
* **ROS bridge**: an internal `ROS2Interface` node runs in a background thread and drives everything below.
* **Live map**: `/amcl_pose` positions a robot marker over `map_hmi_updated.png`; click anywhere on the map (Manual Mode only) to publish a goal `Pose` on `/goal_pose`.
* **Camera feed**: switch between the wrist camera (`/gripper_camera/image_raw/compressed`) and front camera (`/camera/color/image_raw/compressed`), streamed as base64 JPEG.
* **Entity overlays** — one of three views at a time, toggled with the **Display Flowers / Display Pests / Display Sensors** buttons:
  * **Flowers** — from `gui_flower_data` (`gh_twin_msgs/Flower`), colored icons with bloom status and a "Go To Position" action.
  * **Pests** — from `gui_pest_data` (`gh_twin_msgs/Pest`), bug icons with a "Kill Bug" action.
  * **Sensors** — from `/greenhouse/telemetry` (`lupin_greenhouse_msgs/TagReading`), opens a live ECharts line chart per sensor (soil moisture, humidity, temperature, CO₂, light) with a "Take Reading" action.
* **Operating mode** dropdown, published as `std_msgs/Int32` on `/operating_mode`: `-1` No Mode, `0` Measurement Mode, `1` Manual Mode, `2` Pause Mission, `3` Return Home, `4` Reset, `5` Shutdown. Selecting **Measurement Mode** is what kicks off the task-scheduler plan in `gh_twin_data_storage`.
* **Manual teleop**: in Manual Mode, keys `f`/`b`/`r`/`l` publish `Twist` on `/mirte_base_controller/cmd_vel`.
* **Alerts / Log panels**: heartbeat online/offline transitions and navigation clicks (Alerts), connection warnings and save confirmations (Log).
* **Robot status sidebar**: online/offline (from `/robot_heartbeat`), battery bar (from `/io/power/power_watcher`), **SAVE DATA** (writes `plants.yaml`/`bugs.yaml`), **Analyse Data** (sends the current flower/pest/sensor state to an LLM for a health summary).

> [!WARNING]
> The **Analyse Data** feature calls Groq's OpenAI-compatible API (`llama-3.1-8b-instant`) using an API key placeholder in `dashboard_page.py` — supply a real key to use it. Credentials in `users.txt` are stored as unsalted SHA-256 JSON, not suitable for anything beyond local/lab use.

---

# 🌍 World, Models & Maps

* **`worlds/greenhouse.world`** — Gazebo world with a ground plane, sun, 8 `wooden_crate` instances arranged in two rows, and a `greenhouse_walls` include.
* **`models/wooden_crate/`**, **`models/greenhouse_walls/`** — the two custom Gazebo models used in the world.
* **`maps/map.yaml`** / **`map_hardware.yaml`** — simulation and hardware occupancy grids (`resolution: 0.04`) used by `nav.launch.py` / `nav_hardware.launch.py` and `amcl.launch.py`.
* **`maps/waypoints.yaml`** / **`waypoints_hardware.yaml`** — named waypoints (`id`, `bin_id`, `x`, `y`, `yaw`) clustered per crate ("bin"), consumed by `gh_twin_data_storage`'s TypeDB loader.
* **`maps/waypoint_generator.py`** — standalone (non-ROS, interactive) OpenCV tool that detects crate footprints in `map_gh_simplified.pgm` and auto-generates waypoint pairs around each one.

---

# ⚙️ Config (`config/`)

| File | Used by | Notes |
|---|---|---|
| `navigation.yaml` / `navigation_hardware.yaml` | `nav.launch.py` / `nav_hardware.launch.py` | Full Nav2 stack params: AMCL, `RegulatedPurePursuitController`, `SmacPlanner2D`, costmaps, velocity smoother |
| `slam_config.yaml` / `slam_hardware.yaml` | `slam*.launch.py` | `slam_toolbox` params — hardware variant uses a shorter laser range (3 m vs 12 m) and real odometry |
| `amcl_config.yaml` | `amcl.launch.py` | Standalone AMCL tune, notably looser than the AMCL block embedded in `navigation*.yaml` |
| `laser_filter.yaml` | hardware SLAM/nav launches | Clamps `/scan` range to `[0.3, 12.0]` m → `/scan_filtered` |
| `twist_mux.yaml` | `greenhouse_launch.xml` | Arbitrates `/cmd_vel`: `teleop` (`/mirte_base_controller/cmd_vel_unstamped`, priority 200) > `navigation` (`/cmd_vel_nav`, priority 150) > `zero` (`/zero_cmd_vel`, priority 100) |
| `rviz_config.rviz` | most launch files | Shared RViz layout |

> [!IMPORTANT]
> The joystick node and the GUI both publish directly to `/mirte_base_controller/cmd_vel`, **not** to `twist_mux`'s `teleop` input topic (`cmd_vel_unstamped`). In practice this means joystick/GUI teleop bypasses `twist_mux`'s arbitration against Nav2 rather than being priority-arbitrated against it — keep this in mind if the robot is fighting between an active Nav2 goal and manual input.

---

# 📥 Install & Build

```bash
cd ~/ro47007_mirte_ws
pixi shell
colcon build --symlink-install --packages-select gh_twin gh_twin_msgs
source install/setup.bash
```

`console_scripts` (`ros2 run gh_twin <name>`): `nav2_interface_node`, `publish_waypoint`, `joystick_control`.

Depends on `gazebo_ros_pkgs`, `mirte_gazebo`, `mirte_control`, `nav_msgs`, `tf2`, `twist_mux`, `control_msgs`, `joy`, `teleop_twist_keyboard`, `xacro`. The GUI additionally needs `nicegui`, `opencv-python` (`cv_bridge`), `pupil_apriltags` and `openai` from the Pixi/conda environment — these are not declared in `setup.py`.

---

# 📌 Known Issues / Caveats

* `publish_waypoint` executable is unused legacy scaffolding (see above).
* `setup.py` package version (`0.22.0`) doesn't match `package.xml` (`0.2.0`).
* Joystick/GUI teleop bypass `twist_mux` arbitration (see [Config](#%EF%B8%8F-config-config) above).
* `nav_to_pose.py` hardcodes `initial_pose.pose.position.x = 0.5` in its constructor rather than deriving it from a live localization source — intentional per in-code comments, but worth knowing if AMCL is re-seeded unexpectedly.
