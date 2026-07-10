# 🗄️ gh_twin_data_storage

ROS 2 package for the greenhouse digital twin's data layer and autonomous task scheduler: it stores the world state (waypoints, plants, pests, sensor readings) in a **TypeDB** knowledge base, generates **PDDL** planning problems from that state, invokes the **POPF** planner, and executes the resulting plan step by step on the robot.

> This package is part of a Pixi-managed workspace. Run `pixi shell` from the `mdp_mirte_ws` root before building or launching.

---

# 🎥 Demo

![Autonomous navigation](../../docs/media/nav_sidebyside.gif)

Autonomous waypoint navigation driven by the `plan_executor_node` as it works through a generated plan.

---

# 📦 What's in here

| Node | Executable | Role |
|---|---|---|
| `TypeDBStorageNode` | `typedb_node` | Persists detections into TypeDB and republishes flower/pest data to the GUI |
| `PddlPlannerNode` | `pddl_planner_node` | Reads TypeDB state → generates a PDDL problem → runs POPF (also usable as a library) |
| `PlanExecutorNode` | `plan_executor_node` | The task scheduler: parses a POPF plan and drives navigation/arm/vision to execute it |

> [!WARNING]
> `sql_db_node` and `nosql_db_node` are declared in `setup.py`/`CMakeLists.txt` as placeholder future work but their source files (`sql_db_node.py`, `nosql_db_node.py`) don't exist yet — building/running them will fail until implemented.

---

# 🧠 How planning works, end to end

```
TypeDB (waypoints, plants, pests, arm/robot state)
        │  PddlPlannerNode.generate_pddl_problem()
        ▼
problem_generated.pddl  ──(with DomainGreenhouse.pddl)──▶  POPF binary
        │  PddlPlannerNode.run_planner() / replan()
        ▼
POPF stdout ("<time>: (action args...) [duration]" lines)
        │  PlanExecutorNode.parse_popf_plan()
        ▼
ordered action list (move / move-while-scan / start-scan / finish-scan)
        │  PlanExecutorNode.execute_plan_steps()
        ▼
NavToPose (gh_twin) + ArmController (gh_twin) + VisionController (gh_twin)
        │
        └─▶ writes each waypoint arrival / arm-state change back into TypeDB
```

Every waypoint arrival and arm-state change is written straight back into TypeDB, so the next `replan()` always sees up-to-date world state. If a `move` fails, the target waypoint's failure count increments; below `max_replan_attempts` (default `3`) the specific edge is blocked and a fresh plan is generated, otherwise the waypoint is marked infeasible (and treated as already visited/scanned) so the goal no longer requires it.

**PDDL domain** (`DomainGreenhouse.pddl`, repo root) defines 4 durative actions over `robot`, `plant`, `pest`, `waypoint`, `arm` types:

| Action | Precondition | Effect |
|---|---|---|
| `move(?r ?a ?from ?to)` | robot at `from`, `connected-wp from to`, arm at `base` | robot at `to`, `to` marked visited |
| `start-scan(?r ?a ?wp)` | robot at a bin **start** waypoint, arm at `base` | arm → `scan` |
| `move-while-scan(?r ?a ?from ?to)` | like `move`, but over `scan-connected-wp`, arm at `scan` | robot moves while scanning |
| `finish-scan(?r ?a ?wp)` | robot at a bin **end** waypoint, arm at `scan` | arm → `base` |

> [!NOTE]
> There is currently no `spray` action in the domain, even though `pest-at`/`pest-sprayed` predicates exist — pest spraying is modeled in TypeDB/GUI but not yet planned over.

---

# 🗃️ TypeDB (`typedb_node`)

Uses the official `typedb.driver` client against `typedb_address` (default `localhost:1729`), database `greenhouse` (default). **It does not create the database** — it must already exist and be loaded (see the top-level repo README's TypeDB setup section using `typedb_files/greenhouse-schema.tql` and `greenhouse-data.tql`).

Parameters: `typedb_address` (`localhost:1729`), `database_name` (`greenhouse`), `map_id` (`greenhouse-map-001`), `robot_id` (`robot1`), `waypoints_file`, `reset_waypoints_on_start` (`false`).

| Direction | Topic / Service | Type |
|---|---|---|
| Sub | `flower_data` | `gh_twin_msgs/msg/Flower` |
| Sub | `pest_data` | `gh_twin_msgs/msg/Pest` |
| Sub | `/vision/scanned_tag` | `gh_twin_msgs/msg/Sensor` |
| Pub | `/greenhouse/telemetry` | `lupin_greenhouse_msgs/msg/TagReading` |
| Pub | `gui_flower_data` | `gh_twin_msgs/msg/Flower` |
| Pub | `gui_pest_data` | `gh_twin_msgs/msg/Pest` |
| Client | `/greenhouse_bridge/get_tag_reading` | `lupin_greenhouse_msgs/srv/GetTagReading` |

On startup it loads `waypoints_file` (YAML) into TypeDB as `waypoint` entities chained by `path-connection` relations, and seeds a `current-location` relation for `robot1` at `wp_0`. Incoming flower/pest detections within `0.04 m²` of a previous one are treated as duplicates and dropped (checked in-memory, not re-queried from TypeDB). Detections are mapped to the nearest stored waypoint by Euclidean distance before being written.

`lupin_greenhouse_msgs` comes from the external `lupin_greenhouse_ros_group17` repo pulled in via `repos.repos` — it isn't vendored in this package.

---

# 📐 PDDL planner (`pddl_planner_node`)

Can run standalone as a CLI or be embedded as a library (as `plan_executor_node` does):

```bash
ros2 launch gh_twin_data_storage pddl_planner.launch.py \
  domain_file:=/path/to/DomainGreenhouse.pddl \
  problem_file:=/path/to/problem_generated.pddl
```

* Queries TypeDB for the robot/arm pair, current waypoint, all waypoints, bin start/end pairs, connections, and plants/pests-with-waypoints, then assembles a PDDL `(:objects) (:init) (:goal)` block.
* `run_planner()` resolves the POPF binary from the `POPF_BIN` environment variable (falls back to `/opt/ros/humble/lib/popf/popf`), sets `LD_LIBRARY_PATH`, and shells out with a 60 s timeout. If the binary doesn't exist it logs an error and returns no plan rather than raising.
* `replan(...)` regenerates the problem file with the caller's current progress/blocklist state and reruns the planner — this is the single entry point both the CLI and `plan_executor_node`'s execution loop use.

> [!WARNING]
> The `--planner-cmd` / `planner_cmd` argument (CLI and launch file) is parsed but currently **unused** — `run_planner()` always shells out to `POPF_BIN`. Treat it as a reserved flag, not a working override.

See the top-level repo README's [Run the Autonomous Planner](../../README.md#5-run-the-autonomous-planner) section for how to build POPF from source via `pixi run fetch` / `pixi run build`.

---

# 🎯 Task scheduler (`plan_executor_node`)

The node that actually drives the robot. Operating mode is set externally via `std_msgs/Int32` on `/operating_mode` (published by the GUI):

| Mode | Meaning | Effect |
|---|---|---|
| `0` | Measurement Mode | Starts the plan-execution thread |
| `1` | Manual Mode | Aborts any running plan, waits |
| `2` | Pause Mission | Aborts the plan and cancels the current nav goal |
| `3` | Return Home | Navigates to `recharge_waypoint` |
| `4` | Reset | Clears replan state, re-syncs current location, resets arm to `base` |
| `5` | Shutdown | Calls `rclpy.shutdown()` |

Parameters: `domain_file` (`DomainGreenhouse.pddl`), `problem_file` (`problem_generated.pddl`), `map_frame` (`map`), `battery_topic` (`io/power/power_watcher`), `battery_threshold` (`0.25`), `battery_check_period` (`10.0`), `recharge_waypoint` (`wp_0`), `typedb_address` (`localhost:1729`), `database_name` (`greenhouse`), `operating_mode_topic` (`/operating_mode`), `robot_heartbeat_topic` (`/robot_heartbeat`), `arm_poses_file`, `max_replan_attempts` (`3`).

| Direction | Topic | Type |
|---|---|---|
| Sub | `battery_topic` | `sensor_msgs/msg/BatteryState` |
| Sub | `operating_mode_topic` | `std_msgs/msg/Int32` |
| Pub | `robot_heartbeat_topic` | `std_msgs/msg/Bool` (1 Hz) |

A battery watchdog checks `battery_topic` every `battery_check_period` seconds and triggers a return-to-recharge if the level drops below `battery_threshold`. If the robot is driven manually and needs to resume a plan, `update_current_location_to_typedb` inserts a synthetic waypoint at the robot's live pose, connected to its two nearest bin-start waypoints, and updates `current-location` — so replanning continues from wherever the robot actually is.

---

# 🚦 Launch files

| Launch file | Brings up | Key args (default) |
|---|---|---|
| `typedb.launch.py` | `typedb_node` (sim waypoints, `config/waypoints.yml`, `reset_waypoints_on_start:=true`) | — (no launch args, all params hardcoded) |
| `typedb_hardware.launch.py` | `typedb_node` (`config/waypoints_hardware.yaml`) | — |
| `pddl_planner.launch.py` | standalone `pddl_planner_node` | `domain_file`, `problem_file`, `planner_cmd` |
| `task_scheduler.launch.py` | `gh_twin`'s `nav.launch.py` + `typedb_node` + `plan_executor_node` (simulation) | `use_sim_time` (`true`), `domain_file`, `problem_file`, `typedb_address`, `battery_topic` (`/power/power_watcher`), `recharge_waypoint` (`wp_0`) |
| `task_scheduler_hardware.launch.py` | `gh_twin`'s `nav_hardware.launch.py` + `typedb_node` + `plan_executor_node` (physical robot) | Same as above, `use_sim_time:=false`, `battery_topic` (`io/power/power_watcher`) |

> [!NOTE]
> Both `task_scheduler*.launch.py` files declare a `mode` arg (`manual_slam`/`autonomous`) intended to optionally bring up SLAM, but the corresponding `slam_launch` action is currently commented out — `mode` has no runtime effect yet.

Run the full simulation task-scheduler flow (after [TypeDB is set up](../../README.md#4-set-up-typedb)):

```bash
ros2 launch gh_twin greenhouse_launch.xml          # terminal 1
ros2 launch gh_twin_data_storage task_scheduler.launch.py   # terminal 2
cd src/gh_twin/gui/ && python3 main.py             # terminal 3
```

Then select **Measurement Mode** in the GUI to start a plan.

---

# ⚙️ Config (`config/`)

* **`waypoints.yml`** — 80 simulation waypoints (`wp_0`–`wp_79`) across 8 bins, used by `typedb.launch.py`.
* **`waypoints_hardware.yaml`** — a 4-waypoint hardware stub (`bin_7` only), used by `typedb_hardware.launch.py`.
* **`waypoint_dummy.yaml`** — a larger 40-waypoint hardware reference set, not currently wired into any launch file.
* **`arm_poses.yml`** — joint-space presets for `ArmController`: `home`, `base`, `scan` (used by the scheduler), `scan_a` (an alternate scan orientation, not currently referenced by any node), plus `sequ_home`/`sequ_p1`/`sequ_p2` for the spray sequence.

---

# 📥 Install & Build

```bash
cd ~/ro47007_mirte_ws
pixi shell
colcon build --symlink-install --packages-select gh_twin_data_storage
source install/setup.bash
```

> [!WARNING]
> `package.xml` only declares `rclpy` as a dependency, even though the nodes import `gh_twin_msgs`, `lupin_greenhouse_msgs`, `gh_twin` (for `ArmController`/`VisionController`/`NavToPose`), the `typedb-driver` pip package, and `PyYAML`. These must be present in the Pixi environment; a bare `rosdep install` will not pull them in.

`console_scripts` (`ros2 run gh_twin_data_storage <name>`): `typedb_node`, `pddl_planner_node`, `plan_executor_node` (`sql_db_node`/`nosql_db_node` not yet implemented, see above).

---

# 📌 Related files at the repo root

* `DomainGreenhouse.pddl` — the PDDL domain (see [How planning works](#-how-planning-works-end-to-end)).
* `problem_generated.pddl` — the last generated problem file (runtime artifact, regenerated on every `replan()`).
* `typedb_files/greenhouse-schema.tql` — TypeDB schema (entities: `robot`, `arm`, `waypoint`, `plant`, `pest-detection`, `sensor-reading`, `april-tag`, ...; relations: `current-location`, `path-connection`, `object-location`, `observation-location`, ...).
* `typedb_files/greenhouse-data.tql` — minimal seed data (one `robot`, one `arm`, one `slam-map`); everything else (waypoints, plants, pests, readings) is inserted at runtime.
