# gh_twin_data_storage

ROS2 package for greenhouse data storage nodes.

> This package is part of a Pixi-managed workspace. Run `pixi shell` from the `mdp_mirte_ws` root before building or launching.

## Purpose

This package provides typed storage nodes for greenhouse digital twin data. It currently includes:

- `typedb_node`: in-memory typed data store for plants, pests, and sensors
- `sql_db_node`: placeholder node for relational/SQL storage
- `nosql_db_node`: placeholder node for schemaless/NoSQL storage

## Messages

- `Plant` - plant or flower instance with `id`, `location`, `color`, `height`, `is_flower`
- `Pest` - pest instance with `id`, `location`, and `sprayed`
- `Sensor` - sensor instance with `id`, `location`, and repeated `SensorData`
- `SensorData` - single sensor reading with `type`, `value`, and `unit`

## Build

From the workspace root, first enter the Pixi shell and then build the package:

```bash
cd /home/marcel/Git/mdp_ws/src/mdp_mirte_ws
pixi shell
colcon build --symlink-install --packages-select gh_twin_data_storage
```

## Launch

Start the typedb node with:

```bash
ros2 launch gh_twin_data_storage typedb.launch.py
```
## PDDL Manager Node

Use the PDDL manager node to initialize from existing `world.pddl` and `problem.pddl` files and to update the problem definition from typedb data.

```bash
ros2 launch gh_twin_data_storage pddl_manager.launch.py \
	world_file:=/path/to/world.pddl \
	problem_file:=/path/to/problem.pddl
```

Once the node is running, call its `replan()` method in code or configure an external planner using `--planner-cmd`.
