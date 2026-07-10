# ✉️ gh_twin_msgs

ROS 2 interface package: the custom message definitions shared across the greenhouse digital twin (`gh_twin`, `gh_twin_data_storage`, and the operator GUI).

---

# 📦 Messages

### `Flower.msg`

```
uint64 id
geometry_msgs/Point location
string color
bool bloomed
geometry_msgs/Pose robot_pose
```

A detected flower instance. `location` is the flower's estimated position (in the `odom`/map frame); `robot_pose` is where the robot was when the detection was made. Published by `gh_twin_data_storage`'s `typedb_node` on `gui_flower_data` for the operator GUI, and consumed on `flower_data` for storage into TypeDB.

### `Pest.msg`

```
uint64 id
geometry_msgs/Point location
bool sprayed
geometry_msgs/Pose robot_pose
```

A detected pest instance. `sprayed` tracks whether the pest has already been treated. Published on `gui_pest_data` for the GUI, consumed on `pest_data` for storage.

### `Sensor.msg`

```
uint64 id
geometry_msgs/Point location
string tag_id
geometry_msgs/Pose robot_pose
```

A scanned sensor/AprilTag reading location. `tag_id` identifies the physical tag (set by `gh_twin`'s AprilTag detector); `id` is a repeated numeric identifier for the reading. Published on `/vision/scanned_tag`.

---

# 📥 Install & Build

```bash
cd ~/ro47007_mirte_ws
pixi shell
colcon build --symlink-install --packages-select gh_twin_msgs
source install/setup.bash
```

This is a pure `rosidl` interface package (`ament_cmake` + `rosidl_default_generators`) — it has no nodes or launch files. Depends on `geometry_msgs` for `Point`/`Pose`.
