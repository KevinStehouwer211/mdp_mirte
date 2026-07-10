# 🔊 mirte_indicator_devices

ROS 2 package for MIRTE Master's physical feedback devices: an audio node that plays sound effects, and an LED controller that drives a NeoPixel strip, both reacting to robot state, operating mode, and wheel motion.

> [!WARNING]
> This package is **not yet integrated** with the rest of the workspace: no other package currently publishes to the `robot_state` / `operation_mode` topics it listens on, and it is not included by any launch file outside its own. It targets `mirte_msgs` topics/services that, per the package's own commit history, are expected to exist "on the robot" but are unverified against the `mirte_msgs` version vendored in this workspace. Treat it as ready-to-wire-up rather than a working end-to-end feature yet.

---

# 📦 What's in here

| Node | Executable | Role |
|---|---|---|
| `RobotAudioNode` | `audio_node` | Plays WAV sound effects on error state and on movement |
| `LedControllerNode` | `led_controller_node` | Drives a NeoPixel LED strip to reflect state/mode, blinking while moving |

---

# 🔈 Audio node (`audio_node`)

Node name: `mirte_audio_node`.

| Direction | Topic | Type |
|---|---|---|
| Sub | `robot_state` | `std_msgs/msg/String` |
| Sub | `/io/motor/front_left/speed` | `mirte_msgs/msg/SetSpeed` |
| Sub | `/io/motor/front_right/speed` | `mirte_msgs/msg/SetSpeed` |
| Sub | `/io/motor/rear_left/speed` | `mirte_msgs/msg/SetSpeed` |
| Sub | `/io/motor/rear_right/speed` | `mirte_msgs/msg/SetSpeed` |

* While `robot_state.data` (case-insensitive) is `"error"`, `sounds/error_sound.wav` is replayed every **5 seconds** by a timer, not just once.
* When any wheel's `|speed|` crosses above `speed_threshold` (`0.5`) while the robot was previously stationary, `sounds/move_sound.wav` plays once (rising-edge trigger only — no sound on stopping).
* Playback runs through `aplay -D hw:1 <file>` as a non-blocking `subprocess.Popen`, so a slow/missing ALSA device won't block the node.
* `sounds/error_maybe.wav` and `sounds/error_no1.wav` are packaged alongside but not currently referenced by any code — alternate error-sound candidates kept for future use.

---

# 💡 LED controller (`led_controller_node`)

Node name: `led_controller_node`.

| Direction | Topic / Service | Type |
|---|---|---|
| Sub | `robot_state` | `std_msgs/msg/String` |
| Sub | `operation_mode` | `std_msgs/msg/String` |
| Sub | `/io/motor/{front,rear}_{left,right}/speed` | `mirte_msgs/msg/SetSpeed` |
| Client | `/io/leds/leds/set_color` | `mirte_msgs/srv/SetNeopixel` |

Color priority (highest first), re-evaluated on every relevant callback:

1. `robot_state == "error"` → **red**
2. `operation_mode == "move_manual"` → **blue**
3. `operation_mode == "automatic"` → **green**
4. otherwise → **off**

If any wheel speed is above `speed_threshold` (`0.5`), the chosen color blinks on/off at 1 Hz instead of staying solid. The service call is skipped (with a debug log, not an error) if `/io/leds/leds/set_color` isn't available within 0.1 s — so a missing LED driver won't block the node. On shutdown, the node makes a best-effort attempt to turn the LEDs off.

---

# 🚦 Launch

```bash
ros2 launch mirte_indicator_devices mirte_feedback_nodes.launch.py
```

Starts both `audio_node` (renamed `robot_audio_node`) and `led_controller_node` with `output='screen'`. No launch arguments are exposed — all thresholds, colors, and device paths are hardcoded in the node source.

---

# 📥 Install & Build

```bash
cd ~/ro47007_mirte_ws
pixi shell
colcon build --symlink-install --packages-select mirte_indicator_devices
source install/setup.bash
```

> [!IMPORTANT]
> `package.xml` only declares `rclpy` and `std_msgs` as dependencies, but both nodes import from `mirte_msgs.msg` / `mirte_msgs.srv` — make sure `mirte_msgs` is available in your environment even though `rosdep`/colcon dependency resolution won't pull it in automatically for this package.

`console_scripts` (`ros2 run mirte_indicator_devices <name>`): `audio_node`, `led_controller_node`.

Requires ALSA (`aplay`) for sound playback and a NeoPixel-capable LED service (`/io/leds/leds/set_color`) provided by the robot's driver stack for the LED node — neither is provided by this package itself.
