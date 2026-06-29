# 🌷 Flower Perception

ROS 2 package for flower and pest detection using YOLOv8.

The package subscribes to the gripper camera image topic, performs YOLO inference, and publishes detections to a ROS topic.

---

# 📦 Features

* YOLOv8 flower and pest detection
* Per-flower tracking (stable track id across frames)
* 3D flower localization (x, y, z) via **multi-view** triangulation — no LiDAR, no height prior
* Live 3D bounding-box visualization projected back onto the camera image
* ROS 2 Humble compatible
* Compressed image support
* Real-time inference
* Validation and training scripts
* Hugging Face dataset integration
* Pixi environment support

---

# 🚀 Quick Start

After cloning the workspace and installing the Pixi environment:

```bash
cd ~/ro47007_mirte_ws

colcon build --symlink-install --packages-select flower_perception

source install/setup.bash
```

Run the detector:

```bash
ros2 run flower_perception flower_detector
```

Or launch the full live pipeline (tracker + triangulation + RViz + rqt, no rosbag):

```bash
ros2 launch flower_perception flower_perception.launch.py
```

See [3D Triangulation](#-3d-triangulation) below for how to obtain x, y, z.

---

# 🤖 ROS 2 Detector Node

The detector node:

* subscribes to:

```text
/gripper_camera/image_raw/compressed
```

* performs YOLO inference

* publishes detections to:

```text
/flower_perception
```

using:

```text
vision_msgs/msg/Detection2DArray
```

---

# 🏆 Included Model

A pretrained YOLOv8 model is already included:

```text
~/ro47007_mirte_ws/src/flower_perception/models/best.pt
```

No additional training is required to run the detector.

---

# 👀 View Detections

```bash
ros2 topic echo /flower_perception
```

Check topic type:

```bash
ros2 topic info /flower_perception
```

Expected type:

```text
vision_msgs/msg/Detection2DArray
```

---

# 🎯 Recommended Confidence Threshold

Based on the validation F1-confidence curve:

```text
0.43 - 0.45
```

is recommended for deployment.

---

# 📐 3D Triangulation (multi-view)

The `flower_triangulation` node turns 2D detections into a **3D position (x, y, z)** for
each flower — no LiDAR and no height prior. As the camera **moves**, every detection
gives one ray; the node accumulates **many** rays per flower and solves for the point
where they best intersect (an N-ray least-squares closest point). So the estimate
averages out per-frame noise and **refines live** as you sweep the camera.

Flowers are matched across views by the **track id** the `flower_detector_tracker`
assigns (`class_id = "<class>_track_<id>"`), so the same physical flower is paired
automatically. Bad views (a wrong arm pose, a tracker id swap) are **rejected by
residual** before re-solving, so the result is robust.

## Requirements

* The **detector tracker** must be running (publishes `/flower_perception`).
* TF must provide `odom → wrist_camera` (the camera pose in a fixed world frame).
* The camera must **translate** while accumulating (≥ a few cm; a lateral sweep up to
  ~20 cm works well). Moving the **arm** or the **base** both work; pure rotation does
  **not** (no parallax).

## Episodes (one per spot)

Each location is one **episode**, driven by `std_srvs/srv/Trigger` services. While
**idle** the node accumulates nothing, so driving between spots doesn't pollute the
buffer.

| Service | Effect |
|---|---|
| `/flower_triangulation/start`    | Clear buffers and begin accumulating views |
| `/flower_triangulation/finish`   | Solve once, publish the final result, go idle |
| `/flower_triangulation/reset`    | Abort the episode (clear, idle, publish nothing) |
| `/flower_triangulation/snapshot` | Debug: log the current x,y,z table mid-sweep |

### 1. Launch the pipeline

```bash
ros2 launch flower_perception flower_perception.launch.py
```

This starts the tracker, the triangulation node, the 3D-box visualization, RViz and rqt.

### 2. Run an episode

```bash
# arm at pose 1 -> begin accumulating
ros2 service call /flower_triangulation/start std_srvs/srv/Trigger {}

# ... sweep the camera to pose 2 (<= ~20 cm); the 3D boxes appear and tighten LIVE ...

# done sweeping -> solve, publish the final result, go idle
ros2 service call /flower_triangulation/finish std_srvs/srv/Trigger {}
```

Then drive to the next spot and call `start` again. Set `auto_start:=true` to keep the
node always-on (e.g. for quick testing without the start/finish handshake).

## Output topics

Both are `vision_msgs/msg/Detection3DArray`. Per flower: `bbox.center.position` =
**x, y, z** (in `odom`), `results[0].hypothesis.class_id` = **class**, `id` = **track id**.

| Topic | When | Use |
|---|---|---|
| `/flower_triangulation/detections` | live, 5 Hz during a sweep | visualization |
| `/flower_triangulation/result`     | once, on `finish` (**latched**) | downstream consumer |

The downstream node must subscribe to `/result` with **`TRANSIENT_LOCAL`** durability,
or the latched message won't be delivered:

```bash
ros2 topic echo /flower_triangulation/result --qos-durability transient_local
```

## Tips

* **No box for the first cm or two of motion is normal** — not enough parallax yet
  (`min_parallax`). Lower it to show a rough box sooner.
* **Wider, lateral baseline = more accurate.** Sweep sideways across the bin.
* **Short sweeps:** lower `min_view_separation` for denser views. **Long sweeps:** raise
  `max_views` so the rolling window keeps the full baseline instead of evicting old views.
* Spurious low points are dropped by the `min_z` floor.

Key parameters (defaults shown):

```text
detections_topic    : /flower_perception
camera_frame        : wrist_camera
base_frame          : odom        # frame the x,y,z are expressed in
min_view_separation : 0.02        # m; store a new view every this much camera travel
max_views           : 40          # rolling ray-buffer length per flower
min_views           : 3           # min rays before a flower is published
min_parallax        : 0.01        # min motion spread before a flower is solved
max_gap             : 0.05        # m; reject a ray missing the point by more (outlier)
track_timeout       : 10.0        # s; forget a flower unseen this long
publish_rate        : 5.0         # Hz; live solve/publish rate
min_z               : 0.16        # m; reject points below this height in base_frame
auto_start          : false       # true = accumulate without waiting for /start
```

---

# 🧊 3D Bounding-Box Visualization

The `flower_bbox_projection` node projects the triangulated 3D detections back onto the
**clean camera image** as upright wireframe 3D boxes, so you can visually confirm the
triangulation lands on the flower (and that the `odom → wrist_camera` TF is consistent).

* Subscribes to the raw camera (`/gripper_camera/image_raw/compressed`), the 2D
  detections (`/flower_perception`) and the 3D detections
  (`/flower_triangulation/detections`).
* Each box is sized from the matching **2D box** (metric, back-projected at the
  detection's depth), kept **axis-aligned in `odom`** (upright, parallel to the ground),
  and labelled with **class, track id and x, y, z**.
* Publishes the overlay on:

```text
/flower_perception/debug_image/bbox_projected
```

View it live (updates during the sweep):

```bash
ros2 run rqt_image_view rqt_image_view /flower_perception/debug_image/bbox_projected
```

---

# 🎥 Visualize Detection Results with a Rosbag

The detector can be tested using a recorded rosbag.

The rosbag must contain the compressed camera topic:

```text
/gripper_camera/image_raw/compressed
```

---

## 1. Record a Rosbag

First, record a rosbag while the gripper camera is publishing images:

```bash
cd ~/ro47007_mirte_ws/src/flower_perception/datasets/rosbags

ros2 bag record /gripper_camera/image_raw/compressed -o gripper_rec
```

Stop recording with:

```text
Ctrl+C
```

This creates:

```text
datasets/rosbags/gripper_rec/
```

---

## 2. Add the Rosbag to the Repository

Make sure the recorded rosbag folder is stored inside:

```text
src/flower_perception/datasets/rosbags/
```

Example structure:

```text
flower_perception/
└── datasets/
    └── rosbags/
        └── gripper_rec/
            ├── metadata.yaml
            └── ...
```

---

## 3. Check Rosbag Info

```bash
cd ~/ro47007_mirte_ws/src/flower_perception/datasets/rosbags

ros2 bag info gripper_rec
```

Verify that the bag contains:

```text
/gripper_camera/image_raw/compressed
```

---

## 4. Play Rosbag

Open a terminal and run:

```bash
cd ~/ro47007_mirte_ws/src/flower_perception/datasets/rosbags

ros2 bag play gripper_rec --loop
```

---

## 5. Run Detector Node

Open a second terminal and run:

```bash
cd ~/ro47007_mirte_ws

source install/setup.bash

ros2 run flower_perception flower_detector
```

The detector subscribes to the rosbag image topic and publishes:

```text
/flower_perception
/flower_perception/debug_image
```

---

## 6. Check Published Topics

```bash
ros2 topic list
```

Expected topics:

```text
/gripper_camera/image_raw/compressed
/flower_perception
/flower_perception/debug_image
```

Check if the debug image is being published:

```bash
ros2 topic hz /flower_perception/debug_image
```

---

## 7. Visualize with rqt

Start `rqt`:

```bash
rqt
```

In the GUI:

```text
Plugins → Visualization → Image View
```

Select:

```text
/flower_perception/debug_image
```

If the topic does not appear, restart `rqt` after the detector is running.

---

## Notes

* A rosbag must first be recorded and placed in `datasets/rosbags/`.
* `/flower_perception` contains the detection messages.
* `/flower_perception/debug_image` contains the camera image with YOLO bounding boxes drawn on top.
* `rqt` is usually easier for quick image debugging.
* RViz is useful when combining detection output with the rest of the robot visualization.


# 🚀 Selected Model

YOLOv8n was selected because it achieved:

* higher mAP
* faster inference
* better real-time performance

than YOLO26n on this dataset.

---

# 📌 Notes

* Use `--symlink-install` during development.
* CPU inference works correctly on the AMD Ryzen AI MAX platform.
* YOLO training on CPU is significantly slower than CUDA GPU training.
* The detector node uses compressed images directly to reduce bandwidth.

---

# 🧠 Optional — Retrain the Model

## 🔗 Dataset

Dataset source:

```text
https://huggingface.co/datasets/jihyopark/awesome-flowers
```

The dataset is cloned automatically through `repos.repos`.

---

# ⚠️ IMPORTANT — Git LFS Required

The dataset images are stored using Git LFS.

Without Git LFS, the image files become small pointer files and YOLO training will fail.

Git LFS is already included through:

```toml
git-lfs = "*"
```

inside `pixi.toml`.

After installing the Pixi environment and importing repositories, run:

```bash
cd ~/ro47007_mirte_ws/src/flower_perception/datasets/awesome-flowers

git lfs pull
```

---

# ✅ Verify Dataset Images

Verify that the actual image files are downloaded correctly:

```bash
file images/group30_085.jpg
```

Expected output:

```text
JPEG image data
```

Incorrect output:

```text
ASCII text
```

If `ASCII text` appears, Git LFS did not download the real image files.

---

# 📁 Dataset Structure

The dataset must follow the YOLO format:

```text
awesome-flowers/
├── images/
│   ├── train/
│   └── val/
├── labels/
│   ├── train/
│   └── val/
└── data.yaml
```

---

# 📝 data.yaml

```yaml
train: images/train
val: images/val

nc: 4

names:
  - tulip_red
  - tulip_white
  - tulip_pink
  - bug
```

---

# 🧠 Training

Training script:

```text
training/train_flower_detector.py
```

Run training:

```bash
cd ~/ro47007_mirte_ws/src/flower_perception/training

python train_flower_detector.py
```

Training results are saved in:

```text
~/ro47007_mirte_ws/src/flower_perception/runs/flower_detection/
```

---

# 📊 Validation

Validation script:

```text
training/validate_flower_detector.py
```

Run validation:

```bash
cd ~/ro47007_mirte_ws/src/flower_perception/training

python validate_flower_detector.py
```

Validation results are saved in:

```text
~/ro47007_mirte_ws/src/flower_perception/runs/flower_validation/
```

---

# 🏆 Update Best Model

Copy the best trained model to the models folder:

```bash
cp ~/ro47007_mirte_ws/src/flower_perception/runs/flower_detection/yolov8n_flowers/weights/best.pt \
   ~/ro47007_mirte_ws/src/flower_perception/models/best.pt
```
