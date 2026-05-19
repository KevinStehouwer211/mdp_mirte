# gh_twin

## Description
Greenhouse digital twin generation using autonomous robot


## Installation

Create a ROS workspace ~/ros2_ws/
```bash
mkdir -p ~/ros2_ws/src
```

Installation of the Mirte Packages in src folder of the workspace using the instructions in [Mirte Documentation](https://docs.mirte.org/0.2.0/doc/simulation/install_simulation.html). As mentioned in the documentation, it would be better to remove the unused pacakges.

Installation of gh_twin
```bash
cd ~/ros2_ws/src
git clone git@gitlab.tudelft.nl:cor/ro47007/2026/group_17/gh_twin.git
cd ~/ros2_ws
colcon build --symlink-install
```

Packages required for SLAM
```bash
sudo apt install ros-$ROS_DISTRO-slam-toolbox
```

Packages required for navigation
```bash
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup
sudo apt install xterm
```

Packages required for HMI

```bash
pip install nicegui
```
## Launching the Simulation

Launching gazebo and robot model
```bash
ros2 launch gh_twin greenhouse_launch.xml 
```
Launching the SLAM subsystem. SLAM need not be launched for testing out the navigation subsystem. Created map is stored in maps folder
```bash
ros2 launch gh_twin slam.launch.py
```

Launching the navigation subsystem
```bash
ros2 launch gh_twin nav.launch.py
```
After RViz is launched, set "2D Goal Pose" in RViz at the approximate pose of the robot as seen in Gazebo wrt the map. When the nodes are just launched, the position of the robot in RViz is same as the position of the robot in Gazebo. So "2D Goal Pose" can be set at the location seen in RViz. After that "Nav2 Goal" can be given at any desired location on the map and the robot should start moving.

## Development

New subsystem implementations can be added to gh_twin folder