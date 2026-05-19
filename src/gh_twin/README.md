# gh_twin

## Description
Greenhouse digital twin generation using autonomous robot

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