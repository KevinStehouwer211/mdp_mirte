# MIRTE ROS2 Workspace for RO47007 with Pixi


## building

If you don't have Pixi, install it: [Pixi: Installation](https://pixi.prefix.dev/latest/installation/).

Then, in a linux terminal:

```shell
# We need xterm installed
sudo apt install xterm

# clone this repository somewhere. Here I do it in $HOME, but that is not
# required. The workspace could be anywhere
git clone git@gitlab.tudelft.nl:cor/ro47007/2026/group_17/mdp_mirte_ws.git $HOME/ro47007_mirte_ws
cd $HOME/ro47007_mirte_ws

# let pixi do its thing (this could take a while)
pixi install
```

if there were no errors, try fetching the packages (or `pixi run fetch` from within `ro47007_mirte_ws`):

```shell
pixi run vcs import --input $HOME/ro47007_mirte_ws/repos.repos $HOME/ro47007_mirte_ws/src
```

No errors? Ignore some unneeded packages:

```shell
touch src/mirte-ros-packages/mirte_{bringup,telemetrix_cpp,teleop,test,zenoh_setup}/COLCON_IGNORE
```

No errors? Try building the workspace, first enter the pixi shell which contains all the installed dependencies:

```shell
pixi shell
```

Next, build the workspace:

```shell
colcon build
```

note: depending on the machine this runs on, it can take quite some time.

If you want to clean the build artifacts (build, install, log directories) from within the pixi shell, you can use:
```shell
pixi run ws-clean
```

To clean first and then build, use:
```shell
pixi run clean-build
```

## running

If the build was successful, try starting the greenhouse simulation launch file (in the same `pixi shell` session):

```shell
source install/setup.bash
ros2 launch gh_twin greenhouse_launch.xml
```

this should start up only Gazebo with the MIRTE Master in the greenhouse environment.

To teleop the robot, try starting the teleop node (from another `pixi shell` session):
```shell
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/mirte_base_controller/cmd_vel_unstamped
```

To use slam, combined with teleop, try launching the slam launch file. (Make sure the teleop node is closed since this slam launch will use its own teleop node.)

```shell
source install/setup.bash
ros2 launch gh_twin slam.launch.py
```


## working with the physical robot

Working with a physical MIRTE Master robot and your pixi env is straightforward via ROS2. First, connect to the robot either via the Wifi AP or an ethernet cable. 

You should now be able to see the topics of the robot:
```shell
ros2 topic list
```

note: every new pixi session requires you to set the domain id.


## troubleshooting

* **problems with Python when building**: Make sure that you fully deactivate Anaconda or other virtual Python environment managers when trying to set up this repository. The symbolic links may interfere with Pixi and cause problems when compiling your workspace.

* **Shells other than Bash** If you are using shells other than Bash, e.g., ZSH, make sure to source the correct ROS2 setup file in the folder `install`, e.g., `setup.zsh`.

* **Pixi environment problems**: If you encounter Pixi environment problems, you can rebuild your Pixi environment by removing the environment using `pixi clean` and reinstalling it. 

* **Missing packages in Pixi**: This Pixi environment configuration does not provide all ROS2 packages you would find in a ROS2 Desktop Full Install. You can install additional packages using Pixi's install routine, e.g. to install turtle sim use `pixi add ros-humble-turtlesim`. This will install the package and automatically add it to your Pixi manifest file `pixi.toml`. Note: not all packages might be available via Pixi for your system. See the [Pixi documentation](https://pixi.prefix.dev/latest/tutorials/ros2/) for more information.

* **Inconsistent results across your group?** Make sure to use the same configuration of your workspace. Look at `git diff` to see changes. Moreover, we recommend using Pixi's lock file `pixi.lock`. This file pins the exact versions of dependencies installed in your Pixi environment. You can share this lock file within your group to detect inconsistencies and install the same dependency versions across your group.


## TypeDB setup

For the autonomous planning and executing of actions, the TypeDB knowledge base must be set up to store environment data, which PDDL can use for planning.

In a new terminal: 

```shell
typedb server
```

Then again in a new terminal:

```shell
typedb console --core=localhost:1729
```

```shell
database create greenhouse
```

Load the schema into the database:

```shell
transaction greenhouse schema write
source typedb_files/greenhouse-schema.tql
commit
```

Load the data into the database:

```shell
transaction greenhouse data write
source typedb_files/greenhouse-data.tql
commit
```

To check if this went ok you can use the following commands in the same console:

```shell
database schema greenhouse
```

The database should now be set up correctly. You only need to go back into the TypeDB console in the future if you want to write or read from the database manually.

NOTE: Everytime you make use of a function that uses TypeDB, you must be connected to the TypeDB server through the "typedb server" terminal.

## Running the planner

The waypoints must be put in /gh_twin_data_storage/config/waypoint.yml

With the "typedb server" open in one terminal, open a terminal to run the greenhouse simulation:

```shell
source install/setup.bash
ros2 launch gh_twin greenhouse_launch.xml
```

And a terminal to run the planner.

```shell
source install/setup.bash
ros2 launch gh_twin_data_storage task_scheduler.launch.py
```

And the gui:

```shell
cd src/gh_twin/gui/
python3 main.py
```
Then in the gui select "measurement mode", and the planner should start finsing a plan.
















































