# Running and Accessing the GUI with Simulation

## Quick Start

### Terminal 1: Start the Pixi Shell
```bash
cd ~/Git/mdp_ws/src/mdp_mirte_ws
pixi shell
```

### Terminal 2: Start TypeDB
TypeDB must be running before starting the simulation and GUI nodes.
```bash
pixi shell
typedb server
```

The server will start and output:
```
Storage directory: ~/.typedb/data
Server is now running. Listening to connections on localhost:1729
```

Keep this terminal open while using the system.

### Terminal 3: Start Gazebo Simulation
```bash
pixi shell
source install/setup.bash
ros2 launch gh_twin greenhouse_launch.xml
```

This launches:
- Gazebo with the MIRTE Master robot in the greenhouse environment
- The greenhouse world with plant beds and sensors

### Terminal 4: Start Data Storage Nodes
Starts TypeDB subscriber node and PDDL planner:
```bash
pixi shell
source install/setup.bash
ros2 launch gh_twin_data_storage task_scheduler.launch.py
```

This launches:
- **typedb_node**: Subscribes to `flower_data`, `pest_data`, `sensor_data` topics and stores data in TypeDB
- **plan_executor_node**: Executes PDDL plans using Nav2
- **pddl_planner_node**: Generates PDDL problems from TypeDB state

### Terminal 5: Start Navigation
```bash
pixi shell
source install/setup.bash
ros2 launch gh_twin nav.launch.py
```

This launches:
- **RViz**: Visualization of the map and robot
- **Nav2 Stack**: Navigation subsystem for autonomous movement

**Important**: After RViz opens, use "2D Goal Pose" tool to initialize the robot's position on the map. Click on the robot's approximate location in RViz. After initialization, you can use "Nav2 Goal" to send the robot to navigation targets.

### Terminal 6: Start the GUI Server
```bash
cd ~/Git/mdp_ws/src/mdp_mirte_ws/src/gh_twin/gui
pixi shell
python main.py
```

Expected output:
```
Started server process [PID]
Uvicorn running on http://127.0.0.1:8000
```

## Accessing the GUI

### Web Browser Access
Open your web browser and navigate to:
```
http://localhost:8000
```

Or from another machine on the network:
```
http://<your-machine-ip>:8000
```

### GUI Features

#### Login Page
- **Username**: Any of the users in `src/gh_twin/gui/users.txt`
- **Password**: (Check the users.txt file for credentials)

#### Dashboard Features

**Left Sidebar:**
- Robot Status (Online/Offline indicator)
- Operating Mode Selection (Measurement, Manual, Pause Mission, Return Home, Reset, Shutdown)
- Battery Level (visual progress bar)
- Camera Feed Selection (Camera selector dropdown + live video stream)
- Save Data button
- Logout button

**Main Map View:**
- Greenhouse map with plant beds
- Plant locations with bloom status (color-coded flowers, animated blooming)
- Pest locations (bug icons)
- Sensor locations (sensor icons with interactive charts)
- Robot position (blue marker showing current location and orientation)
- Click anywhere on the map to send the robot to that location (in Measurement Mode)

**Interactive Elements:**
- Click on a plant to see ID and bloom status
- Click on a pest to navigate to it for "Kill Bug" action
- Click on a sensor to see historical data in a time-series chart
- Update in real-time as new Flower, Pest, and Sensor messages arrive

## System Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    Gazebo Simulation                        │
│                   + Robot + Environment                     │
└─────────────────────────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 Topics                              │
│  flower_data, pest_data, sensor_data                       │
│  /amcl_pose, /battery_status, etc.                         │
└─────────────────────────────────────────────────────────────┘
                           ↓
         ┌─────────────────┬─────────────────┐
         ↓                 ↓                 ↓
    ┌────────────┐  ┌────────────┐  ┌────────────┐
    │ typedb_node│  │  Nav2 Stack│  │  RViz2     │
    │  Subscriber│  │ Navigation │  │ Visualizer │
    └────┬───────┘  └────────────┘  └────────────┘
         ↓
    ┌──────────────┐
    │   TypeDB     │
    │  Knowledge   │
    │   Graph      │
    │ localhost:17│
    │     29      │
    └──────────────┘
         ↑
         │
    ┌──────────────────┐
    │   GUI Server     │
    │  NiceGUI + ROS2  │
    │  localhost:8000  │
    └──────────────────┘
         ↑
         │
    ┌──────────────────┐
    │  Web Browser     │
    │  Dashboard View  │
    └──────────────────┘
```

## Data Flow

### Real-time Updates to GUI

1. **Sensors detect entities** → Messages published on ROS2 topics
2. **typedb_node receives messages** → Stores data in TypeDB
3. **GUI subscriptions receive messages** → Update in-memory data structures
4. **GUI timer (0.1s)** → Renders updated data on map
5. **Browser display** → Shows real-time greenhouse state

### Entity Message Types

**Flower Message** → Plant/Bloom status
- ID, location, color, bloom status, robot pose

**Pest Message** → Bug detection
- ID, location, spray status

**Sensor Message** → Environmental readings
- ID, location, sensor type, readings (type, value, unit)

## Troubleshooting

### GUI Won't Start
```bash
# Make sure you're in the correct directory
cd ~/Git/mdp_ws/src/mdp_mirte_ws/src/gh_twin/gui

# Check Python dependencies
pip list | grep nicegui

# If NiceGUI is missing, install via pixi
pixi install
```

### Can't Access Localhost:8000
- Ensure firewall allows port 8000
- Check if GUI process is running: `ps aux | grep main.py`
- Check for port conflicts: `lsof -i :8000`

### TypeDB Connection Errors
- Ensure TypeDB server is running in Terminal 2
- Check TypeDB is accessible: `netstat -tuln | grep 1729`
- Verify TypeDB database "greenhouse" exists

### Robot Appears Offline
- Check robot pose subscriber (`/amcl_pose` topic)
- Verify Nav2 is initialized (set "2D Goal Pose" in RViz)
- Check heartbeat timeout (currently 15 seconds)

### GUI Data Not Updating
- Verify ROS2 topics are publishing: `ros2 topic list | grep -E "(flower|pest|sensor)_data"`
- Check GUI subscriptions are created: Look for callback messages in terminal
- Verify typedb_node is running: `ros2 node list | grep typedb`

## Advanced Usage

### Change GUI Port
Edit `src/gh_twin/gui/main.py`:
```python
ui.run(
    title='Greenhouse Robot HMI',
    storage_secret='mm_group17',
    reload=False,
    port=8080  # Change this
)
```

### Monitor ROS2 Topics
In a new terminal:
```bash
pixi shell
source install/setup.bash
ros2 topic echo /amcl_pose
ros2 topic echo flower_data
ros2 topic echo pest_data
ros2 topic echo sensor_data
```

### Access TypeDB Data
In a new terminal:
```bash
pixi shell
typedb console greenhouse
```

Then run TypeQL queries:
```
match $wp isa waypoint; get $wp;
match $plant isa plant; get $plant;
match $pest isa pest-detection; get $pest;
```

### Monitor Data Storage Logs
```bash
pixi shell
source install/setup.bash
ros2 node list
ros2 node info /typedb_storage_node
```

## Performance Tips

1. **Gazebo might be slow**: Reduce physics simulation rate in greenhouse.world
2. **GUI lag**: Close other browser tabs, check network connectivity
3. **ROS2 latency**: Reduce sensor message publication rates if needed
4. **TypeDB queries**: Index frequently queried attributes for faster lookups

## System Requirements

- Linux (Ubuntu 22.04 recommended)
- 4GB RAM minimum (8GB+ recommended for Gazebo)
- Modern web browser (Chrome, Firefox, Safari)
- Network connection (for accessing GUI from other machines)

## Security Note

The GUI login is basic and for demonstration. For production use:
- Implement proper authentication (OAuth, SAML, etc.)
- Use HTTPS instead of HTTP
- Restrict network access to authorized users
- Update credentials regularly
