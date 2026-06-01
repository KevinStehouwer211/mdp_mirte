# GUI Data Flow Architecture

## Overview

The greenhouse GUI displays real-time data about plants, pests, and sensors. This data originates from the TypeDB database and is transmitted to the GUI through ROS2 messages.

## Data Flow Diagram

```
TypeDB (Knowledge Graph)
  ↓
typedb_node.py (Subscribers)
  ├→ flower_callback() → publishes on 'flower_data' topic
  ├→ pest_callback() → publishes on 'pest_data' topic
  └→ sensor_callback() → publishes on 'sensor_data' topic
  
  ↓
ROS2 Message Topics
  ├→ 'flower_data' (Flower messages)
  ├→ 'pest_data' (Pest messages)
  └→ 'sensor_data' (Sensor messages)
  
  ↓
dashboard_page.py (GUI Subscribers)
  ├→ flower_callback_gui() → updates plants[] list
  ├→ pest_callback_gui() → updates bugs[] list
  └→ sensor_callback_gui() → updates sensors[] list
  
  ↓
GUI Display (dashboard_page.py)
  ├→ Plants with bloomed status and color
  ├→ Bugs/Pests with location
  └→ Sensors with type and historical data charts
```

## Data Storage

### 1. TypeDB (Primary Storage)
- **Location**: `localhost:1729` 
- **Database**: `greenhouse`
- **Schema**: `typedb_files/greenhouse-schema.tql`

Stores:
- **Plants**: id, scan-status, location waypoint, observation-location
- **Pests**: id, detection-status, spray-status, location waypoint
- **Sensors**: scan-events, sensor-readings with type, value, unit
- **Waypoints**: id, x, y, yaw, path connections

### 2. ROS2 Message Types

#### Flower.msg
```
uint64 id
geometry_msgs/Point location
string color
bool bloomed
geometry_msgs/Pose robot_pose
```

**GUI Requirements Met:**
- `id` → Used as plant ID (P1, P2, etc.)
- `color` → Display color of flower
- `bloomed` → Show bloom status
- `robot_pose.position` → Map coordinates for placement

#### Pest.msg
```
uint64 id
geometry_msgs/Point location
bool sprayed
```

**GUI Requirements Met:**
- `id` → Used as bug ID (B1, B2, etc.)
- `location` → Map coordinates for placement

#### Sensor.msg (NEW - sensor_type added)
```
uint64 id
geometry_msgs/Point location
string sensor_type
SensorData[] readings
```

**GUI Requirements Met:**
- `id` → Used as sensor ID (S1, S2, etc.)
- `location` → Map coordinates for placement
- `sensor_type` → Display sensor type (e.g., "Humidity", "Temperature")
- `readings[].value` → Extract numerical values for chart data

#### SensorData.msg
```
string type
float32 value
string unit
```

### 3. GUI Memory Storage

Global variables in `dashboard_page.py`:

```python
plants = [
    {'id': 'P1', 'x': 170, 'y': 105, 'bloomed': True, 'color': 'White'},
    {'id': 'P2', 'x': 470, 'y': 485, 'bloomed': True, 'color': 'Pink'},
    ...
]

bugs = [
    {'id': 'B1', 'x': 180, 'y': 235},
    {'id': 'B2', 'x': 470, 'y': 150},
    ...
]

sensors = [
    {'id': 'S1', 'x': 1070, 'y': 180, 'type': 'Humidity', 'data': [22, 22.4, 22.8]},
    {'id': 'S2', 'x': 770, 'y': 120, 'type': 'Temperature', 'data': [24, 24.5, 25]},
    ...
]
```

## Data Flow Details

### Plant (Flower) Data Flow

1. **TypeDB Storage** → Stores plant observations with waypoints
2. **typedb_node.flower_callback()** receives messages and stores to TypeDB
3. **typedb_node publishes** Flower message to `flower_data` topic with:
   - Plant ID
   - Location
   - Color
   - Bloomed status
   - Robot pose at time of detection
4. **dashboard_page.flower_callback_gui()** receives message and:
   - Extracts robot_pose as x, y coordinates
   - Creates plant entry: `{'id': f'P{msg.id}', 'x': ..., 'y': ..., 'bloomed': ..., 'color': ...}`
   - Updates existing plant or adds new one to `plants[]` list
5. **GUI Timer** (0.1s interval) renders plants on map with:
   - Colored flower image based on color value
   - Blooming animation if bloomed=true
   - Tooltip showing plant ID and status

### Pest Data Flow

1. **TypeDB Storage** → Stores pest detections with spray status
2. **typedb_node.pest_callback()** processes and stores to TypeDB
3. **typedb_node publishes** Pest message to `pest_data` topic with:
   - Pest ID
   - Location
   - Sprayed status
4. **dashboard_page.pest_callback_gui()** receives message and:
   - Creates bug entry: `{'id': f'B{msg.id}', 'x': msg.location.x, 'y': msg.location.y}`
   - Updates existing pest or adds new one to `bugs[]` list
5. **GUI Timer** renders pests on map with:
   - Bug image
   - Tooltip with pest ID
   - "Kill Bug" button to navigate to location

### Sensor Data Flow

1. **TypeDB Storage** → Stores sensor readings with type, value, unit
2. **typedb_node.sensor_callback()** processes and stores to TypeDB
3. **typedb_node publishes** Sensor message to `sensor_data` topic with:
   - Sensor ID
   - Location
   - Sensor type (e.g., "Humidity", "Temperature")
   - Array of SensorData readings
4. **dashboard_page.sensor_callback_gui()** receives message and:
   - Extracts numerical values from readings
   - Keeps only last 10 values for chart
   - Creates sensor entry: `{'id': f'S{msg.id}', 'x': ..., 'y': ..., 'type': ..., 'data': [...]}`
   - Updates existing sensor or adds new one to `sensors[]` list
5. **GUI Timer** renders sensors on map with:
   - Sensor image
   - Interactive chart showing historical data
   - "Take Reading" button to navigate to sensor

## Message Changes Made

### Sensor.msg Enhancement
**Before:**
```
uint64 id
geometry_msgs/Point location
SensorData[] readings
```

**After:**
```
uint64 id
geometry_msgs/Point location
string sensor_type
SensorData[] readings
```

**Reason:** The GUI needs to display the sensor type (Humidity, Temperature, etc.) separately from individual reading values. This field allows the typedb_node and GUI to identify the sensor type without ambiguity.

## How the GUI Subscribes

The `ROS2Interface` class in `dashboard_page.py` creates three subscriptions in `__init__()`:

```python
self.flower_sub = self.create_subscription(
    Flower,
    'flower_data',
    self.flower_callback_gui,
    10
)

self.pest_sub = self.create_subscription(
    Pest,
    'pest_data',
    self.pest_callback_gui,
    10
)

self.sensor_sub = self.create_subscription(
    Sensor,
    'sensor_data',
    self.sensor_callback_gui,
    10
)
```

These subscriptions:
- Listen to the appropriate ROS2 topics
- Call the GUI callback functions when messages arrive
- Update the global `plants[]`, `bugs[]`, `sensors[]` lists
- Are rendered on the next GUI timer tick (0.1s intervals)

## Data Persistence

- **Plants, Pests, Sensors** persist in TypeDB across node restarts
- **GUI display** updates in real-time when new messages arrive
- **Historical data** maintained in TypeDB; GUI shows latest readings for charts

## Future Enhancements

1. Add timestamps to messages for better historical tracking
2. Implement message queuing to handle high-frequency updates
3. Add filtering/aggregation in GUI for large numbers of entities
4. Implement persistence of GUI state (zoom, pan, selected entities)
5. Add confidence/accuracy metrics to sensor readings
