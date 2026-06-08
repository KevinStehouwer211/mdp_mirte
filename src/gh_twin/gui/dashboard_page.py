#!/usr/bin/env python3
from asyncio import events
from datetime import datetime

from nicegui import ui, app, events

import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.clock import ClockType

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist
from sensor_msgs.msg import CompressedImage

from std_msgs.msg import Int32, Bool
from cv_bridge import CvBridge
import cv2
import base64

try:
    from gh_twin_interfaces.msg import Flower, Pest, Sensor
except ModuleNotFoundError:
    from gh_twin_msgs.msg import Flower, Pest, Sensor
from lupin_greenhouse_msgs.msg import TagReading

import random
import math



# Constants
MAP_WIDTH_METERS  = 10.0
MAP_HEIGHT_METERS = 5.0
TIMEOUT_STATUS_OFFLINE = 5

CAMERA_TOPICS = [
    'None',
    '/camera/color/image_raw/compressed',
    '/wrist_camera/image_raw/compressed',
]

robot_pose_topic = '/amcl_pose'

battery_topic = '/io/power_analyser/power'

robot_goal_pos_topic = '/goal_pose'

operating_mode_topic = '/operating_mode'

OPERATING_MODE = {
    -1: 'No Mode Selected',
    0: 'Measurement Mode',
    1: 'Manual Mode',
    2: 'Pause Mission',
    3: 'Return Home',
    4: 'Reset',
    5: 'Shutdown',
}

teleop_topic = '/mirte_base_controller/cmd_vel_unstamped'

######################################################################################

# Shared variables
# ROS2 Interface node object
ros2_interface = None

# Camera image obtained from hardware
camera = {
    'topic':   'None', # currently selected camera topic
    'frame':   None,   # latest JPEG bytes
}

# Current robot pose
robot_pos = {
    'x': 50.0,   # normalized position in %
    'y': 50.0,
    'theta': 0.0, # normalized orientation in %
}

# Goal robot position
goal_pos = {
    'x': 50.0,
    'y': 50.0,
}

# Battery percentage
battery_level = 0.0

current_operating_mode = "No Mode Selected"

heartbeat_topic = '/robot_heartbeat'

warning_msgs = []
alert_msgs = []

# ============================================================
# STATIC DATA
# ============================================================
time_points = [
    '10:00',
    '10:05',
    '10:10',

]

sensor_charts = []



# ------------------------------------------------
# PLANT BEDS
# ------------------------------------------------

beds = [
    (160, 95),  (460, 95),  (760, 95),  (1060, 95),
    (160, 475), (460, 475), (760, 475), (1060, 475),
]
            
plants = [
    {'id': 'P1', 'x': 170, 'y': 105, 'bloomed': True, 'color': 'White'},
    {'id': 'P2', 'x': 470, 'y': 485, 'bloomed': True, 'color': 'Pink'},
    {'id': 'P3', 'x': 770, 'y': 105, 'bloomed': False, 'color': 'Red'},
    {'id': 'P4', 'x': 1070, 'y': 105, 'bloomed': False, 'color': 'White'},
]

bugs = [
    {'id': 'B1', 'x': 180, 'y': 235},
    {'id': 'B2', 'x': 470, 'y': 150},
]

sensors = [
    {'id': 'S1', 'x': 1070, 'y': 180, 'type': 'Humidity', 'data': [22,22.4,22.8] },
    {'id': 'S2', 'x': 770, 'y': 120, 'type': 'Temperature', 'data': [24,24.5,25]},
]

sensors_new = [
    {
        'id': 'S1',
        'x': 1070, 'y': 180,
        'humidity': [22,22.4,22.8],
        'temperature': [24,24.5,25],
        'co2': [400,410,420],
        'light': [300,320,350],
        'time': [0,1,2]
    },
    {
        'id': 'S2',
        'x': 770, 'y': 120,
        'humidity': [22,22.4,22.8],
        # 'temperature': [24,24.5,25],
        'co2': [400,410,420],
        'light': [300,320,350],
        'time': [0,1,2]
    }
]

bridge = CvBridge()

# Whether robot is connected or not
robot_status = 0
prev_robot_status = 0

#########################################################################################

# ROS2 INTERFACE NODE

class ROS2Interface(Node):

    def __init__(self):

        super().__init__('ros2_interface')
        
        self.timeout_counter = self.get_time_now()
        
        self.heartbeat_sub = self.create_subscription(
            Bool,
            heartbeat_topic,
            self.heartbeat_callback,
            10
        )

        # Subscriber for robot pose 
        self.robot_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            robot_pose_topic,
            self.pose_callback,
            10
        )
        
        # Subscriber for battery status
        self.battery_sub = self.create_subscription(
            Int32,
            battery_topic,
            self.battery_callback,
            10
        )
        self.camera_sub = None
        self.switch_topic(camera['topic'])
        
        # Subscribers for entity data
        self.flower_sub = self.create_subscription(
            Flower,
            'gui_flower_data',
            self.flower_callback_gui,
            10
        )
        
        self.pest_sub = self.create_subscription(
            Pest,
            'gui_pest_data',
            self.pest_callback_gui,
            10
        )
        
        # self.sensor_sub = self.create_subscription(
        #     Sensor,
        #     'sensor_data',
        #     self.sensor_callback_gui,
        #     10
        # )

        self.sensor_sub = self.create_subscription(
            TagReading,
            '/greenhouse/telemetry',
            self.sensor_callback_gui_new,
            10
        )
        
        # Publisher for robot goal position
        self.goal_pos_pub = self.create_publisher(
            Pose,
            robot_goal_pos_topic,
            10
        )
        
        # Publisher for operating mode
        self.operating_mode_pub = self.create_publisher(
            Int32,
            operating_mode_topic,
            10
        )
        
        # Publisher for teleoperation commands
        self.teleop_pub = self.create_publisher(
            Twist,
            teleop_topic,
            10
        )

    def heartbeat_callback(self, msg):
        global robot_status, prev_robot_status, warning_msgs, alert_msgs
        robot_status = 1 if msg.data else 0
        if robot_status == 0:
            if prev_robot_status == 1:
                alert_msgs.append("Robot is offline")
                warning_msgs.append("Robot connection lost")
                prev_robot_status = 0
        else:
            self.timeout_counter = self.get_time_now()
            if prev_robot_status == 0:
                alert_msgs.append("Robot is online")
                prev_robot_status = 1
        
    
    # Function to convert ROS2 pose to normalized % coordinates and store in shared dict
    def pose_callback(self, msg):
        
        global robot_pos, robot_status

        ros_x = msg.pose.pose.position.x
        ros_y = msg.pose.pose.position.y
        ros_rot = msg.pose.pose.orientation

        normalized_x = (
            ros_x + MAP_WIDTH_METERS / 2
        ) / MAP_WIDTH_METERS

        normalized_y = (
            ros_y + MAP_HEIGHT_METERS / 2
        ) / MAP_HEIGHT_METERS

        # Write into the shared dict
        robot_pos['x'] = normalized_x * 100
        robot_pos['y'] = (1 - normalized_y) * 100
        robot_pos['theta'] = math.atan2(ros_rot.z, ros_rot.w) * 2 * (180 / math.pi)  # Convert to degrees
        
        
    def switch_topic(self, topic: str):
        if self.camera_sub is not None:
            self.destroy_subscription(self.camera_sub)
        self.camera_sub = self.create_subscription(
            CompressedImage,
            topic,
            self.image_callback,
            1,          # queue depth 1 — always use latest frame
        )
        camera['topic'] = topic
        
    def battery_callback(self, msg):
        global battery_level
        battery_level = msg.data
        
    def flower_callback_gui(self, msg: Flower):
        """Update GUI plants list from flower messages."""
        global plants
        
        # Convert color to the format used in the GUI (lowercase for image lookup)
        color = msg.color if msg.color else 'white'
        
        # Create or update plant entry
        plant_entry = {
            'id': f'P{msg.id}',
            'x': msg.location.x if msg.location else 0,
            'y': msg.location.y if msg.location else 0,
            'bloomed': msg.bloomed,
            'color': color,
        }
        
        # Update or add to plants list (check if id already exists)
        existing = next((p for p in plants if p['id'] == plant_entry['id']), None)
        if existing:
            existing.update(plant_entry)
        else:
            plants.append(plant_entry)
    
    def pest_callback_gui(self, msg: Pest):
        """Update GUI bugs list from pest messages."""
        global bugs
        
        # Create or update bug entry
        bug_entry = {
            'id': f'B{msg.id}',
            'x': msg.location.x,
            'y': msg.location.y,
        }
        
        # Update or add to bugs list
        existing = next((b for b in bugs if b['id'] == bug_entry['id']), None)
        if existing:
            existing.update(bug_entry)
        else:
            bugs.append(bug_entry)
    
    def sensor_callback_gui(self, msg: Sensor):
        """Update GUI sensors list from sensor messages."""
        global sensors
        
        # Extract numerical values from readings for chart data
        values = [float(r.value) for r in msg.readings if hasattr(r, 'value')]
        
        # Keep only the most recent readings for the chart (e.g., last 10)
        values = values[-10:] if len(values) > 10 else values
        
        # Create or update sensor entry
        sensor_entry = {
            'id': f'S{msg.id}',
            'x': msg.location.x,
            'y': msg.location.y,
            'type': msg.sensor_type if msg.sensor_type else 'Unknown',
            'data': values if values else [0],
        }
        
        # Update or add to sensors list
        existing = next((s for s in sensors if s['id'] == sensor_entry['id']), None)
        if existing:
            existing.update(sensor_entry)
        else:
            sensors.append(sensor_entry)

    def sensor_callback_gui_new(self, msg: TagReading):
        #Update GUI sensors list from sensor messages.
        global sensors_new
        
        # Extract ID (handling both 'id' from your old msg and 'tag_id' from the new service structure)
        raw_id = getattr(msg, 'id', getattr(msg, 'tag_id', 'Unknown'))
        sensor_id = f'S{raw_id}'
        
        # Convert ROS time stamp to a single float (seconds) for easier plotting
        timestamp = msg.stamp.sec + (msg.stamp.nanosec * 1e-9)
        
        # Find existing sensor entry
        existing = next((s for s in sensors_new if s['id'] == sensor_id), None)
        
        if not existing:
            # Initialize a new sensor entry
            existing = {
                'id': sensor_id,
                # Assuming location properties still exist; fallback to 0 if they don't
                'x': getattr(msg.location, 'x', 0) if hasattr(msg, 'location') else 0,
                'y': getattr(msg.location, 'y', 0) if hasattr(msg, 'location') else 0,
                'time': []
            }
            
            # Dynamically initialize an empty list for each sensor reading type (e.g., 'temperature')
            for r in msg.readings:
                existing[r.name] = []
                
            sensors_new.append(existing)

        # Append the new time
        existing['time'].append(timestamp)
        
        # Append the new values to their respective lists
        for r in msg.readings:
            if r.name in existing:
                existing[r.name].append(float(r.value))
                
        # Keep only the most recent readings for the chart (e.g., last 10)
        max_history = 10
        existing['time'] = existing['time'][-max_history:]
        
        for r in msg.readings:
            if r.name in existing:
                existing[r.name] = existing[r.name][-max_history:]
        
    def image_callback(self, msg):
        # Convert ROS CompressedImage → OpenCV BGR → JPEG bytes → base64
        cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, buf  = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        camera['frame'] = base64.b64encode(buf).decode('utf-8')

    def go_to_pos(self, pos_x, pos_y):
        # Convert % position back to ROS2 map coordinates
        ros_x = (pos_x / 100) * MAP_WIDTH_METERS  - MAP_WIDTH_METERS  / 2
        ros_y = (pos_y / 100) * MAP_HEIGHT_METERS - MAP_HEIGHT_METERS / 2
        goal_pose = Pose()
        goal_pose.position.x = ros_x
        goal_pose.position.y = ros_y
        if current_operating_mode == "Manual Mode":
            self.goal_pos_pub.publish(goal_pose)
            ui.notify(f'Navigating to position at ({ros_x:.1f}, {ros_y:.1f})')
            alert_msgs.append(f"Navigating to ({ros_x:.1f}, {ros_y:.1f})")
        
        
    def set_operating_mode(self, mode: int):
        global current_operating_mode
        current_operating_mode = OPERATING_MODE.get(mode, "Unknown")
        self.operating_mode_pub.publish(Int32(data=mode))
        
    def send_teleop_command(self, command: str):
        cmd = Twist()
        if command == 'f':
            cmd.linear.x = 0.5
        elif command == 'b':
            cmd.linear.x = -0.5
        elif command == 'r':
            cmd.angular.z = -0.5
        elif command == 'l':
            cmd.angular.z = 0.5
        self.teleop_pub.publish(cmd)
        
    def get_time_now(self):
        return self.get_clock().now().to_msg().sec
    
    def get_timeout_counter(self):
        return self.timeout_counter


# ============================================================
# ROS2 SPIN THREAD
# ============================================================

def ros_spin():

    global ros2_interface
    
    if not rclpy.ok():
        rclpy.init()

    ros2_interface = ROS2Interface()
    executor = SingleThreadedExecutor()
    executor.add_node(ros2_interface)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
    finally:
        executor.remove_node(ros2_interface)
        ros2_interface.destroy_node()


def add_log_entry(log_container, text, text_color_class):
    """Appends a new log entry with a real-time timestamp and auto-scrolls."""
    timestamp = datetime.now().strftime("[%H:%M:%S]")
    with log_container:
        with ui.row().classes('items-center gap-2 mb-1'):
            ui.label(timestamp).classes('text-gray-500 font-mono text-sm font-bold')
            ui.label(text).classes(f'{text_color_class} font-mono text-sm')
            
# Start thread exactly once
_ros_thread_started = False

if not _ros_thread_started:
    _ros_thread_started = True
    threading.Thread(
        target=ros_spin,
        daemon=True,
        name='ros2_spin',
    ).start()

########################################################################################

@ui.page('/')
def main_page():
    
    # --------------------------------------------------------
    # LAYOUT
    # --------------------------------------------------------
    ui.query('body').classes(add='light-theme')

    with ui.element('div').classes('main-container'):
        
        with ui.card().classes('side-card light-card items-stretch'):
            ui.label('Robot Status').classes('title text-left')
            robot_status_display = ui.label('Status: Offline').style('font-size:15px;')
            ui.separator()
            ui.label('Operating Mode').classes('text-lg')
            mode_display = ui.label('No Mode Selected').style('font-size:15px; color:grey;')
            teleop_display = ui.label('').classes('whitespace-pre-line').style('font-size:15px')
  
            def set_mode(label, message):
                mode_display.set_text(label)
                ros2_interface.set_operating_mode(list(OPERATING_MODE.keys())[list(OPERATING_MODE.values()).index(label)])
                if current_operating_mode == "Manual Mode":
                    teleop_display.set_text('F: Move Forward\nB: Move Backward\nR: Rotate Right\nL: Rotate Left')
                else:
                    teleop_display.set_text('')
                ui.notify(message)
                alert_msgs.append(message)
                
            # Handling keyboard teleop in manual mode    
            def handle_key(e: events.KeyEventArguments):
                command = None
                if current_operating_mode == "Manual Mode":
                    if e.key=='f' and e.action.keyup:
                        command = 'f'
                        ui.notify('Going forward')
                    elif e.key=='b' and e.action.keyup:
                        command = 'b'
                        ui.notify('Going backward')
                    elif e.key=='r' and e.action.keyup:
                        command = 'r'
                        ui.notify('Rotating right')
                    elif e.key=='l' and e.action.keyup:
                        command = 'l'
                        ui.notify('Rotating left')
                    ros2_interface.send_teleop_command(command)

                    
            keyboard = ui.keyboard(on_key=handle_key)
            
            operating_mode_dropdown = ui.dropdown_button('Select Operating Mode', auto_close=True)
                
            with operating_mode_dropdown:
                ui.item('Measurement Mode', on_click=lambda: set_mode('Measurement Mode', "Started measurement mode"))
                ui.item('Manual Mode', on_click=lambda: set_mode('Manual Mode', "Started manual mode"))
                ui.item('Pause Mission', on_click=lambda: set_mode('Pause Mission', "Pausing mission"))
                ui.item('Return Home', on_click=lambda: set_mode('Return Home', "Returning home"))
                ui.item('Reset', on_click=lambda: set_mode('Reset', "Resetting system"))
                ui.item('Shutdown', on_click=lambda: set_mode('Shutdown', "Shutting down"))

            ui.separator()

            ui.label('Battery Level').classes('text-lg')
            battery_progress_bar = ui.linear_progress(value=0.0, show_value=False).props('size=12px rounded instant-feedback')
            battery_label = ui.label('N/A').style('font-size:12px; color:grey;')

            ui.separator()

            # ── Camera topic selector ────────────────────────────────────
            ui.label('Camera Feed').classes('text-lg')

            current_topic = ui.label(camera['topic']).style(
                'font-size:11px; color:grey; margin-bottom:6px;'
            )

            def switch_camera(topic: str):
                
                ros2_interface.switch_topic(topic)
                current_topic.set_text(topic)
                ui.notify(f'Switched to {topic}')
                
            camera_dropdown = ui.dropdown_button('Select Camera', auto_close=True).props('no-caps')

            with camera_dropdown:
                for topic in CAMERA_TOPICS:
                    ui.item(
                        topic,
                    on_click=lambda t=topic: switch_camera(t)
                    )

            # ── Camera image display ─────────────────────────────────────
            camera_img = ui.html('''
            <img
                id="camera-feed"
                style="
                width:100%;
                border-radius:8px;
                object-fit:cover;
                "
            />
            ''')

            def update_camera():

                if camera['frame'] is None:
                    return

                ui.run_javascript(f'''
                    const img = document.getElementById("camera-feed");

                    if (img) {{
                        img.src = "data:image/jpeg;base64,{camera['frame']}";
                    }}
                ''')
            
            ui.separator()
            save_button = ui.button('SAVE DATA', on_click=lambda: ui.notify("Data saved successfully!")).classes('w-full')
            
            ui.separator()
            
            def logout() -> None:
                app.storage.user.clear()
                ui.notify('Logged out successfully', color='positive')
                ui.navigate.to('/login')

            with ui.column().classes('items-center'):
                ui.button(on_click=logout, icon='logout').props('outline round')
                
        # Get any clicked point
        greenhouse  = ui.image('map_hmi_updated.png').classes('map-panel')
            # for click testing
        greenhouse.on('click', lambda e: ros2_interface.go_to_pos(e.args["clientX"], e.args["clientY"]))

        with greenhouse:

            ui.element('div').classes('greenhouse-border')

            #ui.html('<div class="home-station">Home</div>')


            # for bx, by in beds:
            #     ui.html(
            #         f'<div class="plant-bed" '
            #         f'style="left:{bx}px; top:{by}px;"></div>'
            #     )
            ui.add_css('''
                @layer utilities {
                    .entity{
                        background: transparent !important;
                        background-color: transparent !important;
                        box-shadow: none !important;
                        border: none !important;
                    }
                }
            ''')
            
            # Plotting plants
            for plant in plants:
                status_text = 'Bloomed' if plant['bloomed'] else 'Not Bloomed'
                pid = plant['id']
                filename = plant['color'].lower() + '_flower.png'
                animation_class = 'blooming' if plant['bloomed'] else ''
                
                # 1. Grab a reference to the container
                with ui.element('div').style(
                    f'position:absolute;'
                    f'left:{plant["x"]}px;'
                    f'top:{plant["y"]}px;'
                    f'z-index:10;'
                    f'overflow:visible !important; '
                ).classes('entity') as entity_container:
                     
                    ui.image(filename).style(
                        'width:20px;'
                        'height:20px;'
                        'cursor:pointer;'
                        'background: transparent;'
                        'transition: transform 0.2s ease;'
                        'overflow:visible !important;'
                    ).classes(animation_class)
                    
                    # 2. Use tooltip-js and start hidden (display: none)
                    tooltip_card = ui.element('div').classes('tooltip-js').style('display: none;').props('pointer-events-auto')
                    
                    with tooltip_card:
                        ui.html(f'<b>Plant {pid}</b><br>Status: {status_text}')
                        ui.html(f'Color: {plant["color"]}')

                        ui.button(
                            'Go To Position',
                            on_click=lambda p=plant: ros2_interface.go_to_pos(p['x'], p['y'])
                        ).classes('tooltip-btn')

                # 3. Wire up Python hover events (using default args to prevent loop closure bugs)
                entity_container.on('mouseenter', lambda e, tc=tooltip_card: tc.style('display: block'))
                entity_container.on('mouseleave', lambda e, tc=tooltip_card: tc.style('display: none'))


            # ==========================================
            # Plotting bugs
            # ==========================================
            for bug in bugs:
                bid = bug['id']
                
                # 1. Grab a reference to the container
                with ui.element('div').style(
                    f'position:absolute;'
                    f'left:{bug["x"]}px;'
                    f'top:{bug["y"]}px;'
                    f'z-index:10;'
                    f'overflow:visible !important; '
                ).classes('entity') as entity_container:

                    ui.image('bug.png').style(
                        'width:20px;'
                        'height:20px;'
                        'cursor:pointer;'
                        'background: transparent;'
                        'transition: transform 0.2s ease;'
                        'overflow:visible !important;'
                    )

                    # 2. Use tooltip-js and start hidden (display: none)
                    tooltip_card = ui.element('div').classes('tooltip-js').style('display: none;').props('pointer-events-auto')
                    
                    with tooltip_card:
                        ui.html(f'<b>Bug {bid}</b>')
                        ui.button(
                            'Kill Bug',
                            on_click=lambda b=bug: ros2_interface.go_to_pos(b['x'], b['y'])
                        ).classes('tooltip-btn').props('no-caps unelevated')

                # 3. Wire up Python hover events
                entity_container.on('mouseenter', lambda e, tc=tooltip_card: tc.style('display: block'))
                entity_container.on('mouseleave', lambda e, tc=tooltip_card: tc.style('display: none'))

            # Plotting sensors
            for sensor in sensors_new:
                sid = sensor['id']
                initial_metric = 'temperature'
                
                # --- STEP 1: Define a state dictionary to track this specific tooltip ---
                state = {'is_dropdown_open': False}

                with ui.element('div').style(
                    f'position:absolute;'
                    f'left:{sensor["x"]}px;'
                    f'top:{sensor["y"]}px;'
                    f'z-index:10;'
                    f'overflow:visible !important;'
                ).classes('entity'):

                    ui.image('sensor.png').style(
                        'width:20px;'
                        'height:20px;'
                        'cursor:pointer;'
                        'background: transparent;'
                        'transition: transform 0.2s ease;'
                        'overflow:visible !important;'
                    )

                    # --- STEP 2: Bind visibility to NiceGUI's conditional styling instead of pure CSS ---
                    tooltip_card = ui.element('div').classes('tooltip-js').style('display: none;').props('pointer-events-auto')
                    
                    with tooltip_card:
                        with ui.card().style('width:320px; border-radius:20px; padding: 15px;'):
                            
                            with ui.row().classes('w-full justify-between items-center no-wrap q-mb-sm'):
                                ui.label(f'Sensor {sid}').classes('text-bold text-lg')
                                
                                metric_selector = ui.select(
                                    options={'temperature': 'Temp', 'humidity': 'Humid', 'co2': 'CO2', 'light': 'Light'},
                                    value=initial_metric
                                ).props('dense options-dense outlined').style('width: 110px;')

                                # --- STEP 3: Listen to Quasar's popup open/close events (FIXED WITH DEFAULT ARGS) ---
                                metric_selector.on('popup-show', lambda st=state: st.update({'is_dropdown_open': True}))
                                metric_selector.on('popup-hide', lambda e, st=state, tc=tooltip_card: [
                                    st.update({'is_dropdown_open': False}),
                                    tc.style('display: none') 
                                ])

                            # The Chart
                            chart = ui.echart({
                                'backgroundColor': 'transparent',
                                'animation': True,
                                'tooltip': {'trigger': 'axis'},
                                'xAxis': {'type': 'category', 'data': sensor.get('time', []), 'boundaryGap': False, 'name': 'Time'},
                                'yAxis': {'type': 'value', 'name': initial_metric.capitalize()},
                                'series': [{'data': sensor.get(initial_metric, []), 'type': 'line', 'smooth': True, 'areaStyle': {}}],
                            }).style('height:220px; width: 100%;')
                            
                            # Chart update hook
                            def make_update_handler(s=sensor, c=chart):
                                return lambda e: [
                                    c.options['series'][0].update({'data': s.get(e.value, [])}),
                                    c.options['yAxis'].update({'name': e.value.capitalize()}),
                                    c.update()
                                ]
                            metric_selector.on_value_change(make_update_handler())

                        ui.button(
                            'Take Reading',
                            on_click=lambda s=sensor: ros2_interface.go_to_pos(s['x'], s['y'])
                        ).classes('tooltip-btn').props('no-caps unelevated')

            
            # ------------------------------------------------
            # ROBOT MARKER
            # A single persistent <div> is injected once.
            # The timer updates only its style via JS — no
            # innerHTML replacement, so the element is never
            # destroyed and re-created on every tick.
            # ------------------------------------------------

            # ui.html('''
            # <div
            #     id="robot-marker"
            #     class="robot"
            #     style="
            #         left:50%;
            #         top:50%;
            #     ">
            #     <div class="tooltip">
            #         <b>Robot</b>
            #         <br>
            #     <span id="robot-pos">
            #         Position: (50%, 50%)
            #     </span>
            #     </div>
            # </div>
            # ''')
            ui.html('''
            <div
                id="robot-marker"
                class="robot"
                style="
                    left:50%;
                    top:50%;
                    cursor: pointer;
                "
                onmouseenter="document.getElementById('robot-tooltip').style.display='block'"
                onmouseleave="document.getElementById('robot-tooltip').style.display='none'"
            >
                <div id="robot-tooltip" class="tooltip-js" style="display: none; pointer-events: auto;">
                    <b>Robot</b>
                    <br>
                <span id="robot-pos">
                    Position: (50%, 50%)
                </span>
                </div>
            </div>
            ''')

        # Assuming your main map element is right above this container...
        with ui.row().classes('w-full no-wrap gap-4 mt-4'):
    
            # 1. ALERTS BOX (Critical Issues)
            with ui.card().classes('flex-1 alert-panel'):
                ui.label('🚨 Alerts').classes('text-lg font-bold text-gray-800 border-b pb-2 w-full')
        
                # Scroll area defining a fixed height. Content inside will scroll dynamically.
                alerts_scroll = ui.scroll_area().classes('h-[150px] w-full mt-2')


            # 2. WARNINGS BOX (System Notices)
            with ui.card().classes('flex-1 warning-panel'):
                ui.label('⚠️ Warnings').classes('text-lg font-bold text-gray-800 border-b pb-2 w-full')
        
                # Scroll area matching the height of the alerts box
                warnings_scroll = ui.scroll_area().classes('h-[150px] w-full mt-2')
 

    def update_robot():
        global robot_status, warning_msgs, alert_msgs
    
        if robot_status == 1:
            robot_status_display.set_text('Status: Online')
            robot_status_display.classes(replace='text-green font-bold')
            operating_mode_dropdown.enable()
            x = robot_pos['x']
            y = robot_pos['y']
            theta = robot_pos['theta'] + 90
            ui.run_javascript(f"""
                var el = document.getElementById('robot-marker');
                if (el) {{
                    el.style.left = '{x:.2f}%';
                    el.style.top  = '{y:.2f}%';
                    el.style.background = '#2563eb';
                    el.style.transform = 'translate(-50%, -50%) rotate({theta:.1f}deg)';
                }}
                var pos = document.getElementById('robot-pos');
                if (pos) {{
                    pos.textContent =
                        'Position: ({x:.1f}%, {y:.1f}%)';
                }}
            """)
            camera_dropdown.enable()
            save_button.enable()
            
            battery_level = 0.5  # Placeholder until we get real data from ROS2
            
            if battery_level < 0.2:
                battery_progress_bar.props('color=red')
            else:
                battery_progress_bar.props('color=green')
            
            battery_progress_bar.set_value(battery_level)
            battery_label.set_text(f'{battery_level*100:.0f}%')
            if ros2_interface.get_time_now() - ros2_interface.get_timeout_counter() > TIMEOUT_STATUS_OFFLINE:
                robot_status = 0
                alert_msgs.append("Robot connection timed out")
                warning_msgs.append("Robot connection lost")
            
        else:
            robot_status_display.set_text('Status: Offline')
            robot_status_display.classes(replace='text-red font-bold')
            operating_mode_dropdown.disable()
            ui.run_javascript("""
                var el = document.getElementById('robot-marker');
                if (el) {
                    el.style.background = '#888888';
                    el.style.boxShadow = '0 0 20px rgba(136,136,136,0.45)';
                }
            """)
            camera_dropdown.disable()
            save_button.disable()
            battery_level = 0.0
            battery_progress_bar.props('color=grey')
            battery_progress_bar.set_value(0.0)
            battery_label.set_text('N/A')
            
            
        
        alert_msgs_copy = alert_msgs.copy()
        alert_msgs.clear()
        for _ in alert_msgs_copy:
            msg = alert_msgs_copy.pop(0)
            add_log_entry(alerts_scroll, msg, "text-red-600")
            print("ALERT:", msg)
        
        
        for msg in warning_msgs:
            add_log_entry(warnings_scroll, msg, "text-amber-600")
        warning_msgs.clear()    
            
    # Update sensor charts with new random data for demonstration
    def update_chart():

        for item in sensor_charts:

            data = item['data']
            chart = item['chart']

            data.append(round(data[-1] + random.uniform(-0.5, 0.5),2))

            data.pop(0)

            chart.options['series'][0]['data'] = data

            chart.update()
            

            

    ui.timer(0.1, update_robot)
    ui.timer(0.1, update_camera)
    ui.timer(0.5, update_chart)
