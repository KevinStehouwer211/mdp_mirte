#!/usr/bin/env python3
from asyncio import events
from datetime import datetime

from nicegui import ui, app, events

import threading
import yaml


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

from gh_twin_interfaces.msg import Flower, Pest, Sensor
#from lupin_greenhouse_msgs.msg import TagReading

import random
import math



# Constants
MAP_WIDTH_PX  = 266*2*2.7
MAP_HEIGHT_PX = 600
TIMEOUT_STATUS_OFFLINE = 5
MAP_RESOLUTION = 98*0.04/MAP_HEIGHT_PX  # 1px/m
map_origin_px = [920, MAP_HEIGHT_PX-30]
#map_origin_px = [0, 570]
plant_data_file = 'plants.yaml'
bug_data_file = 'bugs.yaml'


CAMERA_TOPICS = {
    'None': None,
    'Wrist Camera': '/wrist_camera/image_raw/compressed',
    'Front Camera': '/camera/color/image_raw/compressed'    
}

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
    'x': map_origin_px[0],   # normalized position in %
    'y': map_origin_px[1],
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

            
plants = yaml.safe_load(open(plant_data_file, 'r'))

bugs = yaml.safe_load(open(bug_data_file, 'r'))

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
        'temperature': [24,24.5,25],
        'co2': [400,410,420],
        'light': [300,320,350],
        'time': [0,1,2]
    }
]

bridge = CvBridge()

# Whether robot is connected or not
robot_status = 1
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
        global robot_status, warning_msgs, alert_msgs, timeout_counter
        robot_status = 1 if msg.data else 0
        if robot_status == 0:
            if prev_robot_status == 1:
                alert_msgs.append("Robot is offline")
                warning_msgs.append("Robot connection lost")
                prev_robot_status = 0
        else:
            if prev_robot_status == 0:
                alert_msgs.append("Robot is online")
                prev_robot_status = 1
                self.timeout_counter = self.get_time_now()
        
    
    # Function to convert ROS2 pose to normalized % coordinates and store in shared dict
    def pose_callback(self, msg):
        
        global robot_pos, robot_status

        ros_x = msg.pose.pose.position.x
        ros_y = msg.pose.pose.position.y
        ros_rot = msg.pose.pose.orientation

        # Write into the shared dict
        robot_pos['x'], robot_pos['y'] = self.real_to_pixel_transform(ros_x, ros_y)
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
        
        x, y = self.real_to_pixel_transform(msg.location.x, msg.location.y)
        
        # Create or update plant entry
        plant_entry = {
            'id': f'P{msg.id}',
            'x': x,
            'y': y,
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
        
        x, y = self.real_to_pixel_transform(msg.location.x, msg.location.y)
        
        # Create or update bug entry
        bug_entry = {
            'id': f'B{msg.id}',
            'x': x,
            'y': y,
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
        
        x, y = self.real_to_pixel_transform(msg.location.x, msg.location.y)
        
        # Create or update sensor entry
        sensor_entry = {
            'id': f'S{msg.id}',
            'x': x,
            'y': y,
            'type': msg.sensor_type if msg.sensor_type else 'Unknown',
            'data': values if values else [0],
        }
        
        # Update or add to sensors list
        existing = next((s for s in sensors if s['id'] == sensor_entry['id']), None)
        if existing:
            existing.update(sensor_entry)
        else:
            sensors.append(sensor_entry)
    '''
    def sensor_callback_gui_new(self, msg: TagReading):
        """Update GUI sensors list from sensor messages."""
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
        
    '''
    
    def real_to_pixel_transform(self,x,y):
        return [(y/MAP_RESOLUTION) + map_origin_px[0], map_origin_px[1] - (x/MAP_RESOLUTION)]
    
    def pixel_to_real_transform(self,x,y):
        return [(map_origin_px[1] - x)*MAP_RESOLUTION, (y - map_origin_px[0])*MAP_RESOLUTION]
    
    def image_callback(self, msg):
        # Convert ROS CompressedImage → OpenCV BGR → JPEG bytes → base64
        cv_img = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        _, buf  = cv2.imencode('.jpg', cv_img, [cv2.IMWRITE_JPEG_QUALITY, 80])
        camera['frame'] = base64.b64encode(buf).decode('utf-8')

    def go_to_pos(self, pos_x, pos_y):
        # Convert % position back to ROS2 map coordinates
        ros_x, ros_y = self.pixel_to_real_transform(pos_x, pos_y)
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
                    
            def save_data():
                with open(plant_data_file, 'w') as file:
                    yaml.safe_dump(plants, file, default_flow_style=False, sort_keys=False)
                
                with open(bug_data_file, 'w') as file:
                    yaml.safe_dump(bugs, file, default_flow_style=False, sort_keys=False)
                    
                add_log_entry(warnings_scroll, "Data saved successfully", "text-amber-600")

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

            def switch_camera(camera: str):
                
                ros2_interface.switch_topic(CAMERA_TOPICS[camera])
                current_topic.set_text(camera)
                add_log_entry(warnings_scroll, f'Switched to {camera})', "text-amber-600")
                #ui.notify(f'Switched to {topic}')
                
            camera_dropdown = ui.dropdown_button('Select Camera', auto_close=True).props('no-caps')

            with camera_dropdown:
                for key, value in CAMERA_TOPICS.items():
                    ui.item(
                        key,
                    on_click=lambda t=key: switch_camera(t)
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
            save_button = ui.button('SAVE DATA', on_click=lambda: save_data()).classes('w-full')
            
            ui.separator()
            
            def logout() -> None:
                app.storage.user.clear()
                    
                with open(plant_data_file, 'w') as file:
                    yaml.safe_dump(plants, file, default_flow_style=False, sort_keys=False)
                
                with open(bug_data_file, 'w') as file:
                    yaml.safe_dump(bugs, file, default_flow_style=False, sort_keys=False)
                    
                ui.notify('Logged out successfully', color='positive')
                ui.navigate.to('/login')

            with ui.column().classes('items-center'):
                ui.button(on_click=logout, icon='logout').props('outline round')
                
        # Get any clicked point
        greenhouse  = ui.image('map_hmi_updated.png').classes('map-panel').style('position: relative; display: inline-block; overflow: visible !important;')

            # for click testing
        greenhouse.on('click', lambda e: ros2_interface.go_to_pos(e.args["clientX"], e.args["clientY"]))

        with greenhouse:
            
            
            @ui.refreshable
            def draw_plants():
                
                # Plotting plants
                for plant in plants:
                    status_text = 'Bloomed' if plant['bloomed'] else 'Not Bloomed'
                    pid = plant['id']
                    filename = plant['color'].lower() + '_flower.png'
                    animation_class = 'blooming' if plant['bloomed'] else ''
                
                    with ui.element('div').style(
                        f'position:absolute;'
                        f'left:{plant["x"]}px;'
                        f'top:{plant["y"]}px;'
                        f'z-index:10;'
                        f'overflow:visible !important; '
                    ).classes('entity'):
                     
                        ui.image(filename).style(
                            'width:20px;'
                            'height:20px;'
                            'cursor:pointer;'
                            'background: transparent;'
                            'transition: transform 0.2s ease;'
                            'overflow:visible !important;'
                        ).classes(animation_class)
                    
                        with ui.element('div').classes('tooltip').props('pointer-events-auto'):
                        
                                ui.html(f'<b>Plant {pid}</b><br>Status: {status_text}')
                                ui.html(f'Color: {plant["color"]}')

                                ui.button(
                                    'Go To Position',
                                    on_click=lambda p=plant: ros2_interface.go_to_pos(p['x'], p['y'])
                                ).classes('tooltip-btn')

            @ui.refreshable
            def draw_sensors():
            
                for sensor in sensors:
                    sid = sensor['id']
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
                        with ui.element('div').classes('tooltip').props('pointer-events-auto'):
                            
                            with ui.card().style('width:300px; border-radius:20px;'):

                                chart = ui.echart({
                                    'backgroundColor': 'transparent',
                                    'animation': True,
                                    'tooltip': {'trigger': 'axis'},
                                    'xAxis': {'type': 'category','data': time_points,'boundaryGap': False,},
                                    'yAxis': {'type': 'value','name': 'units',},
                                    'series': [
                                        {'data': sensor["data"], 'type': 'line', 'smooth': True, 'areaStyle': {},}
                                    ],
                                }).style(
                                'height:300px'
                                )
                            sensor_charts.append({
                                'chart': chart,
                                'data': sensor["data"],
                            })
                        
                            ui.button(
                                'Take Reading',
                                on_click=lambda s=sensor: ros2_interface.go_to_pos(s['x'], s['y'])
                            ).classes('tooltip-btn').props('no-caps unelevated')

            @ui.refreshable
            def draw_bugs():
                for bug in bugs:
                    bid = bug['id']
                    with ui.element('div').style(
                        f'position:absolute;'
                        f'left:{bug["x"]}px;'
                        f'top:{bug["y"]}px;'
                        f'z-index:10;'
                        f'overflow:visible !important; '
                    ).classes('entity'):

                        ui.image('bug.png').style(
                            'width:20px;'
                            'height:20px;'
                            'cursor:pointer;'
                            'background: transparent;'
                            'transition: transform 0.2s ease;'
                            'overflow:visible !important;'
                        )

                        with ui.element('div').classes('tooltip').props('pointer-events-auto'):
                            ui.html(f'<b>Bug {bid}</b>')
                            ui.button(
                                'Kill Bug',
                                on_click=lambda b=bug: ros2_interface.go_to_pos(b['x'], b['y'])
                            ).classes('tooltip-btn').props('no-caps unelevated')

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
            
            draw_plants()
            
            draw_sensors()
            
            # Plotting bugs
            draw_bugs()

            
            # ------------------------------------------------
            # ROBOT MARKER
            # A single persistent <div> is injected once.
            # The timer updates only its style via JS — no
            # innerHTML replacement, so the element is never
            # destroyed and re-created on every tick.
            # ------------------------------------------------

            ui.html('''
            <div
                id="robot-marker"
                class="robot"
                style="
                    position: absolute !important;
                    left:0px;
                    top:0px;
                ">
                <div class="tooltip">
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
                ui.label('⚠️ Log').classes('text-lg font-bold text-gray-800 border-b pb-2 w-full')
        
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
            
            x, y = ros2_interface.real_to_pixel_transform(0,0)
            
            theta = robot_pos['theta']
            ui.run_javascript(f"""
                var el = document.getElementById('robot-marker');
                if (el) {{
                    el.style.left = '{x:.2f}px';
                    el.style.top  = '{y:.2f}px';
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
            
            # if ros2_interface.get_time_now() - ros2_interface.get_timeout_counter() > TIMEOUT_STATUS_OFFLINE:
            #     robot_status = 0
            #     alert_msgs.append("Robot connection timed out")
            #     warning_msgs.append("Robot connection lost")
            
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
    def update_env_data():

        if robot_status:
            
            draw_plants.refresh()
            
            draw_bugs.refresh()
            
            draw_sensors.refresh()

            for item in sensor_charts:

                data = item['data']
                chart = item['chart']

                data.append(round(data[-1] + random.uniform(-0.5, 0.5),2))

                data.pop(0)

                chart.options['series'][0]['data'] = data

                chart.update()
            

            

    ui.timer(0.1, update_robot)
    ui.timer(0.1, update_camera)
    ui.timer(10.0, update_env_data)


