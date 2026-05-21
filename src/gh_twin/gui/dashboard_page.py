#!/usr/bin/env python3
from asyncio import events

from nicegui import ui, app, events

import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Twist
from sensor_msgs.msg import CompressedImage

from std_msgs.msg import Int32
from cv_bridge import CvBridge
import cv2
import base64

import random



# Constants
MAP_WIDTH_METERS  = 10.0
MAP_HEIGHT_METERS = 5.0

CAMERA_TOPICS = [
    'None',
    '/camera/color/image_raw/compressed',
    '/wrist_camera/image_raw/compressed',
]

robot_pose_topic = '/amcl_pose'

battery_topic = '/battery_status'

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

teleop_topic = '/teleop_cmd'

######################################################################################

# Shared variables
# ROS2 Interface node object
ros2_interface = None

# Camera image obtained from hardware
camera = {
    'topic':   'None', # currently selected camera topic
    'frame':   None,   # latest JPEG bytes
}

# Current robot position
robot_pos = {
    'x': 50.0,   # normalized position in %
    'y': 50.0,
}

# Goal robot position
goal_pos = {
    'x': 50.0,
    'y': 50.0,
}

# Battery percentage
battery_level = 100

current_operating_mode = "No Mode Selected"

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

bridge = CvBridge()
#########################################################################################

# ROS2 INTERFACE NODE

class ROS2Interface(Node):

    def __init__(self):

        super().__init__('ros2_interface')
        
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

    # Function to convert ROS2 pose to normalized % coordinates and store in shared dict
    def pose_callback(self, msg):

        ros_x = msg.pose.pose.position.x
        ros_y = msg.pose.pose.position.y

        normalized_x = (
            ros_x + MAP_WIDTH_METERS / 2
        ) / MAP_WIDTH_METERS

        normalized_y = (
            ros_y + MAP_HEIGHT_METERS / 2
        ) / MAP_HEIGHT_METERS

        # Write into the shared dict
        robot_pos['x'] = normalized_x * 100
        robot_pos['y'] = (1 - normalized_y) * 100
        
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
        self.goal_pos_pub.publish(goal_pose)
        ui.notify(f'Navigating to position at ({ros_x:.1f}, {ros_y:.1f})')
        
    def set_operating_mode(self, mode: int):
        global current_operating_mode
        current_operating_mode = OPERATING_MODE.get(mode, "Unknown")
        self.operating_mode_pub.publish(Int32(data=mode))
        ui.notify(f'Set operating mode to {OPERATING_MODE.get(mode, "Unknown")}')
        
    def send_teleop_command(self, linear_x=0.0, angular_z=0.0):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.teleop_pub.publish(cmd)    


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
            ui.separator()
            ui.label('Operating Mode').classes('text-lg')
            mode_display = ui.label('No Mode Selected').style('font-size:12px; color:grey;')
            
            def set_mode(label, message):
                mode_display.set_text(label)
                ros2_interface.set_operating_mode(list(OPERATING_MODE.keys())[list(OPERATING_MODE.values()).index(label)])
            
            
            # Handling keyboard teleop in manual mode    
            def handle_key(e: events.KeyEventArguments):
                if current_operating_mode == "Manual Mode":
                    if e.key.arrow_left:
                        ui.notify('Going left')
                    elif e.key.arrow_right:
                        ui.notify('Going right')
                    elif e.key.arrow_up:
                        ui.notify('Going up')
                    elif e.key.arrow_down:
                        ui.notify('Going down')
                    
            keyboard = ui.keyboard(on_key=handle_key)
                
            with ui.dropdown_button('Select Operating Mode', auto_close=True):
                ui.item('Measurement Mode', on_click=lambda: set_mode('Measurement Mode', "Starting measurement mode..."))
                ui.item('Manual Mode', on_click=lambda: set_mode('Manual Mode', "Starting manual mode..."))
                ui.item('Pause Mission', on_click=lambda: set_mode('Pause Mission', "Pausing mission..."))
                ui.item('Return Home', on_click=lambda: set_mode('Return Home', "Returning home..."))
                ui.item('Reset', on_click=lambda: set_mode('Reset', "Resetting system..."))
                ui.item('Shutdown', on_click=lambda: set_mode('Shutdown', "Shutting down..."))

            ui.separator()

            ui.label('Battery Level').classes('text-lg')
            ui.label(f'{battery_level}%').style('font-size:12px; color:grey;')

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

            with ui.dropdown_button('Select Camera', auto_close=True).props('no-caps'):
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
            ui.button('SAVE DATA', on_click=lambda: ui.notify("Data saved successfully!")).classes('w-full')
            
            ui.separator()
            
            def logout() -> None:
                app.storage.user.clear()
                ui.notify('Logged out successfully', color='positive')
                ui.navigate.to('/login')

            with ui.column().classes('items-center'):
                ui.button(on_click=logout, icon='logout').props('outline round')
                
        # Get any clicked point
        greenhouse  = ui.element('div').classes('map-panel')  # for click testing
        greenhouse.on('click', lambda e: ros2_interface.go_to_pos(e.args["clientX"], e.args["clientY"]))

        with greenhouse:

            ui.element('div').classes('greenhouse-border')

            ui.html('<div class="home-station">Home</div>')


            for bx, by in beds:
                ui.html(
                    f'<div class="plant-bed" '
                    f'style="left:{bx}px; top:{by}px;"></div>'
                )

            # Plotting plants

            for plant in plants:
                dot_color = '#39ff14' if plant['bloomed'] else "#fbff0c"
                status_text = 'Bloomed' if plant['bloomed'] else 'Not Bloomed'
                pid = plant['id']
                filename = plant['color'].lower() + '_flower.png'
                animation_class = 'blooming' if plant['bloomed'] else ''
                
                with ui.element('div').style(
                    f'position:absolute;'
                    f'left:{plant["x"]}px;'
                    f'top:{plant["y"]}px;'
                    f'z-index:10;'
                    f'overflow:visible;'
                ).classes('entity'):
                    
                    ui.image(filename).style(
                        'width:35px;'
                        'height:35px;'
                        'cursor:pointer;'
                        'background: transparent;'
                        'transition: transform 0.2s ease;'
                    ).classes(animation_class)
                    
                    with ui.element('div').classes('tooltip').props('pointer-events-auto'):
                        
                            ui.html(f'<b>Plant {pid}</b><br>Status: {status_text}')

                            ui.button(
                                'Go To Position',
                                on_click=lambda p=plant: ros2_interface.go_to_pos(p['x'], p['y'])
                            ).classes('tooltip-btn')


            # Plotting bugs
            
            for bug in bugs:
                bid = bug['id']
                with ui.element('div').style(
                    f'position:absolute;'
                    f'left:{bug["x"]}px;'
                    f'top:{bug["y"]}px;'
                    f'z-index:10;'
                    f'overflow:visible;'
                ).classes('entity'):

                    ui.image('bug.png').style(
                        'width:28px;'
                        'height:28px;'
                        'cursor:pointer;'
                        'background: transparent;'
                        'transition: transform 0.2s ease;'
                    )

                    with ui.element('div').classes('tooltip').props('pointer-events-auto'):
                        ui.html(f'<b>Bug {bid}</b>')
                        ui.button(
                            'Kill Bug',
                            on_click=lambda b=bug: ros2_interface.go_to_pos(b['x'], b['y'])
                        ).classes('tooltip-btn').props('no-caps unelevated')

            # Plotting sensors
            
            for sensor in sensors:
                sid = sensor['id']
                with ui.element('div').style(
                    f'position:absolute;'
                    f'left:{sensor["x"]}px;'
                    f'top:{sensor["y"]}px;'
                    f'z-index:10;'
                    f'overflow:visible;'
                ).classes('entity'):

                    ui.image('sensor.png').style(
                        'width:28px;'
                        'height:28px;'
                        'cursor:pointer;'
                        'background: transparent;'
                        'transition: transform 0.2s ease;'
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
                            '   height:300px'
                            )
                        sensor_charts.append({
                            'chart': chart,
                            'data': sensor["data"],
                        })
                        
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

            ui.html('''
            <div id="robot-marker" class="robot"
                 style="left:50%; top:50%;">
                <div class="tooltip" id="robot-tooltip">
                    <b>Robot</b><br>
                    <span id="robot-pos">Position: (50.0%, 50.0%)</span>
                    <button class="tooltip-btn">Pause Mission</button>
                    <button class="tooltip-btn">Return Home</button>
                </div>
            </div>
            ''')


    def update_robot():
        x = robot_pos['x']
        y = robot_pos['y']
        ui.run_javascript(f"""
            var el = document.getElementById('robot-marker');
            if (el) {{
                el.style.left = '{x:.2f}%';
                el.style.top  = '{y:.2f}%';
            }}
            var pos = document.getElementById('robot-pos');
            if (pos) {{
                pos.textContent =
                    'Position: ({x:.1f}%, {y:.1f}%)';
            }}
        """)

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

