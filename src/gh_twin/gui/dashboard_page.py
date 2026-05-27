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
import math



# Constants
MAP_WIDTH_METERS  = 10.0
MAP_HEIGHT_METERS = 5.0
TIMEOUT_STATUS_OFFLINE = 15

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

heartbeat = None

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

# Whether robot is connected or not
robot_status = 0

#########################################################################################

# ROS2 INTERFACE NODE

class ROS2Interface(Node):

    def __init__(self):
        global heartbeat

        super().__init__('ros2_interface')
        
        heartbeat = self.get_time_now()
        
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
        
        global robot_pos, robot_status, heartbeat

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
        heartbeat = self.get_time_now() # Update heartbeat to indicate activity
        
        robot_status = 1
        
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
        if current_operating_mode == "Manual Mode":
            self.goal_pos_pub.publish(goal_pose)
            ui.notify(f'Navigating to position at ({ros_x:.1f}, {ros_y:.1f})')
        
        
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
        return self.get_clock().now()


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
                ui.item('Measurement Mode', on_click=lambda: set_mode('Measurement Mode', "Starting measurement mode..."))
                ui.item('Manual Mode', on_click=lambda: set_mode('Manual Mode', "Starting manual mode..."))
                ui.item('Pause Mission', on_click=lambda: set_mode('Pause Mission', "Pausing mission..."))
                ui.item('Return Home', on_click=lambda: set_mode('Return Home', "Returning home..."))
                ui.item('Reset', on_click=lambda: set_mode('Reset', "Resetting system..."))
                ui.item('Shutdown', on_click=lambda: set_mode('Shutdown', "Shutting down..."))

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
                            ui.html(f'Color: {plant["color"]}')

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
            <div
                id="robot-marker"
                class="robot"
                style="
                    left:50%;
                    top:50%;
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

        #with ui.card().classes('alert-card').style('bottom' 'margin:12px;'):
            #ui.label('This dashboard is a digital twin of the greenhouse. Click anywhere on the map to send the robot there!').classes('text-center')

    def update_robot():
        global heartbeat, robot_status
        if ros2_interface.get_time_now() - heartbeat > rclpy.duration.Duration(seconds=TIMEOUT_STATUS_OFFLINE):
            robot_status = 0
            # If no heartbeat for TIMEOUT_STATUS_OFFLINE seconds, consider robot disconnected
    
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


