import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState


class JoyControl(Node):
    def __init__(self):
        super().__init__('joy_control')

        self.declare_parameter('cmd_vel_topic', '/mirte_base_controller/cmd_vel')
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.declare_parameter('joint_trajectory_topic', '/mirte_master_arm_controller/joint_trajectory')
        joint_trajectory_topic = self.get_parameter('joint_trajectory_topic').value

        # Publisher for base movement
        self.base_publisher = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10
        )

        # Publisher for arm movement
        self.arm_publisher = self.create_publisher(
            JointTrajectory,
            joint_trajectory_topic,
            10
        )

        # Subscriber for joystick input
        self.joy_subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )
        
        # Subscriber for arm state to synchronize joystick control with actual arm position
        self.state_subscription = self.create_subscription(
            JointTrajectoryControllerState,
            '/mirte_master_arm_controller/state',
            self.state_callback,
            10
        )

        # Callback timer to periodically update arm positions based on latest state
        self.latest_arm_positions = None
        self.arm_state_update_period = 1.0
        self.arm_state_timer = self.create_timer(
            self.arm_state_update_period,
            self.sync_arm_positions
        )

        # PlayStation controller mappings
        self.ps_button = 10 # PS button to switch between modes
        self.L1_button = 4 # L1 button to activate joystick control
        self.R1_button = 5 # R1 button to switch to holonomic mode
        self.L_hor = 0 # Left stick horizontal axis
        self.L_ver = 1 # Left stick vertical axis
        self.R_hor = 3 # Right stick horizontal axis
        self.R_ver = 4 # Right stick vertical axis

        # State variables
        self.mode = 'base'  # 'base' or 'arm'
        self.last_ps_state = 0

        # Base control parameters
        self.max_linear_speed = 0.4
        self.max_angular_speed = 1.0

        # Arm control parameters
        self.arm_joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_joint',
        ]
        self.arm_positions = [0.0, 0.0, 0.0, 0.0]
        self.arm_step = 0.002
        self.arm_min = -1.57
        self.arm_max = 1.57

        self.get_logger().info('JoyControl node started')

    def joy_callback(self, msg: Joy):
        
        # PS button edge detection to toggle between 'base' and 'arm' modes
        current_ps = msg.buttons[self.ps_button]
        if current_ps == 1 and self.last_ps_state == 0:
            # Rising edge: toggle mode
            self.mode = 'arm' if self.mode == 'base' else 'base'
            self.get_logger().info(f'Joystick control {self.mode}')
        self.last_ps_state = current_ps

        # Message objects for base and arm control
        twist = Twist()
        joint = JointTrajectory()

        # Only process joystick input if L1 button is held down
        if msg.buttons[self.L1_button] == 1:
            # Base control
            if self.mode == 'base':
                # Non-holonomic mode (default)
                twist.linear.x = self.max_linear_speed * msg.axes[self.L_ver] # left stick vertical
                twist.angular.z = self.max_angular_speed * msg.axes[self.R_hor] # right stick horizontal

                # Holonomic mode when R1 button is pressed (buttons[5])
                if msg.buttons[self.R1_button] == 1:  # R1 button for holonomic mode
                    twist.linear.y = self.max_linear_speed * msg.axes[self.L_hor]  # left stick horizontal
                    twist.linear.x = self.max_linear_speed * msg.axes[self.L_ver]  # left stick vertical
                
                self.base_publisher.publish(twist)

            # Arm control
            elif self.mode == 'arm':
                # Move the shoulder pan joint up/down with the left stick vertical axis.
                self.arm_positions[0] += msg.axes[self.L_hor] * self.arm_step
                self.arm_positions[0] = max(
                    self.arm_min,
                    min(self.arm_positions[0], self.arm_max)
                )

                # Move the shoulder lift joint up/down with the left stick vertical axis.
                self.arm_positions[1] -= msg.axes[self.L_ver] * self.arm_step
                self.arm_positions[1] = max(
                    self.arm_min,
                    min(self.arm_positions[1], self.arm_max)
                )

                # Move the elbow joint up/down with the right stick vertical axis.
                self.arm_positions[2] -= msg.axes[self.R_ver] * self.arm_step
                self.arm_positions[2] = max(
                    self.arm_min,
                    min(self.arm_positions[2], self.arm_max)
                )

                # Move the wrist joint up/down with the right stick horizontal axis.
                self.arm_positions[3] += msg.axes[self.R_hor] * self.arm_step
                self.arm_positions[3] = max(
                    self.arm_min,
                    min(self.arm_positions[3], self.arm_max)
                )

                # Publish the joint trajectory message
                joint.joint_names = self.arm_joint_names
                point = JointTrajectoryPoint()
                point.positions = self.arm_positions.copy()
                joint.points = [point]

                self.arm_publisher.publish(joint)

    # Callback to update latest arm positions from the controller state
    def state_callback(self, msg: JointTrajectoryControllerState):
        if msg.actual.positions:
            self.latest_arm_positions = list(msg.actual.positions)

    def sync_arm_positions(self):
        if self.latest_arm_positions is None:
            return

        self.arm_positions = self.latest_arm_positions.copy()



def main(args=None):
    rclpy.init(args=args)

    node = JoyControl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    stop_msg = Twist()
    node.base_publisher.publish(stop_msg)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()