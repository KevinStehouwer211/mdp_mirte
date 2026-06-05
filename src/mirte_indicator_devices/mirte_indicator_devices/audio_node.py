import rclpy
import subprocess

from pathlib import Path
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from mirte_msgs.msg import SetSpeed
from std_msgs.msg import String

class RobotAudioNode(Node):
    def __init__(self):
        super().__init__('mirte_audio_node')

        # --- Configurations ---
        # Replace these with the absolute paths to your WAV files
        self.error_sound = Path(get_package_share_directory('mirte_indicator_devices')) / 'sounds' / 'error_sound.wav'
        self.move_sound = Path(get_package_share_directory('mirte_indicator_devices')) / 'sounds' / 'move_sound.wav'

        self.speed_threshold = 0.5  # Adjust this to your desired speed threshold
        self.hardware_device = 'hw:1'

        # --- State Tracking ---
        self.is_in_error = False
        
        # Track each motor's state (True if over threshold, False if under)
        self.motor_states = {
            'front_left': False,
            'front_right': False,
            'rear_left': False,
            'rear_right': False
        }
        self.robot_is_moving = False

        # --- Subscriptions ---
        self.state_sub = self.create_subscription(
            String,  # Change if robot_state is an Int32, etc.
            'robot_state',
            self.state_callback,
            10
        )

        # Helper function to create motor callbacks easily
        def create_motor_callback(motor_name):
            return lambda msg: self.motor_speed_callback(msg, motor_name)

        self.fl_sub = self.create_subscription(SetSpeed, '/io/motor/front_left/speed', create_motor_callback('front_left'), 10)
        self.fr_sub = self.create_subscription(SetSpeed, '/io/motor/front_right/speed', create_motor_callback('front_right'), 10)
        self.rl_sub = self.create_subscription(SetSpeed, '/io/motor/rear_left/speed', create_motor_callback('rear_left'), 10)
        self.rr_sub = self.create_subscription(SetSpeed, '/io/motor/rear_right/speed', create_motor_callback('rear_right'), 10)

        # --- Timers ---
        # Timer to check and play the error sound every 5 seconds
        self.error_timer = self.create_timer(5.0, self.error_timer_callback)

        self.get_logger().info("Robot Audio Node has been started.")

    def play_sound(self, filepath):
        """Non-blocking function to play an audio file."""
        try:
            # Using Popen so the ROS 2 executor isn't blocked while the sound plays
            subprocess.Popen(['aplay', '-D', self.hardware_device, filepath], 
                             stdout=subprocess.DEVNULL, 
                             stderr=subprocess.DEVNULL)
            self.get_logger().debug(f"Playing {filepath}")
        except Exception as e:
            self.get_logger().error(f"Failed to play sound: {e}")

    def state_callback(self, msg: String):
        """Updates the error state based on the robot_state topic."""
        # Adjust the string matching depending on what your state topic actually outputs
        if msg.data.lower() == 'error':
            self.is_in_error = True
        else:
            self.is_in_error = False

    def error_timer_callback(self):
        """Fires every 5 seconds. Plays sound if robot is in error state."""
        if self.is_in_error:
            self.get_logger().info("Robot is in error state. Playing error sound.")
            self.play_sound(self.error_sound)

    def motor_speed_callback(self, msg: SetSpeed, motor_name: str):
        """Handles rising edge detection for overall robot movement."""
        # 1. Update the individual motor's state
        is_motor_moving = abs(msg.speed) > self.speed_threshold
        self.motor_states[motor_name] = is_motor_moving

        # 2. Check global state: is ANY motor moving?
        any_motor_moving = any(self.motor_states.values())

        # 3. Detect the rising edge of the overall robot movement
        if any_motor_moving and not self.robot_is_moving:
            self.get_logger().info("Robot started moving! Playing movement sound.")
            self.play_sound(self.move_sound)
            self.robot_is_moving = True
            
        # 4. Detect the falling edge (robot completely stopped) to reset the trigger
        elif not any_motor_moving and self.robot_is_moving:
            self.robot_is_moving = False


def main(args=None):
    rclpy.init(args=args)
    node = RobotAudioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down audio node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()