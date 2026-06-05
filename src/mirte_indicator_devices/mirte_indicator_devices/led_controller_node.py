import rclpy
from rclpy.node import Node

# Replace these with your actual message and service imports
from mirte_msgs.srv import SetNeopixel
from mirte_msgs.msg import NeopixelColor, SetSpeed
from std_msgs.msg import String

class LedControllerNode(Node):
    def __init__(self):
        super().__init__('led_controller_node')

        # --- Configurations ---
        self.speed_threshold = 0.5
        
        # Color Definitions (adjust to match your service's expected format)
        self.COLOR_RED = (255, 0, 0)
        self.COLOR_GREEN = (0, 255, 0)
        self.COLOR_BLUE = (0, 0, 255)
        self.COLOR_OFF = (0, 0, 0)

        # --- State Tracking ---
        self.is_in_error = False
        self.op_mode = 'unknown'
        self.robot_is_moving = False
        
        # Blinking state
        self.blink_is_on = True 
        
        self.motor_states = {
            'front_left': False,
            'front_right': False,
            'rear_left': False,
            'rear_right': False
        }

        # --- Service Client ---
        self.led_client = self.create_client(SetNeopixel, '/io/leds/leds/set_color')

        # --- Subscriptions ---
        self.state_sub = self.create_subscription(String, 'robot_state', self.state_callback, 10)
        self.mode_sub = self.create_subscription(String, 'operation_mode', self.op_mode_callback, 10)

        # Motor subscriptions
        def create_motor_callback(motor_name):
            return lambda msg: self.motor_speed_callback(msg, motor_name)

        self.fl_sub = self.create_subscription(SetSpeed, '/io/motor/front_left/speed', create_motor_callback('front_left'), 10)
        self.fr_sub = self.create_subscription(SetSpeed, '/io/motor/front_right/speed', create_motor_callback('front_right'), 10)
        self.rl_sub = self.create_subscription(SetSpeed, '/io/motor/rear_left/speed', create_motor_callback('rear_left'), 10)
        self.rr_sub = self.create_subscription(SetSpeed, '/io/motor/rear_right/speed', create_motor_callback('rear_right'), 10)

        # --- Timer for Blinking ---
        # 1.0 second timer to toggle LEDs on and off when moving
        self.blink_timer = self.create_timer(1.0, self.blink_timer_callback)

        self.get_logger().info("LED Controller Node started.")
        self.update_leds()  # Set initial state

    def set_led_color(self, color):
        """Asynchronously calls the service to update LED colors."""
        if not self.led_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().debug("LED service not available yet.")
            return

        request = SetNeopixel.Request()
        
        color_msg = NeopixelColor()
        color_msg.r = int(color[0])
        color_msg.g = int(color[1])
        color_msg.b = int(color[2])

        request.color = color_msg

        # Async call to avoid blocking the ROS 2 executor
        self.led_client.call_async(request)

    def update_leds(self):
        """Calculates the desired color based on priorities and blinking state."""
        # Priority 1: Error State
        if self.is_in_error:
            target_color = self.COLOR_RED
        # Priority 2: Operation Modes
        elif self.op_mode == 'move_manual':
            target_color = self.COLOR_BLUE
        elif self.op_mode == 'automatic':
            target_color = self.COLOR_GREEN
        else:
            target_color = self.COLOR_OFF

        # Apply Blinking Logic: Override with OFF if moving and in the 'off' phase of the blink
        if self.robot_is_moving and not self.blink_is_on:
            self.set_led_color(self.COLOR_OFF)
        else:
            self.set_led_color(target_color)

    # --- Callbacks ---
    def state_callback(self, msg: String):
        new_error_state = (msg.data.lower() == 'error')
        if self.is_in_error != new_error_state:
            self.is_in_error = new_error_state
            self.update_leds()

    def op_mode_callback(self, msg: String):
        if self.op_mode != msg.data.lower():
            self.op_mode = msg.data.lower()
            self.update_leds()

    def motor_speed_callback(self, msg: SetSpeed, motor_name: str):
        is_motor_moving = abs(msg.speed) > self.speed_threshold
        self.motor_states[motor_name] = is_motor_moving

        any_motor_moving = any(self.motor_states.values())

        if any_motor_moving and not self.robot_is_moving:
            self.robot_is_moving = True
            # Reset blink state so it immediately turns ON when movement starts
            self.blink_is_on = True 
            self.update_leds()
            
        elif not any_motor_moving and self.robot_is_moving:
            self.robot_is_moving = False
            self.blink_is_on = True # Ensure it stays solid when stopped
            self.update_leds()

    def blink_timer_callback(self):
        """Toggles the blink state every second. Only pushes update if moving."""
        self.blink_is_on = not self.blink_is_on
        if self.robot_is_moving:
            self.update_leds()

def main(args=None):
    rclpy.init(args=args)
    node = LedControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down LED node.")
        # Attempt to turn off LEDs on shutdown
        node.set_led_color(node.COLOR_OFF)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()