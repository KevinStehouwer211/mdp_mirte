from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Define the Audio Feedback Node
    audio_node = Node(
        package='mirte_indicator_devices',          # Replace with your actual package name
        executable='audio_node',              # Matches the string in setup.py console_scripts
        name='robot_audio_node',
        output='screen',
        emulate_tty=True                      # Keeps console outputs cleanly formatted
    )

    # 2. Define the LED Controller Node
    led_node = Node(
        package='mirte_indicator_devices',          # Replace with your actual package name
        executable='led_controller_node',                # Matches the string in setup.py console_scripts
        name='led_controller_node',
        output='screen',
        emulate_tty=True
    )

    # 3. Return the description containing both nodes
    return LaunchDescription([
        audio_node,
        led_node
    ])