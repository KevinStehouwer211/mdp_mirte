from launch import LaunchDescription
from launch_ros.actions import Node

# To make be able to switch between the topic when launching 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
        ),

        DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/mirte_base_controller/cmd_vel',
        description='The topic to which the joystick control node will publish velocity commands'
        ),

        Node(
            package='gh_twin',
            executable='joystick_control',
            name='joystick_control',
            output='screen',
            parameters=[{
                'cmd_vel_topic': cmd_vel_topic
            }]
        ),
    ])