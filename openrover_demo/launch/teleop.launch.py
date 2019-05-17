"""
Launch this on a computer with a keyboard attached.
Listens for arrow keys and sends driving instructions.
"""
import launch
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            package='keystroke', node_executable='keystroke_listen', output='screen',
            parameters=[{'exit_on_esc': True}], arguments=['__log_level:=warn']),
        Node(
            package='keystroke', node_executable='keystroke_arrows_to_twist', output='screen',
            parameters=[{'publish_period': 0.1, 'linear_scale': 0.1, 'angular_scale': 0.2}]),
    ])
