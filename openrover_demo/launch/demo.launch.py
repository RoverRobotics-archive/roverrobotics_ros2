import launch

from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

import sys

print(sys.path)


def generate_launch_description():
    # todo: when ExecutableComposeNodeProcess is launched, use it.
    nodes = [
        Node(
            package='keystroke', node_executable='keystroke_listen', output='screen', node_name='keystroke_listen2',
            parameters=[{'exit_on_esc': True}], arguments=['__log_level:=warn']),
        Node(
            package='keystroke', node_executable='keystroke_arrows_to_twist', output='screen',
            node_name='arrows_to_twist2',
            parameters=[{'publish_period': 0.1, 'linear_scale': 0.1, 'angular_scale': 0.2}]),
        Node(
            package='openrover_core', node_executable='openrover', output='screen',
            arguments=[])
    ]

    events = [RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    ) for node in nodes]

    return launch.LaunchDescription([*nodes, *events])
