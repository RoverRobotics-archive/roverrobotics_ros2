"""
Launch this on a payload tethered to the rover.
Drives the rover in response to teleop messages and publishes odometry
"""

import launch
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    # todo: when ExecutableComposeNodeProcess is launched, use it.
    nodes = [
        Node(
            package='openrover_core', node_executable='openrover', output='screen',
            arguments=['port:=/dev/ttyUSB0'])
    ]

    events = [RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    ) for node in nodes]

    return launch.LaunchDescription([*nodes, *events])
