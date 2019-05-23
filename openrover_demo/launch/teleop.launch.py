"""
Launch this on a computer with a keyboard attached.
Listens for arrow keys and sends driving instructions.
"""
from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    params = Path(get_package_share_directory('openrover_demo'), 'config', 'teleop.yaml')

    nodes = [
        Node(
            package='keystroke', node_executable='keystroke_listen', output='screen',
            arguments=['__log_level:=warn'],
            parameters=[params]
        ),
        Node(
            package='keystroke', node_executable='keystroke_arrows_to_twist', output='screen',
            arguments=['__log_level:=info'],
            parameters=[params]
        )]

    events = [RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=n,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    ) for n in nodes]

    return launch.LaunchDescription([
        *nodes, *events
    ])
