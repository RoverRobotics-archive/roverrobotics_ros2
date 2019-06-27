"""
Launch this on a computer with a keyboard attached.
Listens for arrow keys and sends driving instructions.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    teleop_config = Path(get_package_share_directory('openrover_demo'), 'config', 'teleop.yaml')
    assert teleop_config.is_file()

    nodes = [
        Node(
            package='keystroke', node_executable='keystroke_listen', output='screen',
            arguments=['__log_level:=warn'],
            parameters=[teleop_config]
        ),
        Node(
            package='keystroke', node_executable='keystroke_arrows_to_twist', output='screen',
            arguments=['__log_level:=info'],
            parameters=[teleop_config]
        )]

    events = [RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=n,
            on_exit=[EmitEvent(event=Shutdown())]
        )
    ) for n in nodes]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        *nodes,
        *events
    ])
