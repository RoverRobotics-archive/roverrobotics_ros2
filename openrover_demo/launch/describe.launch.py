import os

import launch
from ament_index_python import get_package_share_directory
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('openrover_demo'), 'urdf', 'rover.urdf')

    nodes = [
        Node(package='robot_state_publisher', node_executable='robot_state_publisher',
             output='screen', arguments=[urdf], node_name='openrover_robot_state_publisher'),
        Node(package='joint_state_publisher', node_executable='joint_state_publisher',
             output='screen', arguments=[urdf], node_name='openrover_joint_state_publisher',
             parameters=[{'publish_default_positions': True}]),
    ]

    events = [RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    ) for node in nodes]

    return launch.LaunchDescription([*nodes, *events])
