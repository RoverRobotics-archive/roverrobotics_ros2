"""
Launch this on a payload tethered to the rover.
Drives the rover in response to teleop messages and publishes odometry
Runs LIDAR scanner
Runs state publishers
"""
import os
from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    urdf = os.path.join(get_package_share_directory('openrover_demo'), 'urdf', 'rover.urdf')
    assert Path(urdf).is_file()
    drive_yaml = Path(get_package_share_directory('openrover_demo'), 'config', 'drive.yaml')
    assert drive_yaml.is_file()
    nodes = [
        Node(
            package='robot_state_publisher', node_executable='robot_state_publisher',
            output='screen', arguments=[urdf]
        ),
        Node(
            package='joint_state_publisher', node_executable='joint_state_publisher',
            output='screen', arguments=[urdf], parameters=[drive_yaml]
        ),
        Node(
            package='openrover_core', node_executable='openrover', output='screen',
            parameters=[drive_yaml]
        ),
        Node(package='bno055_driver',
             node_executable='bno055_driver',
             output='screen',
             parameters=[drive_yaml]),
        Node(
            package='rplidar_ros',
            node_executable='rplidarNode',
            output='screen',
            parameters=[drive_yaml],
        ),
    ]

    return launch.LaunchDescription([*nodes])
