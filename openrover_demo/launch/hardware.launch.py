"""
Interfaces via USB with the hardware of the robot.
Drives the rover in response to teleop messages and publishes odometry from motor encoders.
Runs LIDAR sensor to scan visible obstacles.
Runs IMU to measure orientation and acceleration.
"""
from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    yaml = Path(get_package_share_directory('openrover_demo'), 'config', 'hardware.yaml')
    assert yaml.is_file()
    nodes = [
        Node(
            package='openrover_core', node_executable='rover', output='screen',
            parameters=[yaml]
        ),
        Node(
            package='bno055_driver',
            node_executable='bno055_driver',
            output='screen',
            parameters=[yaml]
        ),
        Node(
            package='rplidar_ros',
            node_executable='rplidarNode',
            output='screen',
            parameters=[yaml],
        ),
    ]

    return launch.LaunchDescription([*nodes])
