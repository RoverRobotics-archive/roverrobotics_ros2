"""
Interfaces via USB with the hardware of the robot.
Drives the rover in response to teleop messages and publishes odometry from motor encoders.
Runs LIDAR sensor to scan visible obstacles.
Runs IMU to measure orientation and acceleration.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    urdf = Path(get_package_share_directory('openrover_demo'), 'urdf', 'rover.urdf')
    assert urdf.is_file()
    hardware_config = Path(get_package_share_directory('openrover_demo'), 'config', 'hardware.yaml')
    assert hardware_config.is_file()

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        Node(
            package='openrover_core', node_executable='rover', output='screen',
            parameters=[hardware_config]
        ),
        Node(
            package='bno055_driver',
            node_executable='bno055_driver',
            output='screen',
            parameters=[hardware_config]
        ),
        Node(
            package='rplidar_ros',
            node_executable='rplidarNode',
            output='screen',
            parameters=[hardware_config],
        ),
        # todo: this publishes static positions for wheel. Switch to publishing wheel position
        # based on encoder data
        Node(
            package='joint_state_publisher', node_executable='joint_state_publisher',
            output='screen', arguments=[str(urdf)], parameters=[hardware_config]
        ),
    ])
