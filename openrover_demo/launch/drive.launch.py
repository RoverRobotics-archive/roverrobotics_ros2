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
    odom_yaml = Path(get_package_share_directory('openrover_demo'), 'config/ekf_odom.yaml')
    assert odom_yaml.is_file()
    imu_yaml = Path(get_package_share_directory('openrover_demo'), 'config/bno055.yaml')
    assert imu_yaml.is_file()
    nodes = [
        Node(package='robot_state_publisher', node_executable='robot_state_publisher',
             output='screen', arguments=[urdf], node_name='openrover_robot_state_publisher'),
        Node(package='joint_state_publisher', node_executable='joint_state_publisher',
             output='screen', arguments=[urdf], node_name='openrover_joint_state_publisher',
             parameters=[{'publish_default_positions': True}]),
        Node(
            package='openrover_core', node_executable='openrover', output='screen',
            arguments=['port:=/dev/rover']
        ),
        Node(
            package='robot_localization',
            node_executable='se_node',
            node_name='robot_localization_node',
            output='screen',
            parameters=[odom_yaml],
        ),
        Node(package='bno055_driver',
             node_executable='bno055_driver',
             node_name='bno055_driver',
             output='screen',
             parameters=[imu_yaml]),
        Node(
            node_name='rplidarNode',
            package='rplidar_ros',
            node_executable='rplidarNode',
            output='screen',
            parameters=[{
                'serial_port': '/dev/lidar',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
    ]

    events = [

        #     RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=node,
        #         on_exit=[EmitEvent(event=Shutdown())],
        #     )
        # ) for node in nodes

    ]

    return launch.LaunchDescription([*nodes, *events])
