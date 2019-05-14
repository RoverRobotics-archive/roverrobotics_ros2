"""
Launch this on a payload attached to the rover.
Runs LIDAR scanner and converts the data into a map
"""

import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    return launch.LaunchDescription([
        Node(
            node_name='rplidarNode',
            package='rplidar_ros',
            node_executable='rplidarNode',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB2',
                'serial_baudrate': 115200,  # A1 / A2
                # 'serial_baudrate': 256000, # A3
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
        ),
        # Node(package='slam_gmapping', node_executable='slam_gmapping', arguments=['__log_level:=debug']),
        Node(
            package='cartographer_ros', node_executable='cartographer_node', output='screen',
            node_name='cartographer_node',
            arguments=['-configuration_directory', get_package_share_directory('openrover_demo') + '/config/',
                       '-configuration_basename', 'cartographer.lua'
                       ],
            parameters=[{}]),
        Node(
            package='cartographer_ros',
            node_executable='occupancy_grid_node',
            node_name="occupancy_grid_node",
            output='screen',
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
        # Node(
        #     package='openrover_core', node_executable='openrover', output='screen',
        #     arguments=[]),
    ])
