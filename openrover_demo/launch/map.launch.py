"""
Launch this on a payload attached to the rover.
Runs LIDAR scanner and converts the data into a map
"""
from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

odom_yaml = Path(get_package_share_directory('openrover_demo'), 'config/ekf_odom.yaml')
assert odom_yaml.is_file()


def generate_launch_description():
    return launch.LaunchDescription([
        # Node(package='slam_gmapping', node_executable='slam_gmapping', arguments=['__log_level:=debug']),
        Node(
            package='cartographer_ros', node_executable='cartographer_node', output='screen',
            arguments=['-configuration_directory', get_package_share_directory('openrover_demo') + '/config',
                       '-configuration_basename', 'cartographer.lua'
                       ],
            remappings=[('imu', 'imu/data')],
        ),
        Node(
            package='cartographer_ros',
            node_executable='occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),
    ])
