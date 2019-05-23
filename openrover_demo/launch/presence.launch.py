"""
Runs state publishers and state estimation.
State publishers broadcast the position of the rover and its parts in local coordinates.
State estimation computes and broadcasts the rover's location and velocity
"""

from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    yaml = Path(get_package_share_directory('openrover_demo'), 'config', 'presence.yaml')
    urdf = Path(get_package_share_directory('openrover_demo'), 'urdf', 'rover.urdf')
    assert urdf.is_file()

    assert yaml.is_file()
    return launch.LaunchDescription([
        Node(
            package='robot_state_publisher', node_executable='robot_state_publisher',
            output='screen', arguments=[str(urdf)]
        ),
        Node(
            package='joint_state_publisher', node_executable='joint_state_publisher',
            output='screen', arguments=[str(urdf)], parameters=[yaml]
        ),
        Node(
            package='robot_localization',
            node_executable='se_node',
            output='screen',
            parameters=[yaml],
            remappings=[
                ('odometry/filtered', 'odom')
            ]
        ),
    ])
