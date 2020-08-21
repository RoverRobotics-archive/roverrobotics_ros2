"""
Runs state publishers and state estimation.
State publishers broadcast the position of the rover and its parts in local coordinates.
State estimation computes and broadcasts the rover's location and velocity
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    presence_config = Path(get_package_share_directory('openrover_demo'), 'config', 'presence.yaml')
    assert presence_config.is_file()
    urdf = Path(get_package_share_directory('openrover_demo'), 'urdf', 'rover.urdf')
    assert urdf.is_file()
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    assert presence_config.is_file()
    return LaunchDescription([
        #SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '0'),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            output='screen', arguments=[str(urdf)], parameters=[presence_config]
        ),
        #Node(
        #    package='robot_localization',
        #    executable='ekf_node',
        #    output='screen',
        #    parameters=[presence_config],
        #),
    ])
