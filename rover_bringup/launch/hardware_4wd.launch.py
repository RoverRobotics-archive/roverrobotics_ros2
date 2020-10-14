from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    urdf = Path(get_package_share_directory(
        'rover_description'), 'urdf', 'rover.urdf')
    assert urdf.is_file()
    hardware_config = Path(get_package_share_directory(
        'rover_bringup'), 'config', 'hardware_4wd.yaml')
    assert hardware_config.is_file()

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        Node(
            package='rover_driver',
            executable='rover_driver',
            output='screen',
            parameters=[hardware_config],
            arguments=[('__log_level:=debug')],
        ),
        # todo: this publishes static positions for wheel. Switch to publishing wheel position
        # based on encoder data
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
            arguments=[str(urdf)],
            parameters=[hardware_config]
        ),
    ])
