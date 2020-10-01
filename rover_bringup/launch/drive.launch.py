from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from math import pi


def generate_launch_description():

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/hardware.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/sensors.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/ps4_teleop.launch.py'])),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output='screen',
            arguments=['0', '0', '0.25', '0', '0',
                       '0', 'base_footprint', 'base_link'],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output='screen',
            arguments=['0', '0', '0.05', str(pi), '0', '0', 'base_link', 'laser'],
        ),
    ])
