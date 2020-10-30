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
        # IncludeLaunchDescription(PythonLaunchDescriptionSource(
        #     [ThisLaunchFileDir(), '/xbox_teleop.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/hardware_4wd.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/sensors.launch.py'])),
        # IncludeLaunchDescription(PythonLaunchDescriptionSource(
        #     [ThisLaunchFileDir(), '/sensor_fusion.launch.py'])),
    ])
