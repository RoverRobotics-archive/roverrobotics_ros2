"""
Brings up all the hardware and gets this rover ready to run
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/hardware.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/presence.launch.py'])),
    ])
