"""
Brings up all the hardware and gets this rover ready to run
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    joy_mapper_prefix = get_package_share_directory('openrover_joy_mapper')
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/hardware.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([joy_mapper_prefix, '/launch/xbox.launch.py'])),
    ])
