"""
Brings up all the hardware and gets this rover ready to run
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

bringup_dir = get_package_share_directory('nav2_bringup')
launch_dir = os.path.join(bringup_dir, 'launch')
def generate_launch_description():
    return LaunchDescription([
        #SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '0'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/hardware.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/presence.launch.py'])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir,
                                                       'bringup_launch.py')),
            launch_arguments={
                              'map': "/home/ros/willmap.yaml"}.items()),
        #IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
    ])
