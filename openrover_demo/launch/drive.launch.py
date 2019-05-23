"""
Brings up all the hardware and gets this rover ready to run
"""

import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    return launch.LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/hardware.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/presence.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/teleop.launch.py'])),
    ])
