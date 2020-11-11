from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from math import pi

import os


def generate_launch_description():
    nav_launch_dir = os.path.join(get_package_share_directory(
        'nav2_bringup'), 'launch')
    nav_yaml = os.path.join(get_package_share_directory(
        'rover_navigation'), 'config', 'nav2.yaml')
    map_yaml = os.path.join(get_package_share_directory(
        'rover_navigation'), 'maps', 'office.yaml')
    
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav_launch_dir, 'bringup_launch.py'),
            ]),
            launch_arguments={
                'map': map_yaml,
                'params_file': nav_yaml
                }.items()
        )

    return LaunchDescription([
        nav_launch
    ])