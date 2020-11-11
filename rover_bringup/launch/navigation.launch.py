from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os


def generate_launch_description():
    nav_launch_dir = os.path.join(get_package_share_directory(
        'nav2_bringup'), 'launch')
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(nav_launch_dir, 'slam_launch.py')]))
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(nav_launch_dir, 'navigation_launch.py')]))

    return LaunchDescription([
        slam_launch,
        navigation_launch
    ])