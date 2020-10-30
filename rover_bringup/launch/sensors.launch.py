from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from pathlib import Path
import os


def generate_launch_description():
    sensor_config = Path(get_package_share_directory(
        'rover_bringup'), 'config', 'sensor.yaml')

    realsense_launch_file = os.path.join(get_package_share_directory(
        'realsense_examples'), 'launch', 'rs_camera.launch.py')
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([realsense_launch_file]))
    print(sensor_config)
    rplidar_node = Node(
        name='rplidarNode',
        package='rplidar_ros',
        executable='rplidarNode',
        output='screen',
        parameters=[sensor_config],
    )

    bno055_node = Node(
        package='bno055_driver',
        executable='bno055_driver',
        output='screen',
        parameters=[sensor_config]
    )

    return LaunchDescription([
        rplidar_node,
        # bno055_node,
        # realsense_launch,
    ])
