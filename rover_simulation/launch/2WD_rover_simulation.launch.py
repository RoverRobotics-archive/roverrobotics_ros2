import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    simulation_launch_dir = os.path.join(get_package_share_directory('rover_simulation'), 'launch')
    navigation_launch_dir = os.path.join(get_package_share_directory('rover_navigation'), 'launch')
    bringup_launch_dir = os.path.join(get_package_share_directory('rover_bringup'), 'launch')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(simulation_launch_dir, 'gazebo.launch.py')),
        launch_arguments={}.items())

    rover_spawn_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(simulation_launch_dir, 'spawn_2WD_rover.launch.py')),
        launch_arguments={}.items())

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(navigation_launch_dir, 'nav2.launch.py')),
        launch_arguments={}.items())

    teleop_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir, 'ps4_teleop.launch.py')),
        launch_arguments={}.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any conditioned actions
    ld.add_action(gazebo_cmd)
    ld.add_action(rover_spawn_cmd)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(teleop_cmd)

    return ld