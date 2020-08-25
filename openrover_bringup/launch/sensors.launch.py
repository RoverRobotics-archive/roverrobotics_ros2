from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

rplidar_dir = get_package_share_directory('rplidar_ros')
rplidar_s1_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([rplidar_dir, '/launch/rplidar_s1.launch.py']))

def generate_launch_description():
    return LaunchDescription({
        rplidar_s1_launch,
    })