from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rviz_config = Path(get_package_share_directory('rover_demo'), 'config', 'default.rviz').resolve()
    assert rviz_config.is_file()

    return LaunchDescription([
        DeclareLaunchArgument('frame', default_value='map', description='The fixed frame to be used in RViz'),
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        ExecuteProcess(cmd=[
                'rviz2',
                '--display-config', str(rviz_config),
                '--fixed-frame', LaunchConfiguration(variable_name='frame')
            ],
            output='screen'),
    ])
