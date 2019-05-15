from pathlib import Path

import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    odom_yaml = Path(get_package_share_directory('openrover_demo'), 'config/ekf_odom.yaml')

    assert odom_yaml.is_file()
    return launch.LaunchDescription([
        Node(
            package='robot_localization',
            node_executable='se_node',
            node_name='se_node',
            output='screen',
            parameters=[odom_yaml],
            arguments=['__log_level:=info'],
        ),
    ])
