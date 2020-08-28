from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    controller_config = Path(get_package_share_directory('openrover_bringup'), 'config', 'controller.yaml')
    assert controller_config.is_file()
    topics_config = Path(get_package_share_directory('openrover_bringup'), 'config', 'topics.yaml')
    assert topics_config.is_file()

    return LaunchDescription([
        Node(
            package='openrover_joy_mapper', executable='mapper_node.py', output='screen',
            parameters=[{"controller": str(controller_config), "topics": str(topics_config)}]
        ),
        Node(
            package='joy', executable='joy_node', output='screen',
            parameters=[{'dev': '/dev/input/js0'}]
        ),
    ])
