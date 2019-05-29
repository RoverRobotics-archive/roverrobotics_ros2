# based on navigation2/nav2_bringup/launch/nav2_bringup_launch.py
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import time

import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription


# ros2 launch openrover_demo nav2.launch.py map:=install/share/openrover_demo/maps/map.yaml
def generate_launch_description():
    map_yaml_file = launch.substitutions.LaunchConfiguration('map')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    params_file = launch.substitutions.LaunchConfiguration('params', default=os.path.join(
        get_package_share_directory('openrover_demo'), 'config', 'nav2.yaml'))

    bt_navigator_install_path = get_package_prefix('nav2_bt_navigator')
    bt_navigator_xml = os.path.join(bt_navigator_install_path,
                                    'behavior_trees',
                                    'navigate_w_recovery_retry.xml')  # TODO(mkhansen): change to an input parameter

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'map', description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        launch_ros.actions.Node(
            package='nav2_map_server',
            node_executable='map_server',
            node_name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, {'yaml_filename': map_yaml_file}]),
        # lots of problems in the launch file related to spinning up too much at the same time.
        # this ugly hack spaces things out to prevent crashes.
        launch.actions.OpaqueFunction(function=(lambda *args: time.sleep(1))),
        launch_ros.actions.Node(
            package='nav2_world_model',
            node_executable='world_model',
            output='screen',
            parameters=[params_file]
        ),
        launch_ros.actions.Node(
            package='nav2_amcl',
            node_executable='amcl',
            output='screen',
            parameters=[params_file],
        ),
        launch_ros.actions.Node(
            package='dwb_controller',
            node_executable='dwb_controller',
            output='screen',
            parameters=[params_file]),
        launch.actions.OpaqueFunction(function=(lambda *args: time.sleep(1))),
        launch_ros.actions.Node(
            package='nav2_navfn_planner',
            node_executable='navfn_planner',
            output='screen',
        ),
        launch_ros.actions.Node(
            package='nav2_motion_primitives',
            node_executable='motion_primitives_node',
            output='screen',
        ),
        launch.actions.OpaqueFunction(function=(lambda *args: time.sleep(1))),
        launch_ros.actions.Node(
            package='nav2_bt_navigator',
            node_name='bt_navigator',
            node_executable='bt_navigator',
            output='screen',
            parameters=[params_file, {
                'bt_xml_filename': bt_navigator_xml}]
        )
    ])
