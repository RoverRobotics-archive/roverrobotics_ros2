import launch
import launch_ros.actions

def generate_launch_description():
    # want to use ExecuteComposableNodeProcess, but it's not in Crystal
    openrover = launch_ros.actions.Node(
        package='openrover_core', node_executable='openrover', output='screen',
        arguments = ['port:=COM3'])
    #client = launch_ros.actions.Node(
    #    package='demo_nodes_cpp', node_executable='add_two_ints_client', output='screen')
    return launch.LaunchDescription([
        openrover,
    ])