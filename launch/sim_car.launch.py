import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    start_gui = LaunchConfiguration('start_gui')
    host = LaunchConfiguration('host')
    port = LaunchConfiguration('port')

    # package share and paths
    ut_automata_share = get_package_share_directory('ut_automata')
    config_dir = os.path.join(ut_automata_share, 'config')

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument(
        'start_gui', default_value='true', description='Enable to start up the gui'))
    
    ld.add_action(DeclareLaunchArgument(
        'host', default_value='127.0.0.1', description='Simulator host IP'))
    
    ld.add_action(DeclareLaunchArgument(
        'port', default_value='9091', description='Simulator port'))

    # simulator_bridge node
    ld.add_action(Node(
        package='av_sim',
        executable='simulator_bridge',
        name='simulator_bridge',
        output='screen',
        parameters=[{
            'host': host,
            'port': port
        }],
        respawn=True,
        respawn_delay=2.0,
    ))

    # joystick node (reusing from ut_automata)
    ld.add_action(Node(
        package='ut_automata',
        executable='joystick',
        name='joystick',
        output='screen',
        arguments=['--config_dir', config_dir],
        respawn=True,
        respawn_delay=2.0,
    ))

    # gui node, only if start_gui is true (reusing from ut_automata)
    ld.add_action(Node(
        package='ut_automata',
        executable='gui',
        name='gui',
        output='screen',
        arguments=['--windowed'],
        respawn=False,
        condition=IfCondition(start_gui),
    ))

    return ld
