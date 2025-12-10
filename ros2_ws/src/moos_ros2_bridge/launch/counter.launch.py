from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_dir = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],
        'share',
        'moos_ros2_bridge',
        'config'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['MOOSDB', os.path.join(config_dir, 'bridge.moos')],
            name='moosdb',
            output='screen'
        ),
        Node(
            package='moos_ros2_bridge',
            executable='moos_ros2_bridge_main',
            name='bridge',
            output='screen',
            arguments=[
                os.path.join(config_dir, 'counters.xml'),
                os.path.join(config_dir, 'bridge.moos')
            ]
        ),
        Node(
            package='moos_ros2_bridge',
            executable='counter_node',
            name='counter',
            output='screen'
        )
    ])