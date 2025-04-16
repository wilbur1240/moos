from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_dir = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(':')[0],
        'share',
        'moos-ros2-bridge',
        'config'
    )

    return LaunchDescription([
        Node(
            package='moos-ros2-bridge',
            executable='moos_ros2_bridge_main',
            name='bridge',
            output='screen',
            arguments=[
                os.path.join(config_dir, 'counters.xml'),
                os.path.join(config_dir, 'bridge.moos')
            ]
        ),
        Node(
            package='moos-ros2-bridge',
            executable='counter',
            name='counter',
            output='screen'
        )
    ])