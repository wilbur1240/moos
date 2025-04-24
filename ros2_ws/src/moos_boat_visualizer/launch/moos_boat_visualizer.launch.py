from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('moos-ros2-bridge'),
        'config'
    )

    moos_mission_dir = '/home/wilbur/moos/moos-ivp-wilbur/missions/'
    mission = 'lab_05/alpha'

    urdf_file = os.path.join(
        FindPackageShare('moos_boat_visualizer').find('moos_boat_visualizer'),
        'urdf',
        'boat.urdf.xacro'
    )

    rviz_config_dir = os.path.join(
        get_package_share_directory('moos_boat_visualizer'),
        'rviz',
        'moos_boat.rviz'
    )

    return LaunchDescription([
        # ExecuteProcess(
        #     cmd=['MOOSDB', os.path.join(moos_mission_dir, mission, 'alpha.moos')],
        #     name='moosdb',
        #     output='screen'
        # ),
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{
        #         'robot_description': Command(['xacro ', urdf_file])
        #     }],
        # ),
        Node(
            package='moos-ros2-bridge',
            executable='moos_ros2_bridge_main',
            name='bridge',
            output='screen',
            arguments=[
                os.path.join(config_dir, 'moos_boat.xml'),
                os.path.join(moos_mission_dir, mission, 'alpha.moos')
            ]
        ),
        Node(
            package='moos_boat_visualizer',
            executable='moos_boat_pose_publisher',
            name='pose_publisher',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_dir],
        ),
    ])