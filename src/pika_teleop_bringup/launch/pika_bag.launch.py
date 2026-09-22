"""Start the lightweight Pika Bag/Demo stack without production services."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config_path = os.path.join(
        get_package_share_directory('pika_teleop_bringup'),
        'config',
        'ros',
        'pika_bag_config.yam',
    )
    start_pika_official = LaunchConfiguration('start_pika_official')
    official_command = (
        'source /opt/ros/humble/setup.bash && '
        'source /home/lei/pika_ros/install/setup.bash && '
        'exec bash '
        '/home/lei/pika_ros/scripts/start_multi_sensor_whit_teleop.bash'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_pika_official',
            default_value='false',
            description='Start the official Pika Sense collection nodes.',
        ),
        ExecuteProcess(
            cmd=['bash', '-c', official_command],
            condition=IfCondition(start_pika_official),
            output='screen',
        ),
        Node(
            package='pika_teleop_virtual_receiver',
            executable='virtual_receiver',
            name='pika_teleop_virtual_receiver',
            parameters=[config_path],
            output='screen',
        ),
        Node(
            package='pika_teleop_bridge',
            executable='pika_teleop_publisher',
            name='pika_teleop_publisher',
            parameters=[config_path],
            output='screen',
        ),
        Node(
            package='pika_realman_mapper',
            executable='pika_realman_mapper',
            name='pika_realman_mapper',
            parameters=[config_path],
            output='screen',
        ),
    ])
