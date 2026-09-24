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
    pika_ros_ws = LaunchConfiguration('pika_ros_ws')
    left_serial_port = LaunchConfiguration('left_serial_port')
    right_serial_port = LaunchConfiguration('right_serial_port')
    official_command = (
        'source "$1/install/setup.bash" && '
        'exec ros2 launch sensor_tools open_multi_sensor_with_teleop.launch.py '
        'l_serial_port:="$2" r_serial_port:="$3"'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_pika_official',
            default_value='false',
            description='Start the official Pika Sense collection nodes.',
        ),
        DeclareLaunchArgument(
            'pika_ros_ws',
            default_value=os.path.join(os.path.expanduser('~'), 'pika_ros'),
            description='Path to the official Pika ROS workspace.',
        ),
        DeclareLaunchArgument('left_serial_port', default_value='/dev/ttyUSB50'),
        DeclareLaunchArgument('right_serial_port', default_value='/dev/ttyUSB51'),
        ExecuteProcess(
            cmd=[
                'bash', '-c', official_command, 'bash', pika_ros_ws,
                left_serial_port, right_serial_port,
            ],
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
