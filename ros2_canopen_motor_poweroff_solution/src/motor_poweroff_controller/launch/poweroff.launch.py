"""Launch file for motor poweroff node."""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('motor_poweroff_controller')
    config_file = os.path.join(pkg_share, 'config', 'poweroff_config.yaml')

    can_interface = LaunchConfiguration('can_interface')
    node_id = LaunchConfiguration('node_id')

    return LaunchDescription([
        DeclareLaunchArgument(
            'can_interface',
            default_value='can0',
            description='CAN interface name'
        ),
        DeclareLaunchArgument(
            'node_id',
            default_value='1',
            description='CANopen node ID'
        ),

        Node(
            package='motor_poweroff_controller',
            executable='poweroff_node',
            name='motor_poweroff_node',
            parameters=[
                config_file,
                {
                    'can_interface': can_interface,
                    'node_id': node_id,
                },
            ],
            output='screen',
        ),
    ])
