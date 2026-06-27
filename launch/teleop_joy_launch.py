import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('wave_rover_controller')
    default_config = os.path.join(pkg_share, 'config', 'teleop_joy.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Gamepad axis/button mapping and teleop scales.',
        ),
        Node(
            package='joy_linux',
            executable='joy_linux_node',
            name='joy_linux_node',
            parameters=[LaunchConfiguration('config_file')],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
