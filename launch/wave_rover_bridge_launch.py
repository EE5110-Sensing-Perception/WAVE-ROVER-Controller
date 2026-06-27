import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('wave_rover_controller')
    bridge_config = os.path.join(pkg_share, 'config', 'wave_rover_bridge.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bridge_config_file',
            default_value=bridge_config,
            description='WiFi/HTTP bridge parameters.',
        ),
        Node(
            package='wave_rover_controller',
            executable='wave_rover_bridge.py',
            name='waverover_bridge',
            parameters=[LaunchConfiguration('bridge_config_file')],
            output='screen',
        ),
    ])
