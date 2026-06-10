import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('wave_rover_controller')
    driver_config = os.path.join(pkg_share, 'config', 'wave_rover_controller.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'driver_config_file',
            default_value=driver_config,
            description='Driver and diff-drive parameters.',
        ),
        Node(
            package='wave_rover_controller',
            namespace='wave_rover_controller',
            executable='wave_rover_controller',
            name='robot',
            parameters=[LaunchConfiguration('driver_config_file')],
        ),
    ])
