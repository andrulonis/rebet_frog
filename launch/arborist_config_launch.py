from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
def generate_launch_description():
    config_file = os.path.join(
    get_package_share_directory('rebet_frog'),
    'config',
    'frog_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='rebet_frog',
            executable='frog_arborist',
            name='frog_arborist_node',
            output='screen',
            parameters=[config_file,{}]
        ),
    ])