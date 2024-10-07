import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import AnyLaunchDescriptionSource

#Replaces this: ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=True map:=/home/$USER/rebet_ws/map_1713963103.yaml params_file:=/home/$USER/rebet_ws/frog_nav2_params.yaml

def generate_launch_description():
    map_file = os.path.join(
    get_package_share_directory('rebet_frog'),
    'config',
    'map_1713963103.yaml'
    )

    nav2_params_file = os.path.join(
    get_package_share_directory('rebet_frog'),
    'config',
    'frog_nav2_params.yaml'
    )
    nav2_path = get_package_share_directory('nav2_bringup')
    nav2_launch_path = os.path.join(
        nav2_path, 'launch', 'bringup_launch.py')


    nav2 =  IncludeLaunchDescription(
        AnyLaunchDescriptionSource(nav2_launch_path),
        launch_arguments = {
            'use_sim_time' : 'false',
            'map' : map_file,
            'params_file' : nav2_params_file
             }.items(),)


    return LaunchDescription([
        nav2
    ])
