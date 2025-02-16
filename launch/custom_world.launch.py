#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import EnvironmentVariable, TextSubstitution
from launch.actions import AppendEnvironmentVariable
from launch.actions import LogInfo

def generate_launch_description():

    gz_models = "GAZEBO_MODEL_PATH"
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Run with gui (true/false)')
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='acsos2024.world',
        description='Gazebo world to use, default acsos2024')


    

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')




    worlds_directory = os.path.join(
        get_package_share_directory('rebet_frog'),'worlds',)


    models_file_dir = os.path.join(get_package_share_directory('rebet_frog'), 'models')
    add_model_path = AppendEnvironmentVariable(name=gz_models, value=models_file_dir)

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': (worlds_directory, os.path.sep, LaunchConfiguration('world')), 'verbose': 'true'}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=LaunchConfigurationEquals('gui', 'true')
        
    )

    return LaunchDescription(
        [add_model_path, gui_arg, world_arg, gzserver_cmd, gzclient_cmd],
    )


