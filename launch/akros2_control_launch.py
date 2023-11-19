# Copyright (c) 2023 Aditya Kamath
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():   
    imu_filter_config_dynamic_path = [get_package_share_directory('akros2_bringup'), 
                                     '/config/', 
                                     LaunchConfiguration('config'), 
                                     '_imu_filter_config.yaml']
    
    return LaunchDescription([ 
        DeclareLaunchArgument(
            name='config',
            default_value='mecanum',
            description='Select Robot Config: mecanum (4 wheeled), omni (3 wheeled)'),
        
        DeclareLaunchArgument(
            name='imu_filter',
            default_value='True',
            description='Enable IMU Madgwick Filter from IMU Tools'),
        
        GroupAction(
            condition=LaunchConfigurationEquals('config', 'mecanum'),
            actions = [
                Node(
                    package='micro_ros_agent',
                    executable='micro_ros_agent',
                    name='micro_ros_agent',
                    output='screen',
                    arguments=['serial', '--dev', '/dev/ttyTEENSY']),
                
                Node(
                    condition=IfCondition(LaunchConfiguration('imu_filter')),
                    package='imu_filter_madgwick',
                    executable='imu_filter_madgwick_node',
                    name='imu_filter',
                    output='screen',
                    parameters=[imu_filter_config_dynamic_path],
                    remappings=[
                        ('/imu/data_raw', '/drive/imu'),  #change from /drive/imu to  /imu once micro-ros remapping is done
                        ('/imu/data', '/imu/filtered')],
                ),
            ]),
    ])
