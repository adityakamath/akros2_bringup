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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('akros2_description'), 'launch', 'akros2_description_launch.py'])
    
    laser_filter_config_path = PathJoinSubstitution(
        [FindPackageShare("akros2_bringup"), "config", "laser_filter_config.yaml"])
    
    basestation_launch_path = PathJoinSubstitution(
        [FindPackageShare('akros2_bringup'), 'launch', 'akros2_basestation_launch.py'])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='config',
            default_value='mecanum',
            description='Select Robot Config: mecanum (4 wheeled), omni (3 wheeled)'),
        
        DeclareLaunchArgument(
            name='joy_config',
            default_value='ps3',
            description='Select Controller: ps3/sixaxis, ps4, stadia, sn30pro'),
        
        DeclareLaunchArgument(
            name='viz_config',
            default_value='none',
            description='Select Visualization Config: foxglove (Foxglove Bridge), rosbridge (ROSBridge Server), rosboard (ROSBoard), none (Disabled)'),
        
        DeclareLaunchArgument(
            name='desc',
            default_value='True',
            description='Enable URDF Description'),
        
        DeclareLaunchArgument(
            name='lidar',
            default_value='True',
            description='Enable LIDAR'),
        
        DeclareLaunchArgument(
            name='remote',
            default_value='True',
            description='If True, Teleop/Visualization disabled (only Twist Mixer is enabled), needs to be run remotely. If False, launch on-board Teleop/Visualization'),
        
        DeclareLaunchArgument(
            name='js_topic',
            default_value='joint_states',
            description='Select Joint States Topic: joint_states (measured), req_states (required)'),
        
        DeclareLaunchArgument(
            name='js_ext',
            default_value='True',
            description='Enable Joint States from external nodes (like the micro-ROS node). If False, enable Joint States from the Joint State Publisher'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            condition=IfCondition(LaunchConfiguration('desc')),
            launch_arguments={'config': LaunchConfiguration('config'),
                              'js_ext': LaunchConfiguration('js_ext'),
                              'js_topic': LaunchConfiguration('js_topic')}.items()),
        
        GroupAction(
            condition=IfCondition(LaunchConfiguration('lidar')),
            actions = [
                Node(
                    package='ldlidar',
                    executable='ldlidar',
                    name='laser_node',
                    output='screen',
                    parameters=[{'serial_port': '/dev/ttyLIDAR'},
                                {'topic_name': 'scan'},
                                {'lidar_frame': 'laser_frame'},
                                {'range_threshold': 0.005}
                    ]),
                
                Node(
                    package='laser_filters',
                    executable='scan_to_scan_filter_chain',
                    name='laser_filter_node',
                    parameters=[laser_filter_config_path]),
            ]),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(basestation_launch_path),
            condition=UnlessCondition(LaunchConfiguration('remote')),
            launch_arguments={'remote': LaunchConfiguration('remote'),
                              'joy_config': LaunchConfiguration('joy_config'),
                              'viz_config': LaunchConfiguration('viz_config')}.items()),
        
        Node(
            condition=IfCondition(LaunchConfiguration('remote')),
            package='akros2_drive',
            executable='twist_mixer',
            name='twist_mixer',
            output='screen',
            remappings=[
                ('/teleop_vel', '/drive/joy_vel'),
                ('/auto_vel', '/drive/nav_vel'),
                ('/mix_vel', '/drive/cmd_vel'),
                ('/mode', '/drive/mode'),
            ]),
    ])