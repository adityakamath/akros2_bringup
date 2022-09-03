# Copyright (c) 2022 Aditya Kamath
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='port',
            default_value='9090'),
        
        DeclareLaunchArgument(
            name='address',
            default_value=''),
        
        DeclareLaunchArgument(
            name='ssl',
            default_value='false'),
        
        DeclareLaunchArgument(
            name='certfile',
            default_value=''),
        
        DeclareLaunchArgument(
            name='keyfile',
            default_value=''),

        DeclareLaunchArgument(
            name='retry_startup_delay',
            default_value='5.0'),
        
        DeclareLaunchArgument(
            name='fragment_timeout',
            default_value='600'),
        
        DeclareLaunchArgument(
            name='delay_between_messages',
            default_value='0'),
        
        DeclareLaunchArgument(
            name='max_message_size',
            default_value='10000000'),
        
        DeclareLaunchArgument(
            name='unregister_timeout',
            default_value='10.0'),
        
        DeclareLaunchArgument(
            name='use_compression',
            default_value='false'),
        
        DeclareLaunchArgument(
            name='topics_glob',
            default_value=''),
        
        DeclareLaunchArgument(
            name='services_glob',
            default_value=''),
        
        DeclareLaunchArgument(
            name='params_glob',
            default_value=''),
        
        DeclareLaunchArgument(
            name='bson_only_mode',
            default_value='false'),
        
        DeclareLaunchArgument(
            condition=IfCondition(LaunchConfiguration('bson_only_mode')),
            name='binary_encoder',
            default_value='default'),

        Node(
            condition=IfCondition(LaunchConfiguration('ssl')),
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[
                {'certfile': LaunchConfiguration('certfile')},
                {'keyfile': LaunchConfiguration('keyfile')},
                {'port': LaunchConfiguration('port')},
                {'address': LaunchConfiguration('address')},
                {'retry_startup_delay': LaunchConfiguration('retry_startup_delay')},
                {'fragment_timeout': LaunchConfiguration('fragment_timeout')},
                {'delay_between_messages': LaunchConfiguration('delay_between_messages')},
                {'max_message_size': LaunchConfiguration('max_message_size')},
                {'unregister_timeout': LaunchConfiguration('unregister_timeout')},
                {'use_compression': LaunchConfiguration('use_compression')},
                {'topics_glob': LaunchConfiguration('topics_glob')},
                {'services_glob': LaunchConfiguration('services_glob')},
                {'params_glob': LaunchConfiguration('params_glob')},
            ]),
        
        Node(
            condition=UnlessCondition(LaunchConfiguration('ssl')),
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[
                {'port': LaunchConfiguration('port')},
                {'address': LaunchConfiguration('address')},
                {'retry_startup_delay': LaunchConfiguration('retry_startup_delay')},
                {'fragment_timeout': LaunchConfiguration('fragment_timeout')},
                {'delay_between_messages': LaunchConfiguration('delay_between_messages')},
                {'max_message_size': LaunchConfiguration('max_message_size')},
                {'unregister_timeout': LaunchConfiguration('unregister_timeout')},
                {'use_compression': LaunchConfiguration('use_compression')},
                {'topics_glob': LaunchConfiguration('topics_glob')},
                {'services_glob': LaunchConfiguration('services_glob')},
                {'params_glob': LaunchConfiguration('params_glob')},
                {'bson_only_mode': LaunchConfiguration('bson_only_mode')},
            ]),
        
        Node(
            package='rosapi',
            executable='rosapi_node',
            name='rosapi',
            parameters=[
                {'topics_glob': LaunchConfiguration('topics_glob')},
                {'services_glob': LaunchConfiguration('services_glob')},
                {'params_glob': LaunchConfiguration('params_glob')},
            ])

    ])