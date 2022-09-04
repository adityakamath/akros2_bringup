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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    drive_launch_path = PathJoinSubstitution(
        [FindPackageShare('akros2_drive'), 'launch', 'akros2_drive_launch.py'])

    ld06_launch_path = PathJoinSubstitution(
        [FindPackageShare('ldlidar'), 'launch', 'ldlidar.launch.py'])
    
    t265_launch_path = PathJoinSubstitution(
        [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
    
    rosbridge_launch_path = PathJoinSubstitution(
        [FindPackageShare('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='namespace',
            default_value='akros2',
            description='Namespace of the robot'),
        
        DeclareLaunchArgument(
            name='t265',
            default_value='false',
            description='Enable Realsense T265 Pose'),
        
        DeclareLaunchArgument(
            name='viz_foxglove',
            default_value='false',
            description='Enable Foxglove visualization using ROSBridge'),
        
        DeclareLaunchArgument(
            name='viz_rosboard',
            default_value='true',
            description='Enable visualization using ROSBoard'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(drive_launch_path),
            launch_arguments={'namespace': LaunchConfiguration('namespace'),
                              'port_addr': '/dev/ttyUSB_TEENSY',
                              'ps4_addr': '84:30:95:2C:67:7C'}.items()),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ld06_launch_path),
            launch_arguments={'serial_port': '/dev/ttyUSB_LIDAR',
                              'topic_name': ['/', LaunchConfiguration('namespace'), '/scan'],
                              'lidar_frame': 'laser_frame',
                              'range_threshold': '0.005'}.items()),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(t265_launch_path),
            condition=IfCondition(LaunchConfiguration('t265')),
            launch_arguments={'device_type': 't265',
                              'enable_pose': 'true',
                              'enable_fisheye1': 'false',
                              'enable_fisheye2': 'false',
                              'linear_accel_cov': '0.001',
                              'unite_imu_method': 'linear_interpolation'}.items()),
        
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(rosbridge_launch_path),
            condition=IfCondition(LaunchConfiguration('viz_foxglove'))),
        
        Node(
            condition=IfCondition(LaunchConfiguration('viz_rosboard')),
            package='rosboard',
            executable='rosboard_node',
            name='rosboard')

    ])