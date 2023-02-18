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
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='drive_ns',
            default_value='drive',
            description='Namespace for the Drive subsystem'),
        
        DeclareLaunchArgument(
            name='sensor_ns',
            default_value='sensor',
            description='Namespace for the Sensors subsystem'),
        
        DeclareLaunchArgument(
            name='t265',
            default_value='true',
            description='Enable T265'),
        
        DeclareLaunchArgument(
            name='drive',
            default_value='true',
            description='Enable Drive'),
        
        DeclareLaunchArgument(
            name='lidar',
            default_value='true',
            description='Enable LIDAR'),
        
        DeclareLaunchArgument(
            name='foxglove',
            default_value='true',
            description='Enable Foxglove Bridge'),
        
        DeclareLaunchArgument(
            name='rosboard',
            default_value='false',
            description='Enable ROSBoard'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(drive_launch_path),
            condition=IfCondition(LaunchConfiguration('drive')),
            launch_arguments={'namespace': LaunchConfiguration('drive_ns'),
                              'port_addr': '/dev/ttyUSB_TEENSY',
                              'joy_config': 'ps4'}.items()),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ld06_launch_path),
            condition=IfCondition(LaunchConfiguration('lidar')),
            launch_arguments={'serial_port': '/dev/ttyUSB_LIDAR',
                              'topic_name': ['/', LaunchConfiguration('sensor_ns'), '/scan'],
                              'lidar_frame': 'laser_frame',
                              'range_threshold': '0.005'}.items()),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(t265_launch_path),
            condition=IfCondition(LaunchConfiguration('t265')),
            launch_arguments={'device_type': 't265',
                              'enable_pose': 'true',
                              'enable_gyro': 'true',
                              'enable_accel': 'true',
                              'enable_fisheye1': 'true',
                              'enable_fisheye2': 'true',
                              'linear_accel_cov': '0.001',
                              'unite_imu_method': '2'}.items()), #linear-interpolation
        
        Node(
            condition=IfCondition(LaunchConfiguration('foxglove')),
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove'),
        
        Node(
            condition=IfCondition(LaunchConfiguration('rosboard')),
            package='rosboard',
            executable='rosboard_node',
            name='rosboard')

    ])