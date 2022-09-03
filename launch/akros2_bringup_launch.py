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
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():

    ld06_launch_path = PathJoinSubstitution(
        [FindPackageShare('ldlidar'), 'launch', 'ldlidar.launch.py'])
    
    t265_launch_path = PathJoinSubstitution(
        [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])

    drive_launch_path = PathJoinSubstitution(
        [FindPackageShare('akros2_drive'), 'launch', 'akros2_drive_launch.py'])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='namespace',
            default_value='akros2',
            description='Namespace of the robot'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ld06_launch_path),
            launch_arguments={'serial_port': '/dev/ttyUSB_LIDAR',
                              'topic_name': ['/', LaunchConfiguration('namespace'), '/scan'],
                              'lidar_frame': 'camera_pose_frame',
                              'range_threshold': '0.005'}.items()),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(t265_launch_path),
            launch_arguments={'device_type': 't265',
                              'enable_pose': 'true',
                              'enable_fisheye1': 'false',
                              'enable_fisheye2': 'false',
                              'linear_accel_cov': '0.001',
                              'unite_imu_method': 'linear_interpolation',
                              'initial_reset': 'true'}.items()),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(drive_launch_path),
            launch_arguments={'namespace': LaunchConfiguration('namespace'),
                              'port_addr': '/dev/ttyUSB_TEENSY',
                              'ps4_addr': '84:30:95:2C:67:7C'}.items()),

    ])