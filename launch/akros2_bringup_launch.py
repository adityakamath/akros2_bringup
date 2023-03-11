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
    
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('akros2_description'), 'launch', 'akros2_description_launch.py'])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='drive_ns',
            default_value='drive',
            description='Namespace for the Drive subsystem'),
        
        DeclareLaunchArgument(
            name='lidar_ns',
            default_value='ld06',
            description='Namespace for the LiDAR component'),
        
        DeclareLaunchArgument(
            name='tracking_ns',
            default_value='t265',
            description='Namespace for the Tracking Camera component'),
        
        DeclareLaunchArgument(
            name='t265',
            default_value='true',
            description='Enable T265'),
        
        DeclareLaunchArgument(
            name='t265_fisheye',
            default_value='false',
            description='Enable T265 Fisheye Cameras'),
        
        DeclareLaunchArgument(
            name='t265_imu',
            default_value='true',
            description='Enable T265 Accel and Gyro data'),
        
        DeclareLaunchArgument(
            name='drive',
            default_value='true',
            description='Enable Drive'),
        
        DeclareLaunchArgument(
            name='joy',
            default_value='ps4',
            description='Select Game Controller: ps4, ps3, stadia'),
        
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

        DeclareLaunchArgument(
            name='description',
            default_value='true',
            description='Enable URDF description'),
        
        DeclareLaunchArgument(
            name='joint_state_topic',
            default_value='joint_states',
            description='Switch between measured (joint_states) / required (req_states) Joint States'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(drive_launch_path),
            condition=IfCondition(LaunchConfiguration('drive')),
            launch_arguments={'namespace': LaunchConfiguration('drive_ns'),
                              'port_addr': '/dev/ttyUSB_TEENSY',
                              'joy_config': LaunchConfiguration('joy')}.items()),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ld06_launch_path),
            condition=IfCondition(LaunchConfiguration('lidar')),
            launch_arguments={'serial_port': '/dev/ttyUSB_LIDAR',
                              'topic_name': ['/', LaunchConfiguration('lidar_ns'), '/scan'],
                              'lidar_frame': 'laser_frame',
                              'range_threshold': '0.005'}.items()),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(t265_launch_path),
            condition=IfCondition(LaunchConfiguration('t265')),
            launch_arguments={'device_type': 't265',
                              'camera_name': 't265',
                              'enable_pose': 'true',
                              'enable_sync': 'true',
                              'enable_gyro': LaunchConfiguration('t265_imu'),
                              'enable_accel': LaunchConfiguration('t265_imu'),
                              'enable_fisheye1': LaunchConfiguration('t265_fisheye'),
                              'enable_fisheye2': LaunchConfiguration('t265_fisheye'),
                              'linear_accel_cov': '0.001',
                              'unite_imu_method': '2'}.items()), #linear-interpolation
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            condition=IfCondition(LaunchConfiguration('description')),
            launch_arguments={'micro_ros_ns': LaunchConfiguration('drive_ns'),
                              'micro_ros': 'true',
                              'joint_state_topic': LaunchConfiguration('joint_state_topic'),
                              'mesh_pub': LaunchConfiguration('foxglove')}.items()), # publish mesh array if Foxglove is enabled

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