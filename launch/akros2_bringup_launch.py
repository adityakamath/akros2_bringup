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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():

    drive_launch_path = PathJoinSubstitution(
        [FindPackageShare('akros2_drive'), 'launch', 'akros2_drive_launch.py'])

    ld06_launch_path = PathJoinSubstitution(
        [FindPackageShare('ldlidar'), 'launch', 'ldlidar.launch.py'])
    
    laser_filter_config_path = PathJoinSubstitution(
        [FindPackageShare("akros2_bringup"), "config", "laser_filter_config.yaml"])
    
    t265_launch_path = PathJoinSubstitution(
        [FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
    
    description_launch_path = PathJoinSubstitution(
        [FindPackageShare('akros2_description'), 'launch', 'akros2_description_launch.py'])
    
    rosbridge_launch_path = PathJoinSubstitution(
        [FindPackageShare('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'])
    
    foxglove_launch_path = PathJoinSubstitution(
        [FindPackageShare('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml'])
    
    return LaunchDescription([ 
        DeclareLaunchArgument(
            name='drive',
            default_value='True',
            description='Enable Drive'),
        
        DeclareLaunchArgument(
            name='drive_ns',
            default_value='drive',
            description='Namespace for the Drive subsystem'),
        
        DeclareLaunchArgument(
            name='joy',
            default_value='ps3',
            description='Select controller: ps3/sixaxis, ps4, stadia, sn30pro'),
        
        DeclareLaunchArgument(
            name='description',
            default_value='True',
            description='Enable URDF description'),
        
        DeclareLaunchArgument(
            name='joint_state_topic',
            default_value='joint_states',
            description='Switch between measured (joint_states) / required (req_states) Joint States'),
        
        DeclareLaunchArgument(
            name='lidar',
            default_value='True',
            description='Enable LIDAR'),
        
        DeclareLaunchArgument(
            name='lidar_ns',
            default_value='lidar',
            description='Namespace for the LIDAR component'),
        
        DeclareLaunchArgument(
            name='lidar_port',
            default_value='/dev/ttyLIDAR',
            description='LIDAR Port Address'),
        
        DeclareLaunchArgument(
            name='t265',
            default_value='false',
            description='Enable T265'),
        
        DeclareLaunchArgument(
            name='tracking_ns',
            default_value='t265',
            description='Namespace for the Tracking Camera component'),
        
        DeclareLaunchArgument(
<<<<<<< HEAD
            name='t265',
            default_value='True',
            description='Enable T265'),
        
        DeclareLaunchArgument(
            name='rosbridge',
            default_value='True',
            description='Enable ROSBridge Server'),
=======
            name='viz',
            default_value='false',
            description='Enable Visualization'),
        
        DeclareLaunchArgument(
            name='foxglove',
            default_value='true',
            description='Enable Foxglove Server'),
>>>>>>> c1bb969 (update launch file)
        
        DeclareLaunchArgument(
            name='rosboard',
            default_value='False',
            description='Enable ROSBoard'),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(drive_launch_path),
            condition=IfCondition(LaunchConfiguration('drive')),
            launch_arguments={'ns': LaunchConfiguration('drive_ns'),
                              'joy_config': LaunchConfiguration('joy')}.items()),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch_path),
            condition=IfCondition(LaunchConfiguration('description')),
            launch_arguments={'uros_ns': LaunchConfiguration('drive_ns'),
                              'uros': LaunchConfiguration('drive'),
                              'joint_state_topic': LaunchConfiguration('joint_state_topic')}.items()),
        
        GroupAction(
            condition=IfCondition(LaunchConfiguration('lidar')),
            actions = [
                Node(
                    package='ldlidar',
                    executable='ldlidar',
                    name='ld06',
                    output='screen',
                    namespace=LaunchConfiguration("lidar_ns"),
                    parameters=[{'serial_port': LaunchConfiguration("lidar_port")},
                                {'topic_name': ['/', LaunchConfiguration('lidar_ns'), '/scan']},
                                {'lidar_frame': 'laser_frame'},
                                {'range_threshold': 0.005}
                    ]),
        
                Node(
                    package="laser_filters",
                    executable="scan_to_scan_filter_chain",
                    name="laser_filter",
                    parameters=[laser_filter_config_path],
                    remappings=[
                        (['/scan'], ['/', LaunchConfiguration('lidar_ns'), '/scan']),
                        (['/scan_filtered'], ['/', LaunchConfiguration('lidar_ns'), '/scan_filtered'])
                    ]),
            ]),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(t265_launch_path),
            condition=IfCondition(LaunchConfiguration('t265')),
            launch_arguments={'device_type': LaunchConfiguration('tracking_ns'),
                              'camera_name': LaunchConfiguration('tracking_ns'),
                              'enable_pose': 'true',
                              'enable_sync': 'true',
                              'enable_gyro': 'false',
                              'enable_accel': 'false',
                              'enable_fisheye1': 'false',
                              'enable_fisheye2': 'false',
                              'linear_accel_cov': '0.001',
                              'unite_imu_method': '2'}.items()), #linear-interpolation
        
        GroupAction(
            condition=IfCondition(LaunchConfiguration('viz')),
            actions = [
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource(foxglove_launch_path),
                    condition=IfCondition(LaunchConfiguration('foxglove')),
                    launch_arguments={
                        'port': '8765',
                        'use_compression': 'true',
                    }.items()),
        
<<<<<<< HEAD
        Node(
            condition=IfCondition(LaunchConfiguration('rosboard')),
            package='rosboard',
            executable='rosboard_node',
            name='rosboard'),
    ])
=======
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource(rosbridge_launch_path),
                    condition=UnlessCondition(LaunchConfiguration('foxglove')), 
                    launch_arguments={
                        'port': '9090',
                        'use_compression': 'true',
                    }.items()),
                
                Node(
                    condition=IfCondition(LaunchConfiguration('rosboard')),
                    package='rosboard',
                    executable='rosboard_node',
                    name='rosboard'),
            ]),
    ])
>>>>>>> c1bb969 (update launch file)
