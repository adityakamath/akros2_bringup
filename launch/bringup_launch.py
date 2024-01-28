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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    desc_launch_path = PathJoinSubstitution(
        [FindPackageShare('akros2_description'), 'launch', 'description_launch.py'])

    laser_launch_path = PathJoinSubstitution(
        [FindPackageShare("akros2_base"), 'launch', 'laser_launch.py'])

    camera_launch_path = PathJoinSubstitution(
        [FindPackageShare("akros2_base"), 'launch', 'camera_launch.py'])

    control_launch_path = PathJoinSubstitution(
        [FindPackageShare('akros2_base'), 'launch', 'control_launch.py'])

    fusion_launch_path = PathJoinSubstitution(
        [FindPackageShare('akros2_base'), 'launch', 'sensor_fusion_launch.py'])

    drive_launch_path = PathJoinSubstitution(
        [FindPackageShare('akros2_base'), 'launch', 'drive_launch.py'])

    teleop_launch_path = PathJoinSubstitution(
        [FindPackageShare('akros2_base'), 'launch', 'teleop_launch.py'])

    viz_launch_path = PathJoinSubstitution(
        [FindPackageShare('akros2_bringup'), 'launch', 'viz_launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(
            name='config',
            default_value='mecanum',
            description='Select Robot Config: mecanum (4 wheeled), omni (3 wheeled), diff (2 wheeled)'),

        DeclareLaunchArgument(
            name='joy_config',
            default_value='steamdeck',
            description='Select Controller: ps4 (PS4/DS4), stadia (Google Stadia), sn30pro (8BitDo SN30 Pro), steamdeck (Valve Steam Deck), none (Disabled)'),

        DeclareLaunchArgument(
            name='viz_config',
            default_value='none',
            description='Select Visualization Config: foxglove (Foxglove Bridge), rosbridge (ROSBridge Server), rosboard (ROSBoard), rviz (RViz Config), none (Disabled)'),

        DeclareLaunchArgument(
            name='desc',
            default_value='True',
            description='Enable URDF Description'),

        DeclareLaunchArgument(
            name='laser',
            default_value='True',
            description='Enable LIDAR'),

        DeclareLaunchArgument(
            name='camera',
            default_value='True',
            description='Enable Camera'),

        DeclareLaunchArgument(
            name='control',
            default_value='True',
            description='Enable Low-Level Control'),

        DeclareLaunchArgument(
            name='fusion',
            default_value='True',
            description='Enable Sensor Fusion Nodes'),

        DeclareLaunchArgument(
            name='drive',
            default_value='True',
            description='Enable Drive Nodes'),

        DeclareLaunchArgument(
            name='js_topic',
            default_value='joint_states',
            description='Select Joint States Topic: joint_states (measured), req_states (required)'),

        DeclareLaunchArgument(
            name='js_ext',
            default_value='True',
            description='Enable Joint States from external nodes (like the micro-ROS node). If False, enable Joint States from the Joint State Publisher'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(desc_launch_path),
            condition=IfCondition(LaunchConfiguration('desc')),
            launch_arguments={'config': LaunchConfiguration('config'),
                              'js_ext': LaunchConfiguration('js_ext'),
                              'js_topic': LaunchConfiguration('js_topic')}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(laser_launch_path),
            condition=IfCondition(LaunchConfiguration('laser'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fusion_launch_path),
            condition=IfCondition(LaunchConfiguration('fusion')),
            launch_arguments={'config': LaunchConfiguration('config')}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(drive_launch_path),
            condition=IfCondition(LaunchConfiguration('drive'))),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleop_launch_path),
            launch_arguments={'joy_config': LaunchConfiguration('joy_config')}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(viz_launch_path),
            launch_arguments={'viz_config': LaunchConfiguration('viz_config')}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_path),
            condition=IfCondition(LaunchConfiguration('camera')),
            launch_arguments={'config': LaunchConfiguration('config')}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch_path),
            condition=IfCondition(LaunchConfiguration('control')),
            launch_arguments={'config': LaunchConfiguration('config')}.items()),
    ])
