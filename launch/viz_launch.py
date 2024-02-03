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
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    rosbridge_launch_path = PathJoinSubstitution(
        [FindPackageShare('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml'])

    foxglove_launch_path = PathJoinSubstitution(
        [FindPackageShare('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml'])

    rviz_config = PathJoinSubstitution(
        [FindPackageShare('akros2_bringup'), 'viz', 'default.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument(
            name='viz_config',
            default_value='none',
            description='Select Visualization Config: foxglove (Foxglove Bridge), rosbridge (ROSBridge Server), rviz (RViz Config), none (Disabled)'),

        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(foxglove_launch_path),
            condition=LaunchConfigurationEquals('viz_config', 'foxglove'),
            launch_arguments={
                'port': '8765',
                'use_compression': 'true',
            }.items()),

        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(rosbridge_launch_path),
            condition=LaunchConfigurationEquals('viz_config', 'rosbridge'),
            launch_arguments={
                'port': '9090',
                'use_compression': 'true',
            }.items()),

        Node(
            condition=LaunchConfigurationEquals('viz_config', 'rviz'),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')],
            output='screen'),
    ])
