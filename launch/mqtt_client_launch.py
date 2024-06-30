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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mqtt_client_config_path = PathJoinSubstitution(
        [FindPackageShare("akros2_bringup"), "config", "mqtt_client_config.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument(
            name='broker_host',
            default_value='ak-win11',
            description='Hostname of the MQTT Broker'),

        DeclareLaunchArgument(
            name='broker_port',
            default_value=1883,
            description='Port used by the MQTT Broker'),

        Node(
            package='mqtt_client',
            executable='mqtt_client',
            name='mqtt_client_node',
            output='screen',
            parameters=[{'broker': {'host': LaunchConfiguration('broker_host'),
                                    'port': LaunchConfiguration('broker_port')}},
                        mqtt_client_config_path]),
    ])