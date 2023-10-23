# akros2_bringup
ROS2 package to bringup the AKROS2 robot with different configurations - Mecanum Drive (4 wheeled), Omni-Wheel Drive (3 wheeled)


## akros2_bringup_launch.py
This is the main launch file that is run on the robot, and can be configured with the following launch arguments:

* ```config```: Configures robot platform: ```mecanum``` (4-wheeled) or ```omni``` (3-wheeled) (Default: ```mecanum```)
* ```joy_config```: Configures joystick mapping: ```ps3``` ([PS3/Sixaxis](https://en.wikipedia.org/wiki/Sixaxis)), ```ps4``` ([PS4](https://www.playstation.com/nl-nl/accessories/dualshock-4-wireless-controller/), ```stadia``` ([Stadia](https://stadia.google.com/controller/)), ```sn30pro``` ([8BitDo SN30 Pro](https://www.8bitdo.com/sn30-pro-g-classic-or-sn30-pro-sn/)) (Default: ```ps3```)
* ```viz_config```: Configures visualization websocket for [Foxglove Studio](https://foxglove.dev/docs/studio/connection) or [ROSBoard](https://github.com/adityakamath/rosboard): ```foxglove``` ([foxglove_bridge](https://github.com/foxglove/ros-foxglove-bridge)), ```rosbridge``` ([rosbridge_server]([rosbridge_server](https://github.com/RobotWebTools/rosbridge_suite/tree/ros2/rosbridge_server/launch)), ```rosboard``` or ```none``` (disabled) (Default: ```foxglove```) (Note: for rosboard, a personal fork is used, with the port number changed to 8000 instead of 8888, because 8888 is used by Jupyterlab in my environment. It also has some small (warning) fixes for Humble.)
* ```desc```: Enable/Disable the launch of URDF packages: [robot_state_publisher](https://index.ros.org/r/robot_state_publisher/), [joint_state_publisher](https://index.ros.org/p/joint_state_publisher/) from [akros2_description_launch.py](https://github.com/adityakamath/akros2_description) (Default: ```True```)
* ```lidar```: Enable/Disable the launch of the Laser packages: [ldlidar](https://github.com/linorobot/ldlidar), [laser_filters](https://github.com/ros-perception/laser_filters) (Default: ```True```)
* ```remote```: If False, enable the launch of the Base Station launch file: [akros2_basestation_launch.py](https://github.com/adityakamath/akros2_bringup/blob/humble/launch/akros2_basestation_launch.py). If True, only the [```twist_mixer``` node](https://github.com/adityakamath/akros2_drive/blob/humble/akros2_drive/twist_mixer_node.py) is launched on the robot, everything else is disabled (Default: ```True```). 
* ```control```: If True, enables low-level control using the Control launch file: [akros2_control_launch.py](https://github.com/adityakamath/akros2_bringup/blob/humble/launch/akros2_bringup_launch.py)
* ```js_topic```: Choose whether to report measured joint states ```joint_states```, or required joint states ```req_states``` (Default: ```joint_states```)
* ```js_ext```: If True, enables joint states from external nodes (like a [micro-ROS node](https://github.com/adityakamath/akros2_firmware/tree/akros2_humble)). If False, enables joint states from the joint_state_publisher (Default: ```True```)

## akros2_basestation_launch.py
This is the base station launch file and is meant to be either launched separately on a remote device terminal, or from within the main launch file on the robot. It can be run from within the main launch file using ```remote:=False```. For distributed systems, the main launch file can be run on the robot using ```remote:=True``` and this launch file can be run using ```remote:=True``` on a remote device. This launch file can be configured using the following launch arguments:

* ```remote```: If True, only the [```joy_mode_handler``` node](https://github.com/adityakamath/akros2_drive/blob/humble/akros2_drive/joy_mode_handler.py) and the rest of the Teleop packages (in [akros2_drive_launch.py] are launched. If False, launches the entire AKROS2 Drive launch file: [akros2_drive_launch.py](https://github.com/adityakamath/akros2_drive/blob/humble/launch/akros2_drive_launch.py) (Default: ```True```, ```False``` config is not recommended) 
* ```joy_config```: Same as in ```akros2_bringup_launch.py``` (Default: ```sn30pro```)
* ```viz_config```: Same as in ```akros2_bringup_launch.py``` (Default: ```foxglove```)

## akros2_control_launch.py
This is the low-level control launch file that is meant to be either launched separately on the robot, or from within the main launch using the ```control``` launch argument. This launch files launches specific low-level control nodes based on config. For example, for the mecanum configuration, it launches the [micro-ROS agent](https://github.com/micro-ROS/micro-ROS-Agent) with the correct arguments. The config is the only launch argument for this file:

* ```config```: Launches specific low-level control nodes based on the configs: ```mecanum```, ```omni``` (Default: ```mecanum```)

Note: For now, it is not recommended to launch the low level control for the mecanum configuration from within the main launch file. The mecanum configuration as mentioned above uses the micro-ROS agent, which for some reason keeps dropping connection and reconnecting to the micro-ROS client periodically. When this launch file is run separately (```control:=false``` when running the main launch file), this problem does not occur.