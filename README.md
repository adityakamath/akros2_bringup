# akros2_bringup
ROS2 package to bringup the AKROS2 robot with different configurations - Mecanum Drive (4 wheeled), Omni-Wheel Drive (3 wheeled)


## akros2_bringup_launch.py
This is the main launch file that is run on the robot, and can be configured with the following launch arguments:

* ```config```: Configures robot platform: ```mecanum``` (4-wheeled) or ```omni``` (3-wheeled) (Default: ```mecanum```)
* ```joy_config```: Configures joystick: ```ps3``` ([PS3/Sixaxis](https://en.wikipedia.org/wiki/Sixaxis)), ```ps4``` ([PS4](https://www.playstation.com/nl-nl/accessories/dualshock-4-wireless-controller/), ```stadia``` ([Stadia](https://stadia.google.com/controller/)), ```sn30pro``` ([8BitDo SN30 Pro](https://www.8bitdo.com/sn30-pro-g-classic-or-sn30-pro-sn/)) (Default: ```ps3```)
* ```viz_config```: Configures visualization websocket for [Foxglove Studio](https://foxglove.dev/docs/studio/connection): ```foxglove``` ([foxglove_bridge](https://github.com/foxglove/ros-foxglove-bridge)), ```rosbridge``` ([rosbridge_server]([rosbridge_server](https://github.com/RobotWebTools/rosbridge_suite/tree/ros2/rosbridge_server/launch)) or ```none``` (disabled) (Default: ```foxglove```)
* ```desc```: Enable/Disable the launch of URDF packages: [robot_state_publisher](https://index.ros.org/r/robot_state_publisher/), [joint_state_publisher](https://index.ros.org/p/joint_state_publisher/) from [akros2_description_launch.py](https://github.com/adityakamath/akros2_description) (Default: ```True```)
* ```lidar```: Enable/Disable the launch of the Laser packages: [ldlidar](https://github.com/linorobot/ldlidar), [laser_filters](https://github.com/ros-perception/laser_filters) (Default: ```True```)
* ```remote```: If False, enable the launch of the Base Station launch file: [akros2_basestation_launch.py](https://github.com/adityakamath/akros2_bringup/blob/humble/launch/akros2_basestation_launch.py). If True, only the [Twist Mixer node](https://github.com/adityakamath/akros2_drive/blob/humble/akros2_drive/twist_mixer_node.py) is launched on the robot, everything else is disabled (Default: ```True```). 
* ```js_topic```: Choose whether to report measured joint states ```joint_states```, or required joint states ```req_states``` (Default: ```joint_states```)
* ```js_uros```: If True, enables joint states from external nodes (like a [micro-ROS node](https://github.com/adityakamath/akros2_firmware/tree/akros2_humble)). If False, enables joint states from the joint_state_publisher (Default: ```True```)

## akros2_basestation_launch.py
This is the base station launch file and is meant to be either launched separately on a remote device terminal, or from within the main launch file on a robot. It can be run from within the main launch file using ```remote:=False```. For distributed systems, the main launch file can be run on the robot using ```remote:=True``` and this launch file can be run using ```remote:=True``` on a remote device. This launch file can be configured using the following launch arguments:

* ```remote```: If True, only the [Joystick Mode Handler node](https://github.com/adityakamath/akros2_drive/blob/humble/akros2_drive/joy_mode_handler.py) and the rest of the Teleop packages (in [akros2_drive_launch.py] are launched. If False, launches the entire AKROS2 Drive launch file: [akros2_drive_launch.py](https://github.com/adityakamath/akros2_drive/blob/humble/launch/akros2_drive_launch.py) (Default: ```True```, ```False``` config is not recommended) 
* ```joy_config```: Same as in ```akros2_bringup_launch.py```
* ```viz_config```: Same as in ```akros2_bringup_launch.py```