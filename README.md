# akros2_bringup
ROS2 package to bringup the AKROS2 robot with different configurations - Mecanum Drive (4 wheeled), Omni-Wheel Drive (3 wheeled)


## bringup_launch.py
This is the main launch file that is run on the robot, and can be configured with the following launch arguments:

* ```config```: Selects robot configuration: ```mecanum``` (4-wheeled) or ```omni``` (3-wheeled) (Default: ```mecanum```)
* ```joy_config```: Configures one of the following joystick mappings for launching related packages (Default: ```steamdeck```): 
    * ```ps4```: [PS4](https://www.playstation.com/nl-nl/accessories/dualshock-4-wireless-controller/)/DS4 Controller
    * ```stadia```: [Google Stadia](https://stadia.google.com/controller/) Controller
    * ```sn30pro``` [8BitDo SN30 Pro](https://www.8bitdo.com/sn30-pro-g-classic-or-sn30-pro-sn/) Controller
    * ```steamdeck```: [Valve Steam Deck](https://store.steampowered.com/steamdeck)
    * ```none```: Disabled
* ```viz_config```: Selects one of the following visualization configs for launching related packages (Default: ```foxglove```): 
    * ```foxglove```: Launches [foxglove_bridge](https://github.com/foxglove/ros-foxglove-bridge) for [Foxglove Studio](https://foxglove.dev/docs/studio/connection) (localhost:8765)
    * ```rosbridge```: Launches [rosbridge_server]([rosbridge_server](https://github.com/RobotWebTools/rosbridge_suite/tree/ros2/rosbridge_server/launch) for [Foxglove Studio](https://foxglove.dev/docs/studio/connection) (localhost:9000)
    * ```rosboard```: Launches [ROSBoard](https://github.com/adityakamath/rosboard) server (localhost:8000)
    * ```rviz```: Launches [RViz2](https://github.com/ros2/rviz) locally on the machine with the default configuration in ```rviz/default.rviz``` in this repository.
    * ```none```:  Disabled
  (Note: for rosboard, a personal fork is used, with the port number changed to 8000 instead of 8888, because 8888 is used by Jupyterlab in my environment. It also has some small (warning) fixes for Humble.)
* ```desc```: Enable/Disable the launch of Description related packages: from [akros2_description/description_launch.py](https://github.com/adityakamath/akros2_description) (Default: ```True```)
* ```laser```: Enable/Disable Laser packages in the Laser launch file: [laser_launch.py](https://github.com/adityakamath/akros2_base/blob/humble/launch/laser_launch.py) (Default: ```True```)
* ```camera```: Enable/Disable Camera packages in the Camera launch file: [camera_launch.py](https://github.com/adityakamath/akros2_base/blob/humble/launch/camera_launch.py) (Default: ```True```)
* ```drive```: Enable/Disable Drive packages in the Drive launch file: [drive_launch.py](https://github.com/adityakamath/akros2_base/blob/humble/launch/drive_launch.py) (Default: ```True```)
* ```control```: Enable/Disable low-level control using the Control launch file: [control_launch.py](https://github.com/adityakamath/akros2_bringup/blob/humble/launch/control_launch.py) (Default: ```True```)
* ```fusion```: Enable/Disable the Sensor Fusion related packages with ```config``` specific arguments using the Sensor Fusion launch file: [akros2_base/sensor_fusion_launch.py](https://github.com/adityakamath/akros2_base/blob/humble/launch/sensor_fusion_launch.py) (Default: ```True```)
* ```js_topic```: Choose whether to report measured joint states ```joint_states```, or required joint states ```req_states``` (Default: ```joint_states```)
* ```js_ext```: If True, enables joint states from external nodes (like a [micro-ROS node](https://github.com/adityakamath/akros2_firmware/tree/akros2_humble)). If False, enables joint states from the joint_state_publisher (Default: ```True```)

## bringup_local_launch.py
This is nearly the same as ```bringup_launch.py```, the only difference is that it uses the ```drive_node_launch.py``` instead of ```drive_launch.py``` and ```joy_launch.py``` separately. More info in the [akros2_base](https://github.com/adityakamath/akros2_base) repository.

## Note:
Note that due to issues with launching everything together, it is recommended to launch ```bringup_launch.py``` and ```bringup_local_launch.py``` with ```control:=false camera:=false```. The Control launch file and the Camera launch file should be launched separately on different terminals.

## basestation_launch.py
This is the base station launch file and is meant to be launched separately on-board the robot or from a remote device terminal. While running the basestation, make sure to run the ```bringup_launch.py``` with ```joy_config:=none``` and ```viz_config:=none```. This launch file can be configured using the following launch arguments:

* ```joy_config```: Same as in ```bringup_launch.py``` (Default: ```steamdeck```)
* ```viz_config```: Same as in ```bringup_launch.py``` (Default: ```foxglove```)

## viz_launch.py
This launch file runs the correct visualization package based on the only launch argument:

* ```viz_config```: Same as in ```bringup_launch.py``` (Default: ```foxglove```)

