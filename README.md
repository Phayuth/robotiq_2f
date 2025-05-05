# Robotiq Gripper ROS2
ROS2 package for Robotiq Gripper for Minimal Visualization with Real Hardware Control. Featuring gripper :
- Robotiq 85
- Robotiq Hand-E

Software architecture:

`gripperio (for all gripper) -> driver (for each gripper) -> ROS2 server (for each gripper)`

## Caution
There will be some error in value due to conversion between signal value and real value that result from round off error, and round down in value.

## Dependencies and Installation
```bash
sudo pip3 install pyserial
```
1. Check usb port with `dmesg | egrep --color 'serial|tty'`
2. Give permission to port with `sudo chmod a+rw /dev/ttyUSB1`

Clone and build the pacakge as usual with `colcon build`

## Description
- Gripper can be directly attach to robot (UR5e `tool0`) by publish `robot_state` in another topic instead of include in single `urdf` file.
- Easier extendable with different robot with adding new or modify `urdf.xacro` file.
- Not good when another application is required to read full chain kinematic until the robot's tip such as inverse kinematic chain.
- Mapping value from grip width to tip height since moving gripper will change it's grip location (important if an object is small).

## Usage
### View as fake hardware standalone test with joint state publisher gui

- Robotiq 85 ```ros2 launch robotiq2f_description r85_view_gripper.launch.py```
- Robotiq Hand-E ```ros2 launch robotiq2f_description rhande_view_gripper.launch.py```

### View with real hardware
- Robotiq 85 ```ros2 launch robotiq2f r85_bringup.launch.py```
- Robotiq Hand-E ```ros2 launch robotiq2f rhande_bringup.launch.py ```

Goto `rviz` panel and add another `robot_state` view and subscribe to `/robot_state`.

## Reference
#### Robotiq 85
- [Robotiq 2F Gripper Driver](https://github.com/KavrakiLab/robotiq_85_gripper)
- [Robotiq PickNick](https://github.com/PickNikRobotics/robotiq_85_gripper)
#### Robotiq Hand-E
- [AcutronicRobotics](https://github.com/AcutronicRobotics/robotiq_modular_gripper)
- [macs-lab](https://github.com/macs-lab/robotiq_hande_ros_driver)