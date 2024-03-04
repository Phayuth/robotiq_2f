# Robotiq 2F-85 Gripper ROS2
ROS2 package for Robotiq 2F-85 Minimal Visualization with Real Hardware Control.

## Dependencies
```
sudo pip3 install -U pymodbus
sudo pip3 install pyserial
```
1. Check usb port with `dmesg | egrep --color 'serial|tty'`
2. Give permission to port with `sudo chmod a+rw /dev/ttyUSB1`

## Description
- Gripper can be directly attach to robot (UR5e `tool0`) by publish `robot_state` in another topic instead of include in single `urdf` file.
- Easier extendable with different robot with adding new or modify `urdf.xacro` file.
- Not good when another application is required to read full chain kinematic until the robot's tip such as inverse kinematic chain.
- Mapping value from grip width to tip height since moving gripper will change it's grip location (important if an object is small).

## Usage
### View as standalone test
```
ros2 launch robotiq2f_description view_gripper.launch.py
```
### View with fake hardware and fake robot
launch ur5e robot without rviz2 and then:
```
ros2 launch robotiq2f_description view_connect_to_ur.launch.py
```

### View with real hardware
```
ros2 launch robotiq2f bringup_gripper.launch.py
```
Goto `rviz` panel and add another `robot_state` view and subscribe to `/gripper/robot_state`.

## Reference
- [Robotiq 2F Gripper Driver](https://github.com/KavrakiLab/robotiq_85_gripper)
- [Robotiq PickNick](https://github.com/PickNikRobotics/robotiq_85_gripper)