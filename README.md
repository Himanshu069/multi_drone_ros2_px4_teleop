# ROS2 & PX4 Teleop Example
We previously had a popular [ros2_px4_offboard_example](https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example) based on Python and the offboard control mode. However, maintaining it became challenging. In this repository, we've migrated the code to C++ and implemented it using a custom PX4 flight mode for improved maintainability and performance.

### Video Walkthrough
TODO

### Prerequisites
* Ubuntu 22.04
* ROS2 Humble
* PX4 Autopilot
* Micro XRCE-DDS Agent
* QGroundControl 
* ROS_GZ bridge

### Setup Steps

You can find the required instructions collected below

https://docs.px4.io/main/en/ros2/user_guide.html

You need the lates PX4-Autopilot, that will contain the required drone with the downward facing camera and the world that has the aruco marker in it
To get ros_gz bridge
```
sudo apt install ros-humble-ros-gzgarden
```
https://github.com/gazebosim/ros_gz

## Usage

### Setup the Workspace
Make sure you source ROS2 Humble in the terminal you are using.
```
source /opt/ros/humble/setup.bash
```
OR
Just add the line above to your bashrc, in that case it is going to be sourced every time you open a terminal.
```
nano ~/.bashrc
```