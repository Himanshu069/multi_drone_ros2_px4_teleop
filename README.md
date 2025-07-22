# ROS2 & PX4 Teleop Example
We previously had a popular [ros2_px4_offboard_example](https://github.com/ARK-Electronics/ROS2_PX4_Offboard_Example) based on Python and the offboard control mode. However, maintaining it became challenging. In this repository, we've migrated the code to C++ and implemented it using a Custom PX4 Flight Mode(PX4 ROS 2 Interface Library) for improved maintainability and performance.

The original implementation was done with PX4 1.16

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

You need the lates PX4-Autopilot
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
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
Navigate to the directory you would like to place the worskpace and then run the following

```
git clone git@github.com:ARK-Electronics/ros2_px4_teleop_example.git
```
Then navigate into the workspace and initialize the submodules:
```
cd ros2_px4_teleop_example/
git submodule update --init --recursive
```
Build the submodules
```
colcon build --packages-select px4_msgs px4_ros2_cpp
```
Source the workspace
```
source install/setup.bash 
```
Build the workspace
```
colcon build
```
After this runs, we do not need to build the whole workspace again, you can just build the individual packages you have modified

```
colcon build --packages-select teleop
```
Source the workspace
```
source install/setup.bash 
```
### Run the example

#### Run the simulation environment
Launch PX4 sim
```
make px4_sitl gz_x500
```
Start the Micro XRCE-DDS Agent:
```
MicroXRCEAgent udp4 -p 8888
```

Launch the Custom Mode Executor:
```
cd ros2_px4_teleop_example/
source install/setup.bash
ros2 run teleop teleop
```

Alternatively, you can use the launch file, in that case you can change the teleop timeout in the config file:
```
source install/setup.bash
ros2 launch teleop teleop.launch.py
```

In a separate terminal, start the teleoperation keyboard:
```
source install/setup.bash
ros2 run teleop_twist_rpyt_keyboard teleop_twist_rpyt_keyboard 
```

ARM the vehicle in QGroundControl, then select the Teleoperation mode.

You should now be able to control the drone using the Teleop_Twist terminal. If there is no input for 60 seconds, the drone will automatically land. You can change this timeout by editing the value in the configuration file.
