# TurtleBot_Roomba
A simple turtleBot3 walker algorithm much like a Roomba robot vacuum cleaner. The robot moves forward until it reaches an obstacle (but does not collide), then rotates in place until the way ahead is clear, then move forward again and repeat.

[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Overview
TurtleBot_Roomba is a turtleBot3-based robot that moves like a Roomba robot. It can move in any environment while avoiding static and dynamic obstacles.
The robot continuously scans it environment for obstacles and if it encounters an obstacle it rotates at it place in a direction that would make its orientation away from the obstacle until there is no obstacle.

The **```turtlebot_roomba_node```** subscribes to the **```\scan```** topic which provides it with the laser scan data (distances of obstacles from the robot) and based on the distances it publishes velocity commands on **```\cmd_vel```**

# Dependencies
* Ubuntu 18.04 (LTS)
* ROS Melodic
* ROS TurtleBot3 Package
* Gazebo

## Build Instructions
* Create a workspace
```
mkdir -p ~/roomba_ws/src
cd ~/roomba_ws/src
```

* Clone the repository inside the src directory of the workspace
```
git clone https://github.com/mjoshi07/turtlebot_roomba.git
```
* Build the workspace
```
cd ~/roomba_ws
catkin_make 
source devel/setup.bash
```

## Install TurtleBot3
```
cd ~/roomba_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
cd ~/roomba_ws
catkin_make
source devel/setup.bash
```
* Set turtlebot environment variable in .bashrc
```
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
```

## Run Instructions
### Launch everything with launch file, by default record_rosbag argument is set to false
```
roslaunch turtlebot_roomba turtlebot_roomba.launch
```
* enable the record_rosbag argument and it wil record a rosbag for 30 seconds and save rosbag file in the results directory
```
roslaunch turtlebot_roomba turtlebot_roomba.launch record_rosbag:=true
```
**```NOTE```** - rosbag will **NOT** record any messages published at ```/camera``` topic

### Run with rosrun
* open a new terminal
```
cd ~/roomba_ws
source devel/setup.bash
roslaunch turtleBot3_gazebo turtleBot3_world.launch
```
* open a new terminal
```
cd ~/roomba_ws
source devel/setup.bash
rosrun turtlebot_roomba turtlebot_roomba_node
```
## Set ROS logger level
* while turtlebot_roomba_node is running
```
rosservice call /turtlebot_roomba/set_logger_level "{logger: 'rosout', level: 'debug'}"
```
