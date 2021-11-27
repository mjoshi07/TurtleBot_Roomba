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

## Rosbag details
* set the launch_gazebo flag to false in the launch command, assuming rosbag file was recorded
```
roslaunch turtlebot_roomba turtlebot_roomba.launch launch_gazebo:=false 
```
* check the recorded ros bag
```
rosbag info rosbag_topics_record_2021-11-27-01-45-21.bag 
```
* you should see something like this, check there should be no topic /camera
```
path:        rosbag_topics_record_2021-11-27-01-45-21.bag
version:     2.0
duration:    30.0s
start:       Dec 31 1969 19:00:00.17 (0.17)
end:         Dec 31 1969 19:00:30.14 (30.14)
size:        48.8 MB
messages:    126159
compression: none [64/64 chunks]
types:       dynamic_reconfigure/Config            [958f16a05573709014982821e6822580]
             dynamic_reconfigure/ConfigDescription [757ce9d44ba8ddd801bb30bc456f946f]
             gazebo_msgs/LinkStates                [48c080191eb15c41858319b4d8a609c2]
             gazebo_msgs/ModelStates               [48c080191eb15c41858319b4d8a609c2]
             geometry_msgs/Twist                   [9f195f881246fdfa2798d1d3eebca84a]
             nav_msgs/Odometry                     [cd5e73d190d741a2f92e81eda573aca7]
             rosgraph_msgs/Clock                   [a9c97c1d230cfc112e270351a944ee47]
             rosgraph_msgs/Log                     [acffd30cd6b6de30f120938c17c593fb]
             sensor_msgs/Imu                       [6a62c6daae103f4ff57a132d6f95cec2]
             sensor_msgs/JointState                [3066dcd76a6cfaef579bd0f34173e9fd]
             sensor_msgs/LaserScan                 [90c7ef2dc6895d81024acba2ac42f369]
             tf2_msgs/TFMessage                    [94810edda583a504dfda3829e70d7eec]
topics:      /clock                           29963 msgs    : rosgraph_msgs/Clock                  
             /cmd_vel                           149 msgs    : geometry_msgs/Twist                  
             /gazebo/link_states              29877 msgs    : gazebo_msgs/LinkStates               
             /gazebo/model_states             29874 msgs    : gazebo_msgs/ModelStates              
             /gazebo/parameter_descriptions       1 msg     : dynamic_reconfigure/ConfigDescription
             /gazebo/parameter_updates            1 msg     : dynamic_reconfigure/Config           
             /imu                             26413 msgs    : sensor_msgs/Imu                      
             /joint_states                      897 msgs    : sensor_msgs/JointState               
             /odom                              897 msgs    : nav_msgs/Odometry                    
             /rosout                           3525 msgs    : rosgraph_msgs/Log                     (4 connections)
             /rosout_agg                       3516 msgs    : rosgraph_msgs/Log                    
             /scan                              149 msgs    : sensor_msgs/LaserScan                
             /tf                                897 msgs    : tf2_msgs/TFMessage

```
* play the recorded ros bag
```
rosbag play rosbag_topics_record_2021-11-27-01-45-21.bag
```
* verify this by running the rqt graph
```
rqt_graph
```
* you should see something like this
![image](https://github.com/mjoshi07/turtlebot_roomba/blob/Week13_HW/results/rosbag_verified.png)
