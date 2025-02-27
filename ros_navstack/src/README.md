# Robomaster Navigation RoNa

In this repository, you will find the necessary packages to use navigation on the Robomaster using waypoints via MQTT and the robot follower. 
Follow the instructions to install all the required libraries and packages.

## Table of Contents

- [Introduction](#introduction)
- [Installation](#installation)
- [Usage](#usage)


## Introduction

This project is based on ROS2 Humble. It includes a simple navigation stack that receives waypoints through MQTT messages and the position using the vicon system . The package for controlling the robot can be found in the "robomaster_setup" package.


## Installation

First we need to install all the dependencies:
```
pip install paho-mqtt
sudo apt install ros-humble-rmw-cyclonedds-cpp
```

## Usage
the user computer Need to use  cycloneDDs and same Domain
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

and same domain  
```
export ROS_DOMAIN_ID=130
```
SSH on the robot
the current IP of the robots conected to the FLW_CPS:
robot_2:192.168.2.115
robot_3:192.168.2.80
robot_5:192.168.2.161
or you can use the following command 
    ssh robot_3@ep03.local (e.g. for robomaster 3) 
    password: robomaster

 (after ssh on the robot)

The following command launch the bringup for the waypoint follower using the mqtt and the position of the robot via Vicon 

```
ros2 launch rona_navigation waypoint_bringup_launch.py
```
If you wan to use the follower code use the launcher on the follower robot 
```
ros2 launch rona_navigation follower_bringup_launch.py
```
and In the followed robot use this command 
```
ros2 run rona_navigation base_footprint_pose_publisher
```
 To use the navigation code inside of the robot without the mqtt receiver 

```
ros2 launch rona_navigation robomaster_nav_launch.py
```

The launch file for the mqtt waypoint receiver is
```
ros2 launch rona_mqtt mqtt_waypoint_receiver_launch.py namespace:=ep03/ep02/ep05
```

to see the rviz from the user computer use the next command:
```
ros2 launch robomaster_nav2_bringup rviz_launch.py namespace:=rm04 use_namespace:=true rviz_config:="/wk_name/src/robomaster_nav2_bringup/config/rviz/nav2_namespaced_view.rviz"
```
## Issues

1. **Tunning of the parameters of the controller***
The parameters are tune for robots moving an average of 1,0m/s 

