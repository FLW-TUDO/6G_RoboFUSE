# 6G RoboFUSE - A Robot Framework for Unified Sensing in Smart Warehouses

## Introduction
This repository proposes the fundamental development of the 6GRoboFUSE (Framework for Unified Sensing and Exploration),
designed for collaborative robotic perception using 6G-based Integrated Sensing and Communication (ISAC).
Thus, this framework enables networked multi-robot Collaborative Perception (CP) in warehouses.
As a first proof of concept, the RoboFUSE provides ISAC Manager and Controller (ISMAC) that emulates the 6G ISAC based on Time Divison (TD) scheduling of sensing (FMCW Radar) and communication (ESP32 OFDM WiFi).

## Applications
* Radio-based SLAM (Simultaneous Localization and Mapping)
* Global map generation using fusion algorithm for warehouse mapping
* Collision avoidance
* D2D communication for collision warning between robots

## Sensing Features
The sensing utilizes a Texas Instrument (TI) mmWave Radar Sensor IWR 6843 ISK. The ROS mmWave toolbox offers straightforward access to specific OOB firmware versions included in TI's mmWave SDKs and Industrial Toolboxes, emphasizing data capture and visualization with RVIZ and written in C++. 
* GUI Monitor (every 10-12 ms depending on buffer)
  * 3D Point Cloud (default CFAR): X, Y, Z coordinates and range, azimuth, elevation, and velocity of each detected object points 
  * range profile 
  * noise profile
  * side information of detected points: SNR and Noise
* Data logger
* Reset sensor via software
* 3D plot real-time visualization of 3D point cloud

## ESP32 Communication Features
* D2D communication for collision avoidance using distance values (Vicon position data) of robots to robots or obstacles
* D2D communication for sharing the local perception data of each robot to other robots

## How to Use

1. cd 6G_RoboFUSE open 3 different terminal tabs
2. Set first tab and open Sensing and run source /home/robot_x/ti/dev/python_tool/Python_venv/mmwave_plot/myenv/bin/activate
3. Set second tab and open ESP32
4. Set third tab to run the main ISMAC program

RUN Step by Step in following:
1. Run the python3 serial_data_ismac.py on second tab
2. Run ISMAC main program python3 time_division_main_controller.py on third tab
3. Press reset on radar sensor and run python py_mmw_main_robomaster.py on first tab

> In progress: ROS2 based mmWave sensor reading using C++

## Publication
Please cite this paper if you want to use this repository.

> Priyanta, Irfan Fachrudin; Freytag, Julia; Reining, Christopher; Roidl, Moritz; Kirchheim, Alice
“Towards 6G-Driven Cooperative Robot Framework for Unified Sensing in Smart Warehouses “
IEEE Conference on Standards for Communication and Networking (CSCN) 2024, Belgrade, Serbia, November 2024. DOI: http://doi.org/10.1109/CSCN63874.2024.10849725 




