Usage Instructions

Prerequisites

Ensure you have installed:

    ROS 2 Humble full desktop install

Install required Linux packages:

sudo apt-get install libpthread-stubs0-dev
sudo apt install ros-humble-perception-pcl -y
sudo apt install ros-humble-composition -y

Clone/Download this repo

cd ~/ros2_ws/src/
git clone <repository_url>

Build the package using colcon:

cd ~/ros2_ws/

# Build serial interface package
colcon build --symlink-install --packages-select serial
source install/local_setup.bash

# Build message interfaces package
colcon build --symlink-install --packages-select ti_mmwave_ros2_interfaces
source install/local_setup.bash

# Build main radar processing package (including socket communication)
colcon build --symlink-install --packages-select ti_mmwave_ros2_pkg
source install/local_setup.bash

# Optional: Build example visualizations or additional tools
colcon build --symlink-install --packages-select ti_mmwave_ros2_examples
source install/local_setup.bash

Launching
(To Use with Robofuse lauch the serial and time division code in sperate terminals before this launch command)
Use the provided launch file to start all components:
bash

ros2 launch ti_mmwave_ros2_pkg mmwave_datahdl_socket_launch_rosbag.py

This will:

    Configure the mmWave sensor using its configuration file (6843ISK_3d_0512.cfg).
    Start reading and processing radar data from /dev/ttyUSB0.
    Send processed JSON packets over TCP sockets to 127.0.0.1:65432.
    Publish processed radar data to relevant ROS topics (e.g., /detected_objects,
    range_profile) and publish topics into a bag file during runtime.



To change which configuration file is loaded onto the mmWave sensor during startup, update this line in the launch file:
    # Configuration file for mmWave sensor
    cfg_file = "6843ISK_3d_0512.cfg"


