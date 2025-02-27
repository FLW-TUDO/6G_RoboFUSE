#!/bin/bash

# Function to send a command using clipboard paste (Only for ros2 launch & SSH ros2 launch)
send_command() {
  local cmd="$1"
  echo -n "$cmd" | xclip -selection clipboard
  sleep 0.3
  xdotool key ctrl+shift+v
  sleep 0.3
  xdotool key Return
  sleep 1
}

############################
# TERMINATOR WINDOW 1 (Robot_3)
############################
terminator --geometry=1920x1080 --title="Robot_3" &
sleep 3

send_command "export ROS_DOMAIN_ID=3"
send_command "ros2 launch vicon_bridge_ros2 vicon_bridge.launch.py"
sleep 1

xdotool key Shift+Ctrl+O
sleep 1
xdotool key Shift+Ctrl+Right
sleep 1

# ✅ SSH with sourced ROS2 environment (runs ros2 launch)
send_command "sshpass -p 'robomaster' ssh -t robot_3@192.168.2.99"
#send_command "sshpass -p 'robomaster' ssh -t robot_3@192.168.2.80 \"source /opt/ros/humble/setup.bash; source /home/robot_3/MeNu/install/setup.bash; ros2 launch rona_navigation waypoint_bringup_launch.py; exit\""
sleep 1

xdotool key Alt+Up
sleep 1

xdotool key Shift+Ctrl+E
sleep 1
xdotool key Shift+Ctrl+Down
sleep 1

# ✅ SSH with sourced ROS2 environment (navigate & stay interactive)
send_command "sshpass -p 'robomaster' ssh -t robot_3@192.168.2.99 'cd /home/robot_3/6G_RoboFUSE_Dev/Sensing/ros_sensing; bash'"
sleep 1

xdotool key Alt+Down
sleep 1

xdotool key Shift+Ctrl+E
sleep 1
xdotool key Shift+Ctrl+Right
sleep 1

# ✅ SSH with sourced ROS2 environment (navigate & stay interactive)
send_command "sshpass -p 'robomaster' ssh -t robot_3@192.168.2.99 'cd MeNu/src/rona_navigation/scripts; bash'"
sleep 1

xdotool key Alt+Up
sleep 1

xdotool key Shift+Ctrl+E
sleep 1
xdotool key Shift+Ctrl+Right
sleep 1

send_command "cd dev/vicon_bridge/vicon-mqtt-subscriber-py/"

xdotool key Alt+Down
sleep 1

xdotool key shift+ctrl+E
sleep 1
xdotool key shift+ctrl+Right

xdotool key Shift+Ctrl+Right
sleep 1
send_command "export ROS_DOMAIN_ID=3"
sleep 1
send_command "ros2 launch robomaster_nav2_bringup rviz_launch.py namespace:=ep03 use_namespace:=true rviz_config:='/home/irfanflw/ros2_ws_galactic/src/robomaster_nav2_bringup/config/rviz/nav2_namespaced_view.rviz'"
sleep 5

############################
# TERMINATOR WINDOW 2 (Robot_5)
############################
terminator --geometry=1920x1080 --title="Robot_5" &
sleep 4

send_command "export ROS_DOMAIN_ID=5"
send_command "ros2 launch vicon_bridge_ros2 vicon_bridge.launch.py"
sleep 1

xdotool key Shift+Ctrl+O
sleep 1
xdotool key Shift+Ctrl+Right
sleep 1

# ✅ SSH with sourced ROS2 environment (runs ros2 launch)
send_command "sshpass -p 'robomaster' ssh -t robot_5@192.168.2.161"
#send_command "sshpass -p 'robomaster' ssh -t robot_5@192.168.2.161 \"source /opt/ros/humble/setup.bash; source /home/robot_5/MeNu/install/setup.bash; ros2 launch rona_navigation waypoint_bringup_launch.py; exit\""
sleep 1

xdotool key Alt+Up
sleep 1

xdotool key Shift+Ctrl+E
sleep 1
xdotool key Shift+Ctrl+Down
sleep 1

# ✅ SSH with sourced ROS2 environment (navigate & stay interactive)
send_command "sshpass -p 'robomaster' ssh -t robot_5@192.168.2.161 'cd /home/robot_5/6G_RoboFUSE_Dev/Sensing/ros_sensing; bash'"
sleep 1

xdotool key Alt+Down
sleep 1

xdotool key Shift+Ctrl+E
sleep 1
xdotool key Shift+Ctrl+Right
sleep 1

# ✅ SSH with sourced ROS2 environment (navigate & stay interactive)
send_command "sshpass -p 'robomaster' ssh -t robot_5@192.168.2.161 'cd MeNu/src/isaac_nav/rona_navigation/scripts; bash'"
sleep 1

xdotool key Alt+Up
sleep 1

xdotool key shift+ctrl+E
sleep 1
xdotool key shift+ctrl+Right
sleep 1

send_command "sshpass -p 'raspi' ssh pi@192.168.2.84 'sudo docker run -it --rm --network host -v /home/pi/ros2_mmwave:/ros2_mmwave -v /etc/localtime:/etc/localtime:ro --device=/dev/ttyACM0 --device=/dev/ttyACM1 -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp -e ROS_DOMAIN_ID=20 docker-ros-ti'"
sleep 2

xdotool key Alt+Down
sleep 1

xdotool key shift+ctrl+E
sleep 1

xdotool key shift+ctrl+Right
sleep 1
send_command "export ROS_DOMAIN_ID=5"
sleep 1
send_command "ros2 launch robomaster_nav2_bringup rviz_launch.py namespace:=ep05 use_namespace:=true rviz_config:='/home/irfanflw/ros2_ws_galactic/src/robomaster_nav2_bringup/config/rviz/nav2_namespaced_view.rviz'"
sleep 5

############################
# TERMINATOR WINDOW 3 (Vicon Costmap)
############################
terminator --geometry=1920x1080 --title="Vicon_Costmap Robot 3" &
sleep 5

#send_command "cd MeNu && source install/local_setup.bash && ros2 launch rona_physical vicon_tf_converter.launch.py namespace:=AS_1_neu"
#sleep 7

for ns in AS_1_neu AS_3_neu AS_4_neu AS_5_neu AS_6_neu ep05; do
  send_command "sshpass -p 'robomaster' ssh -t robot_3@192.168.2.99 'cd MeNu; bash'"
  sleep 5
  send_command "source install/local_setup.bash && ros2 launch rona_physical vicon_tf_converter.launch.py namespace:=$ns"
  sleep 2
  xdotool key Shift+Ctrl+E
  sleep 1
  xdotool key Shift+Ctrl+Left
  sleep 1
done

sleep 1
send_command "sshpass -p 'robomaster' ssh -t robot_3@192.168.2.99 'cd MeNu; bash'"
sleep 5
send_command "source install/local_setup.bash && ros2 run rona_physical clear_costmap_on_goal.py"
sleep 1

############################
# TERMINATOR WINDOW 4 (Vicon Costmap)
############################
terminator --geometry=1920x1080 --title="Vicon_Costmap Robot 5" &
sleep 4

#send_command "cd MeNu && source install/local_setup.bash && ros2 launch rona_physical vicon_tf_converter.launch.py namespace:=AS_1_neu"
#sleep 7

for ns in AS_1_neu AS_3_neu AS_4_neu AS_5_neu AS_6_neu ep03; do
  send_command "sshpass -p 'robomaster' ssh -t robot_5@192.168.2.161 'cd MeNu; bash'"
  sleep 5
  send_command "source install/local_setup.bash && ros2 launch rona_physical vicon_tf_converter.launch.py namespace:=$ns"
  sleep 2
  xdotool key Shift+Ctrl+E
  sleep 1
  xdotool key Shift+Ctrl+Left
  sleep 1
done

sleep 1
send_command "sshpass -p 'robomaster' ssh -t robot_5@192.168.2.161 'cd MeNu; bash'"
sleep 5
send_command "source install/local_setup.bash && ros2 run rona_physical clear_costmap_on_goal.py"
sleep 1

echo "✅ All four Terminator windows are now running with correctly split panes!"

