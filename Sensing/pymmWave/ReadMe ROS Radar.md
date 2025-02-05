1. open a (first) terminal, navigate to /home/robot_3/6G_RoboFUSE/Sensing
	using: cd 6G_RoboFUSE/Sensing

2. open a second terminal, source the mmwave_ws setup
	using: source ~/mmwave_ws/install/setup.bash
	
3. run RViz in the second terminal
	using: rviz2

4. in the first terminal- 
	if you want to publish radar data from an existing log file, run ros_pub_log_v2.py
		using: python ros_pub_log_v2.py
	if you want to publish live radar data (read log file as its being written), run dyn_ros_pub.py
		using: python dyn_ros_pub.py
		
5. in rviz2, 	
	A. add a display using the 'Add' button in the bottom left corner of the 'Displays' panel. In the prompt that opens after clicking on 'Add', scroll down in the list of 'rviz_default_plugins' to select 'PointCloud2', then press 'OK'.
	B. Now, expand 'PointCloud2' in the Displays section, in the 'Topic' field add the topic '/point_cloud' (select from drop-down menu or type manually).
	C. Then, in the same Displays section, at the top in the 'Global Options' tab, in the 'Fixed Frame' field, change the frame from 'map' to 'base_link' (type manually)
	D. (when you run one of the python scripts from point 4 of this ReadMe) the points would start appearing in RViz. To change how the points appear, in the tab of 'PointCloud2' you can change the 'Style' default from 'Flat Squares' to 'points', then change the 'Size (Pixels)' to 5 or 7 for better visualization.
	
	
	

Note: 
A. if you change anything in the mmwave_ws located at '/home/robot_3/mmwave_ws', make sure to colcon build inside mmwave_ws
	using: cd /home/robot_3/mmwave_ws
	       colcon build

B. in point 2, sourcing the bash file might not be necessary every time.

C. you can get the info or echo the topic '/point_cloud'
	using: ros2 topic info /point_cloud
	       ros2 topic echo /point_cloud
	       
D. if you make any changes to the ros2 package of mmwave_ws or mmwave_pkg, to test the working of the environment:
	1. in the (first) terminal, colcon build in the mmwave_ws (refer to point A in the Note)
	2. in the first terminal, perform 'source ~/mmwave_ws/install/setup.bash'
	3. run the subscriber (sample, to test package) for the '/point_cloud' topic
		using: ros2 run mmwave_pkg mmwave_sub
	4. in the second terminal, perform 'source ~/mmwave_ws/install/setup.bash'
	5. run the publisher (sample, to test package)
		using: ros2 run mmwave_pkg mmwave_pub 	
	6. If you received data in the first terminal (where subscriber is running), then the topic and the package are working.
	
E. Don't forget to save any changes you make to the python files before running them again.
	
