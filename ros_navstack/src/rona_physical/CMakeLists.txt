cmake_minimum_required(VERSION 3.5)
project(rona_physical)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)

find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

# Include Cpp "include" directory
include_directories(include)
include_directories(/opt/ros/humble/include/tf2_geometry_msgs)
include_directories(
  include
  ${nav2_costmap_2d_INCLUDE_DIRS}
)
# Add libraries
add_library(cmd_vel_to_ackermann_node SHARED src/cmd_vel_to_ackermann/cmd_vel_to_ackermann.cpp)
add_library(vicon_tf_converter_node SHARED src/vicon/vicon_tf_converter.cpp)
add_library(waypoint_publisher_node SHARED src/waypoint_publisher/waypoint_publisher.cpp)
add_library(vicon_obstacle_layer SHARED src/vicon/vicon_obstacle_layer.cpp)
add_library(vicon_obstacle_layerA SHARED src/vicon/vicon_obstacle_layerA.cpp)
add_library(vicon_obstacle_layerAS4 SHARED src/vicon/vicon_obstacle_layerAS4.cpp)
add_library(vicon_obstacle_layerAS5 SHARED src/vicon/vicon_obstacle_layerAS5.cpp)
add_library(vicon_obstacle_layerAS6 SHARED src/vicon/vicon_obstacle_layerAS6.cpp)
add_library(vicon_obstacle_layerRM SHARED src/vicon/vicon_obstacle_layerRM.cpp)
# Create Cpp executable
add_executable(cmd_vel_to_ackermann src/cmd_vel_to_ackermann/cmd_vel_to_ackermann_main.cpp)
add_executable(vicon_tf_converter src/vicon/vicon_tf_converter_main.cpp)
add_executable(waypoint_publisher src/waypoint_publisher/waypoint_publisher_main.cpp)


# Target dependencies
set(cmd_vel_to_ackermann_dependencies rclcpp std_msgs std_srvs geometry_msgs ackermann_msgs)
ament_target_dependencies(cmd_vel_to_ackermann_node ${cmd_vel_to_ackermann_dependencies})
ament_target_dependencies(cmd_vel_to_ackermann ${cmd_vel_to_ackermann_dependencies})
set(vicon_tf_converter_dependencies rclcpp geometry_msgs tf2_kdl tf2_kdl)
ament_target_dependencies(vicon_tf_converter_node ${vicon_tf_converter_dependencies})
ament_target_dependencies(vicon_tf_converter ${vicon_tf_converter_dependencies})
set(waypoint_publisher_dependencies rclcpp geometry_msgs nav2_msgs rclcpp_action rclcpp_components tf2_ros)
ament_target_dependencies(waypoint_publisher_node ${waypoint_publisher_dependencies})
ament_target_dependencies(waypoint_publisher ${waypoint_publisher_dependencies})

ament_target_dependencies(vicon_obstacle_layer rclcpp nav2_costmap_2d pluginlib)
ament_target_dependencies(vicon_obstacle_layerA rclcpp nav2_costmap_2d pluginlib)
ament_target_dependencies(vicon_obstacle_layerAS4 rclcpp nav2_costmap_2d pluginlib)
ament_target_dependencies(vicon_obstacle_layerAS5 rclcpp nav2_costmap_2d pluginlib)
ament_target_dependencies(vicon_obstacle_layerAS6 rclcpp nav2_costmap_2d pluginlib)
ament_target_dependencies(vicon_obstacle_layerRM rclcpp nav2_costmap_2d pluginlib)
pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)
# Link libraries
target_link_libraries(cmd_vel_to_ackermann cmd_vel_to_ackermann_node)
target_link_libraries(vicon_tf_converter vicon_tf_converter_node)
target_link_libraries(waypoint_publisher waypoint_publisher_node)


# Install Cpp executables
install(TARGETS
  cmd_vel_to_ackermann
  vicon_tf_converter
  waypoint_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
  vicon_obstacle_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  vicon_obstacle_layerA
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(TARGETS 
  vicon_obstacle_layerAS4
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS 
  vicon_obstacle_layerAS5
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS 
  vicon_obstacle_layerAS6
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  vicon_obstacle_layerRM
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS 
  cmd_vel_to_ackermann_node
  vicon_tf_converter_node
  waypoint_publisher_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)
# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/curve_fitting.py
  scripts/camera_pub.py
  scripts/costmap_generator.py
  scripts/clear_costmap_on_goal.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
