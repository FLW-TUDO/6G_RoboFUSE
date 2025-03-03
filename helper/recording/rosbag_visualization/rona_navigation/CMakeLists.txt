cmake_minimum_required(VERSION 3.8)
project(rona_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)  # Added nav2_msgs as it is used for NavigateToPose
find_package(gazebo_msgs REQUIRED)

find_package(tf2_ros REQUIRED)
find_package(tf2_kdl REQUIRED)

# Include C++ include directory
include_directories(include)

# Install other directories (launch, config, etc.)
install(DIRECTORY 
  config
  launch
  maps
  scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executable scripts
install(PROGRAMS
  src/follower_node.py  # Path to your Python node script
  scripts/timestamp_publisher.py #Path to timestamp publisher
  launch/rviz/rosbag_rviz_launch.py
  DESTINATION lib/${PROJECT_NAME}  # Install into the package lib folder
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
