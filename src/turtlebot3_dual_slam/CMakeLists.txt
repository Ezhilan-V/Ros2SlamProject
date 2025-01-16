cmake_minimum_required(VERSION 3.8)
project(turtlebot3_dual_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(tf2_ros REQUIRED)

# Install directories and files
install(
  FILES
    urdf/turtlebot3_dual_lidar_standalone.urdf.xacro
  DESTINATION
    share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY
    launch
    rviz
  DESTINATION
    share/${PROJECT_NAME}
)

# Make launch files executable
install(
  PROGRAMS
    launch/dual_slam.launch.py
  DESTINATION
    share/${PROJECT_NAME}/launch
)

ament_package()