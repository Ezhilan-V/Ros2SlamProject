cmake_minimum_required(VERSION 3.5.0)
project(velodyne_description)

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-dev)
endif()
cmake_policy(SET CMP0048 NEW)
ament_export_dependencies(gazebo_ros_pkgs)

ament_package()

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

install(DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(DIRECTORY world/
  DESTINATION share/${PROJECT_NAME}/world
)

install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)
