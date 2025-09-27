cmake_minimum_required(VERSION 3.0.2)
project(localization_interface)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  any_worker
  any_node
  tf2
  signal_handler
  ros_ecat_msgs
  tf2_ros

  # tf2_geometry_msgs
  geometry_msgs
  ops9_controller
)

catkin_package(
  INCLUDE_DIRS include

  # LIBRARIES localization_interface
  CATKIN_DEPENDS
  roscpp
  any_worker
  any_node
  tf2
  signal_handler
  ros_ecat_msgs
  tf2_ros

  # tf2_geometry_msgs
  geometry_msgs
  ops9_controller

  # DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

find_package(tf2_geometry_msgs REQUIRED)

# # Declare a C++ library
add_library(PubOdom
  src/PubOdom.cpp
)
add_dependencies(PubOdom
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(PubOdom
  ${catkin_LIBRARIES}
)

add_executable(pub_odom_node src/pub_odom_node.cpp)
add_dependencies(pub_odom_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_odom_node
  PubOdom
  ${catkin_LIBRARIES}
)

add_executable(test_pub_odom_node src/test_pub_odom_node.cpp)
add_dependencies(test_pub_odom_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_pub_odom_node
  ${catkin_LIBRARIES}
)
