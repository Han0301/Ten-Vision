cmake_minimum_required(VERSION 3.0.2)
project(position_sensor)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  any_node
  any_worker
  roscpp
  serial_manager
  signal_handler
  ros_ecat_msgs
)

catkin_package(

  INCLUDE_DIRS include

  # LIBRARIES position_sensor
  CATKIN_DEPENDS
  any_node
  any_worker
  roscpp
  serial_manager
  signal_handler
  ros_ecat_msgs

  # DEPENDS system_lib
)

# ##########
# # Build ##
# ##########

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(

  include
  ${catkin_INCLUDE_DIRS}
)

# # Declare a C++ library
add_library(position_sensor_hw
  src/PositionSensorHW.cpp
)
add_dependencies(position_sensor_hw
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(position_sensor_hw
  ${catkin_LIBRARIES}
)

add_executable(position_sensor_hw_node src/position_sensor_hw_node.cpp)
add_dependencies(position_sensor_hw_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(position_sensor_hw_node
  ${catkin_LIBRARIES}
  position_sensor_hw
)
