cmake_minimum_required(VERSION 3.0.2)
project(rc_ecat_master)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# add_definitions(-DMELO_USE_COUT)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS

  any_node

  any_worker
  message_logger
  param_io
  roscpp
  signal_handler
  soem_interface_rsl
  soem_rsl
  rosconsole
  controller_manager
  ros_ecat_msgs
  geometry_msgs
  tf2
  tf2_ros
  serial_manager
)

catkin_package(
  INCLUDE_DIRS include

  # LIBRARIES rc_ecat_master
  CATKIN_DEPENDS

  any_node

  # any_worker
  message_logger
  param_io
  roscpp
  signal_handler
  soem_interface_rsl
  soem_rsl
  rosconsole
  controller_manager
  ros_ecat_msgs
  geometry_msgs
  tf2
  tf2_ros
  serial_manager

  # DEPENDS system_lib
)

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Add library Of rc_ecat_slave
add_library(rc_ecat_slave
  src/RcEcatSlave.cpp
  src/ecat_hardware_interface/gpio/GpioResourceManager.cpp
  src/serial_hw/ops9/Ops9ResourceManager.cpp
  src/serial_hw/serial_to_32/SerialTo32ResourceManager.cpp
)
add_dependencies(rc_ecat_slave
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rc_ecat_slave
  ${catkin_LIBRARIES}
)

# Add library Of rc_ecat_hw
find_package(yaml-cpp REQUIRED)
add_library(rc_ecat_hw
  src/RcEcatHardwareInterface.cpp
)
add_dependencies(rc_ecat_hw
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rc_ecat_hw
  ${catkin_LIBRARIES}
  yaml-cpp
  rc_ecat_slave
)

add_library(rt_joint_state
  src/rt_joint_state.cpp
)
add_dependencies(rt_joint_state
  ${catkin_EXPORTED_TARGETS}
)

add_executable(rc_ecat_node src/rc_ecat_node.cpp)
add_dependencies(rc_ecat_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rc_ecat_node
  ${catkin_LIBRARIES}
  rc_ecat_slave
)

add_executable(rc_ecat_hw_node src/rc_ecat_hw_node.cpp)
add_dependencies(rc_ecat_hw_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rc_ecat_hw_node
  ${catkin_LIBRARIES}
  rc_ecat_hw
)
