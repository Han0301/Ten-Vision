cmake_minimum_required(VERSION 3.0.2)
project(ops9_controller)

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
  control_msgs
  control_toolbox
  controller_interface
  geometry_msgs
  hardware_interface
  pluginlib
  roscpp
  roslint
  rc_ecat_master
)

# # System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
catkin_package(

  INCLUDE_DIRS include

  # LIBRARIES ops9_controller
  CATKIN_DEPENDS
  any_node
  any_worker
  control_msgs
  control_toolbox
  controller_interface
  geometry_msgs
  hardware_interface
  pluginlib
  roscpp
  roslint
  rc_ecat_master

  # DEPENDS system_lib
)

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(

  include
  ${catkin_INCLUDE_DIRS}
)

# # Declare a C++ library
add_library(Ops9Controller
  src/Ops9Controller.cpp
)
target_link_libraries(Ops9Controller
  ${catkin_LIBRARIES}
)
install(FILES ops9_controller_plugins.xml
  LIBRARY DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS Ops9Controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
