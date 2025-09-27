cmake_minimum_required(VERSION 3.0.2)
project(chassis_controller)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  hardware_interface
  pluginlib
  roslint
  geometry_msgs
  control_msgs
  nav_msgs
  control_toolbox
  any_node
  effort_controllers
  ctrl_common
  std_msgs
  rc_ecat_master
  ros_ecat_msgs
  vision_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(

  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}

  # LIBRARIES chassis_controller
  CATKIN_DEPENDS
  roscpp
  controller_interface
  hardware_interface
  pluginlib
  roslint
  geometry_msgs
  control_msgs
  control_toolbox
  any_node
  effort_controllers

  # DEPENDS system_lib
)

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(

  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# # Declare a C++ library
add_library(OmniChassisController
  src/OmniChassisController.cpp
)
target_link_libraries(OmniChassisController
  ${catkin_LIBRARIES}
)
install(FILES omni_chassis_controller_plugins.xml
  LIBRARY DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS OmniChassisController
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

add_library(SwerveChassisController
  src/SwerveChassisController.cpp
)
target_link_libraries(SwerveChassisController
  ${catkin_LIBRARIES}
)
install(FILES swerve_chassis_controller_plugins.xml
  LIBRARY DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS SwerveChassisController
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)


add_library(wujiaController
  src/wujiaController.cpp
)
target_link_libraries(wujiaController
  ${catkin_LIBRARIES}
)
install(TARGETS wujiaController
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES wujia_plugins.xml
  LIBRARY DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



