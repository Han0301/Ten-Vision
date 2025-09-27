cmake_minimum_required(VERSION 3.0.2)
project(imu_hw)

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
  ros_ecat_msgs
  roscpp
  serial_manager
  signal_handler
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include

  # LIBRARIES imu_hw
  CATKIN_DEPENDS
  any_node
  any_worker
  ros_ecat_msgs
  roscpp
  serial_manager
  signal_handler
  std_msgs
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
# # Declare a C++ library
add_library(imu_hw
  src/ImuHW.cpp
)
add_dependencies(imu_hw
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(imu_hw
  ${catkin_LIBRARIES}
)

# # Add cmake target dependencies of the library
# # as an example, code may need to be generated before libraries
# # either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Declare a C++ executable
# # With catkin_make all packages are built within a single CMake context
# # The recommended prefix ensures that target names across packages don't collide
add_executable(imu_hw_node src/imu_hw_node.cpp)
add_dependencies(imu_hw_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(imu_hw_node
  ${catkin_LIBRARIES}
  imu_hw
)
