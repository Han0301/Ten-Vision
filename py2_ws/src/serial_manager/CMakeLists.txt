cmake_minimum_required(VERSION 3.0.2)
project(serial_manager)

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
  serial
  signal_handler
)

catkin_package(

  INCLUDE_DIRS include

  LIBRARIES SerialManager
  CATKIN_DEPENDS
  any_node
  any_worker
  roscpp
  serial
  signal_handler

  # DEPENDS system_lib
)

include_directories(

  include
  ${catkin_INCLUDE_DIRS}
)

# # Declare a C++ library
add_library(SerialManager
  src/SerialManager.cpp
)
target_link_libraries(SerialManager
  ${catkin_LIBRARIES}
)

add_executable(serial_test_node src/serial_test_node.cpp)
add_dependencies(serial_test_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(serial_test_node
  ${catkin_LIBRARIES}
  SerialManager
)
