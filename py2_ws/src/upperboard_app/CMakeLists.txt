cmake_minimum_required(VERSION 3.0.2)
project(upperboard_app)

# # Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# # Find catkin macros and libraries
# # if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
# # is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  any_worker
  roscpp
  serial_manager
  signal_handler
  ros_ecat_msgs
)

catkin_package(

  INCLUDE_DIRS include

  # LIBRARIES upperboard_app
  CATKIN_DEPENDS
  any_worker
  roscpp
  serial_manager
  signal_handler
  ros_ecat_msgs

  # DEPENDS system_lib
)

include_directories(

  include
  ${catkin_INCLUDE_DIRS}
)

# # Declare a C++ library
# add_library(${PROJECT_NAME}
# src/${PROJECT_NAME}/upperboard_app.cpp
# )

# # Add cmake target dependencies of the library
# # as an example, code may need to be generated before libraries
# # either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# # Declare a C++ executable
# # With catkin_make all packages are built within a single CMake context
# # The recommended prefix ensures that target names across packages don't collide
add_executable(upperboard_app_node src/upperboard_app_node.cpp)
add_dependencies(upperboard_app_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(upperboard_app_node
  ${catkin_LIBRARIES}
)
