cmake_minimum_required(VERSION 3.0.2)
project(node_monitor)

add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosmon_msgs
)

catkin_package(

  INCLUDE_DIRS include

  # LIBRARIES node_monitor
  CATKIN_DEPENDS
  roscpp
  rosmon_msgs

  # DEPENDS system_lib
)

# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(

  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(monitor_node src/monitor_node.cpp)
add_dependencies(monitor_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(monitor_node
  ${catkin_LIBRARIES}
)
