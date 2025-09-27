cmake_minimum_required(VERSION 3.0.2)
project(ros_ecat_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

# # Generate messages in the 'msg' folder
add_message_files(
  FILES
  DJIEcatRosMsg.msg
  DJIEcatRosCommands.msg
  ActionData.msg
  Ops9Command.msg
  UpperBoardCmd.msg
  PositionSensorMsg.msg
  PositionSensorCmd.msg
  VescEcatRosMsg.msg
  VescEcatRosCommands.msg
  RemoteState.msg
  PboxMsg.msg
  InitState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(

  # INCLUDE_DIRS include

  # LIBRARIES ros_ecat_msgs
  CATKIN_DEPENDS std_msgs message_runtime

  # DEPENDS system_lib
)