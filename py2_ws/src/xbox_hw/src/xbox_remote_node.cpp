/**
 * @file xbox_remote_node.cpp
 * @author Keten (2863861004@qq.com)
 * @brief xbox接收节点
 * @version 0.1
 * @date 2025-05-07
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include <ros/ros.h>
#include <xbox_hw/xbox_remote.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "xbox_remote_node");
  ros::NodeHandle nh;

  xbox_hw::XboxRemote xbox_remote;
  if (xbox_remote.init(nh)) {
    ROS_INFO_STREAM("Xbox remote initialized");
  } else {
    ROS_ERROR_STREAM("Failed to initialize Xbox remote");
    return -1;
  }

  ros::spin();

  return 0;
}