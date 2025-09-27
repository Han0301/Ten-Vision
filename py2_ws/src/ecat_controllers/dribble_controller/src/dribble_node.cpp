/**
 * @file dribble_node.cpp
 * @author py <997276894@qq.com>
 * @brief  运球节点
 * @version 0.1
 * @date  2025-07-23 
 * 
 * @copyright Copyright (c)  2025 
 * 
 * @attention  
 * @note  
 */

#include <ros/ros.h>
#include "dribble_controller/dribble_control.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "dribble_node");
  ros::NodeHandle nh;

  dribble_control::DribbleControl Dribble;
  if (Dribble.init(nh)) {
    ROS_INFO_STREAM("Xbox remote initialized");
  } else {
    ROS_ERROR_STREAM("Failed to initialize Xbox remote");
    return -1;
  }

  ros::spin();

  return 0;
}