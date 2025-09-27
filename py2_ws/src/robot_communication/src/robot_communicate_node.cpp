/**
 * @file robot_communicate_node.cpp
 * @author py <997276894@qq.com>
 * @brief  
 * @version 0.1
 * @date  2025-05-27 
 * 
 * @copyright Copyright (c)  2025 
 * 
 * @attention  
 * @note  
 */

#include "robot_communication/robot_communicate.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "robot_communicate_node");
  ros::NodeHandle nh;

  robot_communicate::RobotCommunicate robot_communicate;
  if (!robot_communicate.init(nh)) {
    ROS_ERROR("Failed to initialize RobotCommunicate");
    return -1;
  }

  ros::Rate loop_rate(10); // Adjust the rate as needed
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}