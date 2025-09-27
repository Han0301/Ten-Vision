/**
 * @file pbox_remote_node.h
 * @author py <997276894@qq.com>
 * @brief  
 * @version 0.1
 * @date  2025-06-08
 * 
 * @copyright Copyright (c)  2025 
 * 
 * @attention  
 * @note  
 */

#include <ros/ros.h>
#include <pbox_hw/pbox_remote.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "xbox_remote_node");
    ros::NodeHandle nh;
  
    pbox_hw::PboxRemote pbox_remote;
    if (pbox_remote.init(nh)) {
      ROS_INFO_STREAM("Xbox remote initialized");
    } else {
      ROS_ERROR_STREAM("Failed to initialize Xbox remote");
      return -1;
    }
    ros::spin();
    return 0;
  }