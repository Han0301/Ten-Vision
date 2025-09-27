/**
 * @file pub_odom_node.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-05-09
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include <localization_interface/PubOdom.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "pub_odom_node");
  ros::NodeHandle nh;
  localization_interface::OdomPublisher odom_publisher(nh);

  ros::spin();

  return 0;
}