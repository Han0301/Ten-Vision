/**
 * @file position_sensor_hw_node.cpp
 * @author Keten (2863861004@qq.com)
 * @brief position硬件传感器启动节点
 * @version 0.1
 * @date 2025-05-26
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include <position_sensor/PositionSensorHW.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "position_sensor_hw_node");
  ros::NodeHandle nh("position_sensor");
  position_sensor::PositionSensorHW position_sensor_hw;
  if (!position_sensor_hw.init(nh)) {
    ROS_ERROR_STREAM("position_sensor_hw init failed");
    return -1;
  }

  ros::spin();
  return 0;
}