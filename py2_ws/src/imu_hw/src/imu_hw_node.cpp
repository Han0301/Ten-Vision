/**
 * @file imu_hw_node.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-06
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include <imu_hw/ImuHW.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_hw");

  ros::NodeHandle nh;
  imu_hw::Imu_HW imu(nh);

  if (!imu.init()) {
    ROS_ERROR("faild to start the imu_hw!");
    return -1;
  }
  int32_t count = 0;
  ros::Time start = ros::Time::now();

  bool to_recover = false;
  while (ros::ok()) {
    // 如果需要恢复，不可进入读数据解包
    if (to_recover == true) {
      int recover_result = imu.recoverPort();  // 恢复流程
      ros::Duration(0.5).sleep();
      if (recover_result != -1) {
        // 恢复成功
        to_recover = false;
      }
    } else {
      int8_t if_open = imu.unpack();
      if (if_open == -1) {
        // ros::shutdown();
        to_recover = true;
      }

      if (if_open == 0) {
        count++;
      }
      imu.getYawData();
      ros::Time now = ros::Time::now();
      if ((now - start).toSec() >= 1.0) {
        ROS_INFO_STREAM_THROTTLE(0.1, "Approx. rate:" << count << "Hz");
        count = 0;
        start = now;
      }

      ros::spinOnce();
    }
  }

  return 0;
}