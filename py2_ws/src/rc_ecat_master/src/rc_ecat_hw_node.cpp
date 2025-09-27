/**
 * @file rc_ecat_hw_node.cpp
 * @author Keten (2863861004@qq.com)
 * @brief ros-control hw 主节点
 * @version 0.1
 * @date 2025-05-05
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "rc_ecat_master/RcEcatHardwareInterface.h"

#include <any_node/ThreadedPublisher.hpp>
#include <any_worker/Worker.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "rc_ecat_hw_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  rc_ecat_master::RcEcatHW rc_ecat_hw;
  if (!rc_ecat_hw.init(nh, nh_private)) {
    ROS_ERROR("Failed to initialize RcEcatHW");
    return -1;
  }

  ROS_INFO_STREAM("RC ECAT hardware interface initialized successfully");

  any_worker::Worker printfWorker(
      "printfWorker", 0.01, [&](const any_worker::WorkerEvent& event) -> bool {
        std::cout << rc_ecat_hw;
        return true;
      });

  // printfWorker.start(45);
  ros::spin();

  return 0;
}