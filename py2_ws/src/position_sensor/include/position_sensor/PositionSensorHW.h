/**
 * @file PositionSensorHW.h
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-05-26
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note : RESET 命令重启
 *         UPDATE_XY 命令更新XY数据
 * @versioninfo :
 */
#pragma once

#include <ros/ros.h>
#include <ros_ecat_msgs/PositionSensorCmd.h>
#include <ros_ecat_msgs/PositionSensorMsg.h>
#include <serial_manager/SerialManager.h>

#include <any_node/ThreadedPublisher.hpp>
#include <any_worker/Worker.hpp>
#include <memory>

/*
  智能指针使用场景：管理生命周期非自己掌握的资源

*/

namespace position_sensor {

class PositionSensorHW {
 public:
  PositionSensorHW();
  ~PositionSensorHW() = default;

  bool init(ros::NodeHandle& nh);

  void sensorCmdCallback(const ros_ecat_msgs::PositionSensorCmd::ConstPtr& msg);

  bool updateWorkerCb(const any_worker::WorkerEvent& event);

 private:
  /* ros relative */
  ros::NodeHandle nh_;

  /* serial manager relative */
  std::shared_ptr<serial_manager::SerialMember> serial_member_;
  std::shared_ptr<float> rev_float_data_{};
  std::shared_ptr<float> send_float_data_{};
  std::shared_ptr<uint8_t> send_frame_id_{};     // 发送包中的帧id
  std::shared_ptr<uint8_t> send_data_length_{};  // 发送包中的数据长度

  /* publisher relative */
  any_node::ThreadedPublisherPtr<ros_ecat_msgs::PositionSensorMsg>
      sensordataPublisher_;

  /* subscriber relative */
  ros::Subscriber sensorcmdSubscriber_;  // 接收控制命令，对传感器写入

  /* any worker relative */
  std::shared_ptr<any_worker::Worker> updateWorker_;
};

}  // namespace position_sensor