/**
 * @file ImuHW.h
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
#pragma once
#include <fcntl.h>
#include <ros/ros.h>
#include <ros_ecat_msgs/PositionSensorCmd.h>
#include <ros_ecat_msgs/PositionSensorMsg.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <any_node/ThreadedPublisher.hpp>
#include <any_worker/Worker.hpp>
#include <std_msgs/Float32.h>
#include <memory>
#include <mutex>

namespace imu_hw {
class Imu_HW {
 public:
  Imu_HW(ros::NodeHandle& nh);
  ~Imu_HW();

  bool init();

  bool pubWorkerCb(const any_worker::WorkerEvent& event);

  int8_t unpack();

  int8_t if_open_;

  int8_t recoverPort(void);

  void getYawData();

  void ImuCmdCallback(const ros_ecat_msgs::PositionSensorCmd::ConstPtr& msg);

 private:
  /* ros relative */
  ros::NodeHandle nh_;
  ros::Subscriber cmd_sub_;

  std::string serial_port_;
  int serial_fd_;
  int baudrate_;

  // worker
  std::shared_ptr<any_worker::Worker> pubWorker_;
  double pub_worker_cycle_;

  // rev data
  unsigned char buffer[16];
  short ax[6] = {0};
  short gx[6] = {0};
  short sMag[6] = {0};
  short sAngle[6] = {0};

  // pub data
  ros_ecat_msgs::PositionSensorMsg yaw_msg_;
  any_node::ThreadedPublisherPtr<ros_ecat_msgs::PositionSensorMsg>
      yaw_msg_publisher_;
  std_msgs::Float32 raw_msg_;
  any_node::ThreadedPublisherPtr<std_msgs::Float32> raw_yaw_publisher_;

  // diff final data
  float last_data_;
  float cur_data_;
  float delta_data_;
  float final_data_;
  bool if_first_get_{false};

  // mutex lock
  std::mutex yaw_data_mutex_;
};

}  // namespace imu_hw
