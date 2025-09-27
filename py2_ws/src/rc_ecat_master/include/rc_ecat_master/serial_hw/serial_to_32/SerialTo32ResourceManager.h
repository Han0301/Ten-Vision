/**
 * @file SerialTo32ResourceManager.h
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-05-17
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include <rc_ecat_master/common/hardware_interface/SerialTo32Interface.h>
#include <ros/ros.h>
#include <serial_manager/SerialManager.h>

#include <any_node/ThreadedPublisher.hpp>

namespace serial_hw {

class SerialTo32ResourceManager {
 public:
  SerialTo32ResourceManager() = default;
  ~SerialTo32ResourceManager() = default;
  explicit SerialTo32ResourceManager(ros::NodeHandle& nh)
      : serial_member_(nh) {}

  bool init();

  void read();

  void write();

  void debugWrite(float data[], uint8_t data2[]) {
    for (int i = 0; i < 6; i++) {
      send_data_.float_data[i] = data[i];
    }
    for (int i = 0; i < 4; i++) {
      send_data_.uint8_data[i] = data2[i];
    }
  }

  void bindToHandle(common::SerialTo32Handle& handle) {
    // 绑定数据到句柄
    handle = common::SerialTo32Handle(
        name_, rev_data_.float_data, rev_data_.uint8_data,
        send_data_.float_data, send_data_.uint8_data);
  }

  void setName(const std::string& name) { name_ = name; }

  // 获取当前最新接收的数据
  common::RevData getRevData() const { return rev_data_; }

  // 获取当前最新写入的数据
  common::SendData getSendData() const { return send_data_; }

 private:
  std::string name_;
  /* serial relative */
  serial_manager::SerialMember serial_member_;
  common::RevData rev_data_{};    // 接收数据结构体
  common::SendData send_data_{};  // 发送数据结构体
  /* pub relative */
};

}  // namespace serial_hw