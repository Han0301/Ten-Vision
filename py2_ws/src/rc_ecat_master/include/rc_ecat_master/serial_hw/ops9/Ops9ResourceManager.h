/**
 * @file Ops9ResourceManager.h
 * @author Keten (2863861004@qq.com) adapted from myx
 * @brief Ops9 全场定位
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once
#include <fcntl.h>
#include <rc_ecat_master/common/hardware_interface/Ops9Interface.h>
#include <ros/ros.h>
#include <ros_ecat_msgs/ActionData.h>
#include <serial/serial.h>
#include <termios.h>
#include <unistd.h>

#include <any_node/ThreadedPublisher.hpp>
#include <memory>

template <typename T>
T angToRad(T ang) {
  return ang * M_PI / 180.0;
}

template <typename T>
T radToAng(T rad) {
  return rad * 180.0 / M_PI;
}
namespace serial_hw {

union F3DataTransCh4 {
  float f_data[3];
  char ch_data[12];
};
union F6DataTransCh24 {
  char ch_data[24];
  float f_data4[6];
};
enum class SerialFlamMark { START, FIRST_HEAD, SECOND_HEAD, DATA, FIRST_TAIL };

struct serial_settings {
  int serial_fd;
  std::string serial_port;
  F3DataTransCh4 cmd_trans;
  F6DataTransCh24 data_trans;
  ros::Time last_write_time;
  SerialFlamMark serial_flam_mark;
  u_int serial_data_pose;
};

class Ops9ResourceManager {
 public:
  Ops9ResourceManager() = default;
  ~Ops9ResourceManager() = default;

  bool addOps9(const std::string& name, serial_settings serial_setting,
               const common::OpsRevData& ops_rev_data,
               const common::OpsCmdData& ops_cmd_data);
  bool initOps9(ros::NodeHandle& root_nh);
  bool readOps9(const ros::Time& time, const ros::Duration& period);
  bool writeOps9(const ros::Time& time, const ros::Duration& period);

  void bindToHandle(std::vector<common::Ops9Handle>& ops9_handles);

  std::unordered_map<std::string, serial_settings> getOps9SerialMap() {
    return ops9_serial_map_;
  }
  std::unordered_map<std::string, common::OpsRevData> getOps9RevDataMap() {
    return ops9_rev_data_map_;
  }
  std::unordered_map<std::string, common::OpsCmdData> getOps9CmdDataMap() {
    return ops9_cmd_data_map_;
  }

 private:
  void unPack(const std::string& name, const uint8_t data);
  bool writeCmd(const std::string& name, const std::string cmd);
  bool writeCmd(const std::string& name, const std::string cmd,
                const float data);
  bool writeCmd(const std::string& name, const std::string cmd,
                const std::vector<float>& dataList);

  std::unordered_map<std::string, serial_settings> ops9_serial_map_;
  std::unordered_map<std::string, common::OpsRevData> ops9_rev_data_map_;
  std::unordered_map<std::string, common::OpsCmdData> ops9_cmd_data_map_;

  /* pub the ops data */
};

}  // namespace serial_hw
