/**
 * @file DJIRosEcatSlave.hpp
 * @author Keten (2863861004@qq.com)
 * @brief DJI 固件控制板 特化
 * @version 0.1
 * @date 2025-05-03
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once
/* ros-lib */
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
/* third party */
#include <any_node/ThreadedPublisher.hpp>
#include <soem_interface_rsl/EthercatSlaveBase.hpp>
/* c++-lib */
#include <iostream>
#include <memory>
#include <mutex>
/* self-lib */
#include <ros_ecat_msgs/DJIEcatRosMsg.h>

#include <rc_ecat_master/RcEcatSlave.hpp>

namespace ros_ecat_slave {

template <>
class RosEcatSlave<RcEcatSlave::RcEcatSlave> {
 public:
  RosEcatSlave(const std::string& name,
               soem_interface_rsl::EthercatBusBase* bus,
               const uint32_t address);
  ~RosEcatSlave() = default;

  void updateRosState();

  void updateRosCommand();

  ros_ecat_msgs::DJIEcatRosMsg getEcatReadingsMsg() {
    std::lock_guard<std::recursive_mutex> lock(ecatreadingsmsgsMutex_);
    return ecat_readings_msgs_;
  }

  sensor_msgs::JointState getJointStateMsg() {
    std::lock_guard<std::recursive_mutex> lock(jointstatemsgsMutex_);
    return joint_state_msgs_;
  }

 private:
  std::string name_;
  SlaveType slave_type_;
  std::shared_ptr<RcEcatSlave::RcEcatSlave> slave_ptr_;
  /* ros-relative */
  ros::NodeHandle nh_;
  any_node::ThreadedPublisherPtr<sensor_msgs::JointState>
      joint_state_publisher_;
  any_node::ThreadedPublisherPtr<ros_ecat_msgs::DJIEcatRosMsg>
      ecat_readings_publisher_;

  sensor_msgs::JointState joint_state_msgs_;
  std::recursive_mutex jointstatemsgsMutex_;
  ros_ecat_msgs::DJIEcatRosMsg ecat_readings_msgs_;
  std::recursive_mutex ecatreadingsmsgsMutex_;
};

template <>
RosEcatSlave<RcEcatSlave::RcEcatSlave>::RosEcatSlave(
    const std::string& name, soem_interface_rsl::EthercatBusBase* bus,
    const uint32_t address)
    : slave_ptr_(
          std::make_shared<RcEcatSlave::RcEcatSlave>(name, bus, address)),
      slave_type_(SlaveType::DJI_MOTOR_BOARD),
      name_(name) {
  /* ros-relative */
  joint_state_publisher =
      std::make_shared<any_node::ThreadedPublisher<sensor_msgs::JointState>>(
          nh.advertise<sensor_msgs::JointState>(name_ + "JointState", 10), 50,
          false);
  ecat_readings_publisher_ = std::make_shared<
      any_node::ThreadedPublisher<ros_ecat_msgs::DJIEcatRosMsg>>(
      nh.advertise<ros_ecat_msgs::DJIEcatRosMsg>(name_ + "EcatReadings", 10),
      50, false);
}

/* 更新状态，就是从从站读 */
template <>
void RosEcatSlave<RcEcatSlave::RcEcatSlave>::updateRosState() {}

/* 更新命令，就是往从站写 */
template <>
void RosEcatSlave<RcEcatSlave::RcEcatSlave>::updateRosCommand() {}
}  // namespace ros_ecat_slave
