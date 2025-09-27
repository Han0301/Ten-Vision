/**
 * @file RosEcatHw.hpp
 * @author Keten (2863861004@qq.com)
 * @brief
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
/* ros lib */
#include <ros/ros.h>
/* third partys */
#include <any_node/ThreadedPublisher.hpp>
#include <soem_interface_rsl/EthercatSlaveBase.hpp>
/* c++ lib */
#include <iostream>
#include <memory>

namespace ros_ecat_slave {

/* 从站固件类型 */
enum class SlaveType {
  DJI_MOTOR_BOARD,
  VESC_MOTOR_BOARD,
};

/* 这里的SlaveBase只能是继承于soem_interface_rsl::EthercatSlaveBase的 */
template <typename SlaveBase,
          typename dummy = std::enable_if_t<std::is_base_of_v<
              soem_interface_rsl::EthercatSlaveBase, SlaveBase>>>
class RosEcatSlave {
 public:
  RosEcatSlave(const std::string& name,
               soem_interface_rsl::EthercatBusBase* bus,
               const uint32_t address);
  ~RosEcatSlave() = default;

  void updateRosState();

  void updateRosCommand();

  SlaveType getSlaveType() const { return slave_type_; }

 private:
  std::string name_;
  SlaveType slave_type_;
  std::shared_ptr<SlaveBase> slave_ptr_;

  /* ros-relative */
  ros::NodeHandle& nh_;
};

}  // namespace ros_ecat_slave

#include "ros_ecat_slave/DJIRosEcatSlave.hpp"