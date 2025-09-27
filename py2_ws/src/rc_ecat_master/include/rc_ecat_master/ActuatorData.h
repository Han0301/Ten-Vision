/**
 * @file ActuatorData.h
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once
#include <ros/time.h>

#include <rc_ecat_master/RcEcatSlave.hpp>

namespace rc_ecat_master {

struct ActuatorCoefficients {
  bool if_vesc;  // 是否使用VESC驱动器，是的话将采用其他方式进行换算
  double act2pos, act2vel, act2effort, effort2act, max_out;
};

struct ActuatorData {
  std::string name;
  std::string type;
  int8_t id;
  rc_ecat_master::RcEcatSlave::CanBus canbus;
  double reducer;
  ros::Time stamp;
  int64_t temp;
  double pos, vel, effort;               // 执行器关节数据
  double act_pos, act_vel, act_effort;   // 执行器原始数据
  int64_t pos_raw, vel_raw, effort_raw;  // 电调返回数据
  int64_t last_pos_raw;
  int64_t actualCircle;
  bool firstReceived{true};

  // commands
  double eff_cmd, vel_cmd, pos_cmd;
  double eff_exe_cmd, vel_exe_cmd, pos_exe_cmd;
  double act_eff_cmd, act_vel_cmd, act_pos_cmd;
  int16_t actual_cmd;     // 下发到can的指令
  int32_t actual_cmd_32;  // 下发到VESC的指令
  bool halted;
  bool if_vesc;
};

}  // namespace rc_ecat_master