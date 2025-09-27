/**
 * @file RcEcatSlave.hpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-05-02
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include <iostream>

#include "soem_interface_rsl/EthercatBusBase.hpp"
#include "soem_interface_rsl/EthercatSlaveBase.hpp"

#define RX_PDO_ID 0x6000
#define TX_PDO_ID 0x7000

namespace rc_ecat_master {

struct TxPdo {
  bool inputIO[8];
  int16_t can0_motor_positions[8];
  int16_t can0_motor_velocities[8];
  int16_t can0_motor_currents[8];
  uint8_t can0_motor_temperatures[8];
  int16_t can1_motor_positions[8];
  int16_t can1_motor_velocities[8];
  int16_t can1_motor_currents[8];
  uint8_t can1_motor_temperatures[8];
  int32_t vesc_can0_motor_velocities[6];
  int32_t vesc_can1_motor_velocities[6];
  int32_t vesc_can0_motor_currents[6];
  int32_t vesc_can1_motor_currents[6];
} __attribute__((packed));

struct RxPdo {
  bool outputIO[8];
  int16_t can0_motor_commands[8];
  int16_t can1_motor_commands[8];
  int32_t vesc_can0_commands[6];
  int32_t vesc_can1_commands[6];
} __attribute__((packed));

class RcEcatSlave : public soem_interface_rsl::EthercatSlaveBase {
 public:
  enum class CanBus { CAN0 = 0, CAN1 = 1 };

  RcEcatSlave(const std::string& name, soem_interface_rsl::EthercatBusBase* bus,
              const uint32_t address);
  ~RcEcatSlave() override = default;

  std::string getName() const override { return name_; }

  bool startup() override;
  void updateRead() override;
  void updateWrite() override;
  void shutdown() override;

  PdoInfo getCurrentPdoInfo() const override { return pdoInfo_; }

  friend std::ostream& operator<<(std::ostream& os, const RcEcatSlave& tmp);

  const int64_t getActuatorPositionRaw(const CanBus& id, const int& index) {
    if (id == CanBus::CAN0)
      return reading_.can0_motor_positions[index];
    else
      return reading_.can1_motor_positions[index];
  }
  const int64_t getActuatorVelocityRaw(const CanBus& id, const int& index) {
    if (id == CanBus::CAN0)
      return reading_.can0_motor_velocities[index];
    else
      return reading_.can1_motor_velocities[index];
  }
  const int64_t getActuatorEffortRaw(const CanBus& id, const int& index) {
    if (id == CanBus::CAN0)
      return reading_.can0_motor_currents[index];
    else
      return reading_.can1_motor_currents[index];
  }
  const int64_t getActuatorTemperatureRaw(const CanBus& id, const int& index) {
    if (id == CanBus::CAN0)
      return reading_.can0_motor_temperatures[index];
    else
      return reading_.can1_motor_temperatures[index];
  }
  const int32_t getVescActuatorVelocityRaw(const CanBus& id, const int& index) {
    if (id == CanBus::CAN0)
      return reading_.vesc_can0_motor_velocities[index];
    else
      return reading_.vesc_can1_motor_velocities[index];
  }
  const int32_t getVescActuatorCurrentRaw(const CanBus& id, const int& index) {
    if (id == CanBus::CAN0)
      return reading_.vesc_can0_motor_currents[index];
    else
      return reading_.vesc_can1_motor_currents[index];
  }

  /* 直接将两条can总线上的数据拷贝过来 */
  void setActuatorCommands(
      const std::unordered_map<CanBus, std::unordered_map<int, int16_t>>&
          actuatorCommands) {
    for (const auto& it : actuatorCommands) {
      const CanBus& canbus = it.first;
      const auto& commands = it.second;
      if (canbus == CanBus::CAN0) {
        for (const auto& itit : commands) {
          const int& index = itit.first;
          const int16_t& command = itit.second;
          command_.can0_motor_commands[index] = command;
        }
      } else if (canbus == CanBus::CAN1) {
        for (const auto& itit : commands) {
          const int& index = itit.first;
          const int16_t& command = itit.second;
          command_.can1_motor_commands[index] = command;
        }
      }
    }
  }

  void setVescActuatorCommands(
      const std::unordered_map<CanBus, std::unordered_map<int, int32_t>>&
          actuatorCommands) {
    for (const auto& it : actuatorCommands) {
      const CanBus& canbus = it.first;
      const auto& commands = it.second;
      if (canbus == CanBus::CAN0) {
        for (const auto& itit : commands) {
          const int& index = itit.first;
          const int32_t& command = itit.second;
          if (index >= 0 && index < 6) {
            command_.vesc_can0_commands[index] = command;
          } else {
            ROS_WARN("VESC CAN0 index %d out of range", index);
          }
        }
      } else if (canbus == CanBus::CAN1) {
        for (const auto& itit : commands) {
          const int& index = itit.first;
          const int32_t& command = itit.second;
          if (index >= 0 && index < 6) {
            command_.vesc_can1_commands[index] = command;
          } else {
            ROS_WARN("VESC CAN1 index %d out of range", index);
          }
        }
      }
    }
  }

  std::unordered_map<int32_t, bool> getInputGpioData(void) const {
    std::unordered_map<int32_t, bool> gpio_data;
    for (int i = 0; i < sizeof(reading_.inputIO) / sizeof(reading_.inputIO[0]);
         ++i) {
      gpio_data[i] = reading_.inputIO[i];
    }
    return gpio_data;
  }

  std::unordered_map<int32_t, bool> getOutputGpioData(void) const {
    std::unordered_map<int32_t, bool> gpio_data;
    for (int i = 0;
         i < sizeof(command_.outputIO) / sizeof(command_.outputIO[0]); ++i) {
      gpio_data[i] = command_.outputIO[i];
    }
    return gpio_data;
  }

  /**
   * @brief Set the Output Gpio Data object
   *        入参是一个map,指示你要写入的io
   *
   * @param gpio_data
   */
  void setOutputGpioData(const std::unordered_map<int32_t, bool> cmds) {
    for (const auto& cmd : cmds) {
      command_.outputIO[cmd.first] = cmd.second;
    }
  }

 private:
  const std::string name_;
  PdoInfo pdoInfo_;
  TxPdo reading_;
  RxPdo command_;
};

}  // namespace rc_ecat_master