/**
 * @file RcEcatSlave.cpp
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
#include "rc_ecat_master/RcEcatSlave.hpp"

namespace rc_ecat_master {

RcEcatSlave::RcEcatSlave(const std::string& name,
                         soem_interface_rsl::EthercatBusBase* bus,
                         const uint32_t address)
    : name_(name), soem_interface_rsl::EthercatSlaveBase(bus, address) {
  pdoInfo_.rxPdoId_ = RX_PDO_ID;
  pdoInfo_.txPdoId_ = TX_PDO_ID;
  pdoInfo_.rxPdoSize_ = sizeof(command_);
  pdoInfo_.txPdoSize_ = sizeof(reading_);
  pdoInfo_.moduleId_ = 0x00123456;
}

bool RcEcatSlave::startup() { return true; }

void RcEcatSlave::updateRead() { bus_->readTxPdo(address_, reading_); }

void RcEcatSlave::updateWrite() { bus_->writeRxPdo(address_, command_); }

void RcEcatSlave::shutdown() {
  // Do nothing
}

std::ostream& operator<<(std::ostream& os, const RcEcatSlave& tmp) {
  std::stringstream ss;
  // 将光标向上移动 9 行（依据实际行数调整）
  ss << "\033[9F";
  // 清除所有行并输出新的数据
  ss << "\033[K"
     << "inputIO: ";
  for (int i = 0; i < 8; ++i) {
    ss << tmp.reading_.inputIO[i] << " ";
  }
  ss << "\n";
  ss << "\033[K"
     << "can0_motor_positions: ";
  for (int i = 0; i < 8; ++i) {
    ss << tmp.reading_.can0_motor_positions[i] << " ";
  }
  ss << "\n";
  ss << "\033[K"
     << "can0_motor_velocities: ";
  for (int i = 0; i < 8; ++i) {
    ss << tmp.reading_.can0_motor_velocities[i] << " ";
  }
  ss << "\n";
  ss << "\033[K"
     << "can0_motor_currents: ";
  for (int i = 0; i < 8; ++i) {
    ss << tmp.reading_.can0_motor_currents[i] << " ";
  }
  ss << "\n";
  ss << "\033[K"
     << "can0_motor_temperatures: ";
  for (int i = 0; i < 8; ++i) {
    ss << static_cast<int>(tmp.reading_.can0_motor_temperatures[i]) << " ";
  }
  ss << "\n";
  ss << "\033[K"
     << "can1_motor_positions: ";
  for (int i = 0; i < 8; ++i) {
    ss << tmp.reading_.can1_motor_positions[i] << " ";
  }
  ss << "\n";
  ss << "\033[K"
     << "can1_motor_velocities: ";
  for (int i = 0; i < 8; ++i) {
    ss << tmp.reading_.can1_motor_velocities[i] << " ";
  }
  ss << "\n";
  ss << "\033[K"
     << "can1_motor_currents: ";
  for (int i = 0; i < 8; ++i) {
    ss << tmp.reading_.can1_motor_currents[i] << " ";
  }
  ss << "\n";
  ss << "\033[K"
     << "can1_motor_temperatures: ";
  for (int i = 0; i < 8; ++i) {
    ss << static_cast<int>(tmp.reading_.can1_motor_temperatures[i]) << " ";
  }
  ss << "\n";
  // 下面添加打印 command_ 部分
  ss << "\033[K"
     << "can0_motor_commands: ";
  for (int i = 0; i < 8; ++i) {
    ss << tmp.command_.can0_motor_commands[i] << " ";
  }
  ss << "\n";
  ss << "\033[K"
     << "can1_motor_commands: ";
  for (int i = 0; i < 8; ++i) {
    ss << tmp.command_.can1_motor_commands[i] << " ";
  }
  ss << "\n";
  ss << "\033[K"
     << "vesc_can0_motor_velocities: ";
  for (int i = 0; i < 6; ++i) {
    ss << tmp.reading_.vesc_can0_motor_velocities[i] << " ";
  }
  ss << "\n";
  ss << "\033[K"
     << "vesc_can1_motor_velocities: ";
  for (int i = 0; i < 6; ++i) {
    ss << tmp.reading_.vesc_can1_motor_velocities[i] << " ";
  }
  ss << "\n";
  ss << "\033[K"
     << "vesc_can1_motor_currents: ";
  for (int i = 0; i < 6; ++i) {
    ss << tmp.reading_.vesc_can1_motor_currents[i] << " ";
  }
  ss << "\n";
  ss << "\033[K"
     << "vesc_can0_motor_currents: ";
  for (int i = 0; i < 6; ++i) {
    ss << tmp.reading_.vesc_can0_motor_currents[i] << " ";
  }
  ss << "\n";

  os << ss.str();
  return os;
}

}  // namespace rc_ecat_master