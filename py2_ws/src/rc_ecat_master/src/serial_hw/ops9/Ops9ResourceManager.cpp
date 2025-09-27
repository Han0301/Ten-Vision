/**
 * @file Ops9ResourceManager.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note : 如果要更新，顺序必须为 angle
 * 、position_x、position_y,三个变量必须为float型变量
 * @versioninfo :
 */
#include <rc_ecat_master/serial_hw/ops9/Ops9ResourceManager.h>

namespace serial_hw {

bool Ops9ResourceManager::addOps9(const std::string& name,
                                  serial_settings serial_setting,
                                  const common::OpsRevData& ops_rev_data,
                                  const common::OpsCmdData& ops_cmd_data) {
  // 先检查重复
  if (ops9_serial_map_.find(name) != ops9_serial_map_.end() ||
      ops9_rev_data_map_.find(name) != ops9_rev_data_map_.end() ||
      ops9_cmd_data_map_.find(name) != ops9_cmd_data_map_.end()) {
    ROS_ERROR_STREAM("add a duplicate ops9 name: " << name);
    return false;
  }
  // 插入新元素
  ops9_serial_map_[name] = serial_setting;
  ops9_rev_data_map_[name] = ops_rev_data;
  ops9_cmd_data_map_[name] = ops_cmd_data;
  return true;
}

bool Ops9ResourceManager::initOps9(ros::NodeHandle& root_nh) {
  for (auto& it : ops9_serial_map_) {
    std::string name = it.first;
    std::string serial_port = it.second.serial_port;
    char* port = (char*)serial_port.data();
    it.second.serial_fd = open(port, O_RDWR | O_NOCTTY);
    if (it.second.serial_fd == -1) {
      ROS_ERROR_STREAM(
          "Unable to open serial of ops9. Serial port:" << serial_port);
      return false;
    }
    struct termios options {};

    if (tcgetattr(it.second.serial_fd, &options) != 0) {
      perror("SetupSerial 1");
    }
    memset(&options, 0, sizeof(options));
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~CSIZE;

    options.c_cflag |= CS8;

    options.c_cflag &= ~PARENB;

    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag &= ~CSTOPB;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 0;
    tcflush(it.second.serial_fd, TCIFLUSH);
    if ((tcsetattr(it.second.serial_fd, TCSANOW, &options)) != 0) {
      perror("com set error");
    }
    ROS_INFO_STREAM("Successful to open " << serial_port << " port.");
    it.second.last_write_time = ros::Time::now();
    for (int i = 0; i < 6; i++) {
      it.second.data_trans.f_data4[i] = 0.0;
    }
    for (int i = 0; i < 3; i++) {
      it.second.cmd_trans.f_data[i] = 0.0;
    }
    it.second.serial_flam_mark = serial_hw::SerialFlamMark::START;
  }
  return true;
}

bool Ops9ResourceManager::readOps9(const ros::Time& time,
                                   const ros::Duration& period) {
  const size_t bufSize = 28;
  uint8_t buffer[bufSize];

  for (auto& pair : ops9_serial_map_) {
    const std::string& name = pair.first;
    serial_settings& settings = pair.second;
    auto it = ops9_cmd_data_map_.find(name);
    if (it == ops9_cmd_data_map_.end()) {
      ROS_ERROR_STREAM("Device " << name << " not found in command data map.");
      return false;
    }
    auto& cmd_data = it->second;

    if (settings.serial_fd == -1) {
      ROS_ERROR_STREAM("Serial port " << name << " is not open.");
      return false;
    }

    if (!cmd_data.calibration_state) {
      // 一次读取最多 bufSize 字节，减少系统调用次数
      ssize_t n = read(settings.serial_fd, buffer, bufSize);
      if (n > 0) {
        for (ssize_t i = 0; i < n; ++i) {
          unPack(name, buffer[i]);
        }
      }
    }
  }
  return true;
}

/**
 * 外部如何设置action的校准命令，通过设置cmd进行修改，比如将某个要进行的动作置为true
 * 在这里检测到，就会发送，发送完并将其置回0
 */
bool Ops9ResourceManager::writeOps9(const ros::Time& time,
                                    const ros::Duration& period) {
  for (auto& it : ops9_serial_map_) {
    const std::string& name = it.first;
    serial_hw::serial_settings& settings = it.second;  // 获取serial相关
    auto itCmd = ops9_cmd_data_map_.find(name);        // 获取cmd相关
    if (itCmd == ops9_cmd_data_map_.end()) {
      ROS_ERROR_STREAM("Device " << name << " not found in resource maps.");
      return false;
    }
    auto& cmd_data = itCmd->second;
    if (cmd_data.reset_state) {
      // 发送校准命令 "ACT0"
      writeCmd(name, "ACT0");
      cmd_data.reset_state = false;
    }
    if (cmd_data.update_x_state) {
      // 发送更新x轴命令 ACTX + position_x
      writeCmd(name, "ACTX", cmd_data.update_x);
      cmd_data.update_x_state = false;
    }
    if (cmd_data.update_y_state) {
      // 发送更新y轴命令 ACTY + position_y
      writeCmd(name, "ACTY", cmd_data.update_y);
      cmd_data.update_y_state = false;
    }
    if (cmd_data.update_yaw_state) {
      // 发送更新yaw轴命令 ACTJ + yaw_angle
      writeCmd(name, "ACTJ", cmd_data.update_yaw);
      cmd_data.update_yaw_state = false;
    }
    if (cmd_data.update_x_y_state) {
      // 发送更新xy轴命令 ACTXY + position_x + position_y
      std::vector<float> dataList = {cmd_data.update_x, cmd_data.update_y};
      writeCmd(name, "ACTD", dataList);
      cmd_data.update_x_y_state = false;
    }
    if (cmd_data.update_yaw_x_y_state) {
      // 发送更新yaw和xy轴命令 ACTA + yaw_angle + position_x + position_y
      std::vector<float> dataList = {cmd_data.update_yaw, cmd_data.update_x,
                                     cmd_data.update_y};
      writeCmd(name, "ACTA", dataList);
      cmd_data.update_yaw_x_y_state = false;
    }
  }
  return true;
}

void Ops9ResourceManager::bindToHandle(
    std::vector<common::Ops9Handle>& ops9_handles) {
  for (auto& it : ops9_serial_map_) {
    const std::string& name = it.first;
    auto itRev = ops9_rev_data_map_.find(name);
    if (itRev == ops9_rev_data_map_.end()) {
      ROS_ERROR_STREAM("Device " << name << " not found in resource maps.");
      return;
    }
    auto& rev = itRev->second;
    auto itCmd = ops9_cmd_data_map_.find(name);
    if (itCmd == ops9_cmd_data_map_.end()) {
      ROS_ERROR_STREAM("Device " << name << " not found in command data map.");
      return;
    }
    auto& cmd = itCmd->second;
    ops9_handles.push_back(common::Ops9Handle(
        name, &rev.pos_x, &rev.pos_y, &rev.yaw_angle, &rev.omega,
        &cmd.calibration_state, &cmd.reset_state, &cmd.update_x_state,
        &cmd.update_y_state, &cmd.update_yaw_state, &cmd.update_yaw_x_y_state,
        &cmd.update_x_y_state, &cmd.update_x, &cmd.update_y, &cmd.update_yaw));
  }
}

void Ops9ResourceManager::unPack(const std::string& name, const uint8_t data) {
  auto it = ops9_serial_map_.find(name);
  auto itRev = ops9_rev_data_map_.find(name);
  if (it == ops9_serial_map_.end() || itRev == ops9_rev_data_map_.end()) {
    ROS_ERROR_STREAM("Device " << name << " not found in resource maps.");
    return;
  }
  auto& settings = it->second;
  auto& rev = itRev->second;
  switch (settings.serial_flam_mark) {
    case serial_hw::SerialFlamMark::START:
      if (data == 0x0d) {
        settings.serial_flam_mark = serial_hw::SerialFlamMark::FIRST_HEAD;
        // ROS_INFO_STREAM("Ops9 [" << name << "] data received: 0x0d");
      } else {
        settings.serial_flam_mark = serial_hw::SerialFlamMark::START;
      }
      break;
    case serial_hw::SerialFlamMark::FIRST_HEAD:
      if (data == 0x0a) {
        settings.serial_data_pose = 0;
        settings.serial_flam_mark = serial_hw::SerialFlamMark::SECOND_HEAD;
      } else if (data == 0x0d) {
        settings.serial_flam_mark = serial_hw::SerialFlamMark::FIRST_HEAD;
      } else {
        settings.serial_flam_mark = serial_hw::SerialFlamMark::START;
      }
      break;
    case serial_hw::SerialFlamMark::SECOND_HEAD:
      settings.data_trans.ch_data[settings.serial_data_pose] = data;
      settings.serial_data_pose++;
      if (settings.serial_data_pose >= 24) {
        settings.serial_data_pose = 0;
        settings.serial_flam_mark = serial_hw::SerialFlamMark::DATA;
      }
      break;
    case serial_hw::SerialFlamMark::DATA:
      if (data == 0x0a) {
        settings.serial_flam_mark = serial_hw::SerialFlamMark::FIRST_TAIL;
      } else {
        settings.serial_flam_mark = serial_hw::SerialFlamMark::START;
      }
      break;
    case serial_hw::SerialFlamMark::FIRST_TAIL:
      if (data == 0x0d) {
        rev.yaw_angle = angToRad(settings.data_trans.f_data4[0]);
        rev.pos_x = settings.data_trans.f_data4[3] / 1000.0;
        rev.pos_y = settings.data_trans.f_data4[4] / 1000.0;
        rev.omega = angToRad(settings.data_trans.f_data4[5]);
        // ROS_INFO_STREAM("Ops9 ["
        //                 << name << "] data received: yaw=" << rev.yaw_angle
        //                 << ", pos_x=" << rev.pos_x << ", pos_y=" << rev.pos_y
        //                 << ", omega=" << rev.omega);
      }
      settings.serial_flam_mark = serial_hw::SerialFlamMark::START;
      break;
    default:
      settings.serial_flam_mark = serial_hw::SerialFlamMark::START;
      break;
  }
}

// 不用附带数据的发送
bool Ops9ResourceManager::writeCmd(const std::string& name,
                                   const std::string cmd) {
  auto it = ops9_serial_map_.find(name);
  if (it == ops9_serial_map_.end()) {
    ROS_ERROR_STREAM("Device " << name << " not found in resource maps.");
    return false;
  }
  auto& settings = it->second;
  if (ros::Time::now() - settings.last_write_time > ros::Duration(0.01)) {
    if (write(settings.serial_fd, (char*)cmd.data(), cmd.length()) != -1) {
      ROS_INFO_STREAM("Successful write command to " << name << ".");
      settings.last_write_time = ros::Time::now();
    } else {
      ROS_ERROR_STREAM("Failed to write command to " << name << ".");
    }
  }
  return true;
}

// 附带数据的发送
bool Ops9ResourceManager::writeCmd(const std::string& name,
                                   const std::string cmd, const float data) {
  auto it = ops9_serial_map_.find(name);
  if (it == ops9_serial_map_.end()) {
    ROS_ERROR_STREAM("Device " << name << " not found in resource maps.");
    return false;
  }
  auto& settings = it->second;

  // 利用 union，将数据存放到第一个 float 中
  settings.cmd_trans.f_data[0] = data;

  // 构造发送缓冲区：先附加命令头部，再附加 4 字节二进制数据
  std::string fullCmd;
  fullCmd.append(cmd);
  fullCmd.append(settings.cmd_trans.ch_data, sizeof(float));  // 4字节

  if (ros::Time::now() - settings.last_write_time > ros::Duration(0.01)) {
    if (write(settings.serial_fd, fullCmd.data(), fullCmd.size()) != -1) {
      ROS_INFO_STREAM("Successful write command to " << name
                                                     << ": (binary cmd)");
      settings.last_write_time = ros::Time::now();
    } else {
      ROS_ERROR_STREAM("Failed to write command to " << name << ".");
      return false;
    }
  } else {
    ROS_WARN_STREAM("Please keep "
                    << name << " in absolute stillness and continuous 15s.");
  }
  return true;
}

// 新增支持多个数据的发送，dataList中依次包含各个数值，如更新XY坐标或AngleXY
bool Ops9ResourceManager::writeCmd(const std::string& name,
                                   const std::string cmd,
                                   const std::vector<float>& dataList) {
  auto it = ops9_serial_map_.find(name);
  if (it == ops9_serial_map_.end()) {
    ROS_ERROR_STREAM("Device " << name << " not found in resource maps.");
    return false;
  }
  auto& settings = it->second;

  // 最多只能发送3个数据
  size_t count = dataList.size();
  if (count > 3) {
    count = 3;
  }

  // 将数据依次存放到 union 的 f_data 数组中
  for (size_t i = 0; i < count; ++i) {
    settings.cmd_trans.f_data[i] = dataList[i];
  }

  // 构造发送缓冲区：先附加命令头部，再附加 count * sizeof(float) 字节二进制数据
  std::string fullCmd;
  fullCmd.append(cmd);
  fullCmd.append(settings.cmd_trans.ch_data, count * sizeof(float));

  if (ros::Time::now() - settings.last_write_time > ros::Duration(0.01)) {
    if (write(settings.serial_fd, fullCmd.data(), fullCmd.size()) != -1) {
      ROS_INFO_STREAM("Successful write command to " << name
                                                     << ": (binary cmd)");
      settings.last_write_time = ros::Time::now();
    } else {
      ROS_ERROR_STREAM("Failed to write command to " << name << ".");
      return false;
    }
  } else {
    ROS_WARN_STREAM("Please keep "
                    << name << " in absolute stillness and continuous 15s.");
  }
  return true;
}

}  // namespace serial_hw