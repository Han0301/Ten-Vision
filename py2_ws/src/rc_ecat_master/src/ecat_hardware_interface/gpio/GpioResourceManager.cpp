/**
 * @file GpioResourceManager.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-05-09
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note : 怎么说，你可以不用这个io，但是我这个io照样会更新
 *         在RobotHW那里读取了使用的gpio数量后，在那边会有一个设置GPIO的接口
 *         在那边就是真正要使用的gpio数量
 *
 *         这里要依赖于从站的函数
 * @versioninfo :
 */
#include <rc_ecat_master/ecat_hardware_interface/gpio/GpioResourceManager.h>

namespace ecat_slave_hw {

bool GpioResourceManager::addGpioData(const std::string& name,
                                      const GpioData& gpio_data) {
  if (gpio_data.type == GpioType::INPUT) {
    // check first if there are some fool who add the same gpio
    for (const auto& it : input_gpio_map_) {
      if (it.second.id == gpio_data.id) {
        ROS_ERROR_STREAM("add a duplicate gpio id: " << gpio_data.id);
        return false;  // 错误的GPIO类型
      }
      if (name == it.first) {
        ROS_ERROR_STREAM("add a duplicate gpio name: " << name);
        return false;  // 错误的GPIO类型
      }
    }
    input_gpio_map_[name] = gpio_data;
  } else if (gpio_data.type == GpioType::OUTPUT) {
    for (const auto& it : output_gpio_map_) {
      if (it.second.id == gpio_data.id) {
        ROS_ERROR_STREAM("add a duplicate gpio id: " << gpio_data.id);
        return false;  // 错误的GPIO类型
      }
      if (name == it.first) {
        ROS_ERROR_STREAM("add a duplicate gpio name: " << name);
        return false;  // 错误的GPIO类型
      }
    }
    output_gpio_map_[name] = gpio_data;
  } else {
    return false;  // 错误的GPIO类型
  }
  return true;
}

std::unordered_map<int32_t, bool> GpioResourceManager::getGpioData(
    const ecat_slave_hw::GpioType& type) {
  std::unordered_map<int32_t, bool> gpio_data;
  if (type == ecat_slave_hw::GpioType::INPUT) {
    // 读取从站的gpio数据
    for (const auto& it : input_gpio_map_) {
      auto& data = it.second;
      gpio_data[data.id] = data.value;
    }
  } else if (type == ecat_slave_hw::GpioType::OUTPUT) {
    for (const auto& it : output_gpio_map_) {
      auto& data = it.second;
      gpio_data[data.id] = data.value;
    }
  }
  return gpio_data;
}

void GpioResourceManager::bindToStateHandle(
    std::vector<GpioStateHandle>& state_handles) {
  // 对于输入和输出都添加
  for (auto& it : input_gpio_map_) {
    auto& gpio_data = it.second;
    state_handles.push_back(GpioStateHandle(it.first, gpio_data.type,
                                            gpio_data.id, &gpio_data.value));
  }
  for (auto& it : output_gpio_map_) {
    auto& gpio_data = it.second;
    state_handles.push_back(GpioStateHandle(it.first, gpio_data.type,
                                            gpio_data.id, &gpio_data.value));
  }
}

void GpioResourceManager::bindToCommandHandle(
    std::vector<GpioCommandHandle>& command_handles) {
  for (auto& it : output_gpio_map_) {
    auto& gpio_data = it.second;
    command_handles.push_back(GpioCommandHandle(
        it.first, gpio_data.type, gpio_data.id, &gpio_data.value));
  }
}

void GpioResourceManager::writeGpioData() {
  std::unordered_map<int32_t, bool> cmds;
  for (auto& it : output_gpio_map_) {
    auto& gpio_data = it.second;
    cmds[gpio_data.id] = gpio_data.value;
  }

  // 调用从站的函数来写入数据
  slave_.setOutputGpioData(cmds);
}

void GpioResourceManager::readGpioData() {
  // 读取从站的gpio数据
  std::unordered_map<int32_t, bool> input_gpio_data = slave_.getInputGpioData();
  std::unordered_map<int32_t, bool> output_gpio_data =
      slave_.getOutputGpioData();
  for (auto& it : input_gpio_map_) {
    for (const auto& data : input_gpio_data) {
      if (it.second.id == data.first) {
        it.second.value = data.second;
      }
    }
  }
  for (auto& it : output_gpio_map_) {
    for (const auto& data : output_gpio_data) {
      if (it.second.id == data.first) {
        it.second.value = data.second;
      }
    }
  }
}

// 提供打印函数
std::ostream& operator<<(std::ostream& os, const GpioResourceManager& tmp) {
  std::stringstream ss;
  // 假设总共打印约10行，可以根据实际情况调整参数
  ss << "\033[10F";
  ss << "\033[K"
     << "Input Gpio States:" << std::endl;
  for (const auto& it : tmp.input_gpio_map_) {
    ss << "\033[K"
       << "Gpio Name: " << it.first << ", Gpio ID: " << it.second.id
       << ", Gpio Type: "
       << (it.second.type == ecat_slave_hw::GpioType::INPUT ? "INPUT"
                                                            : "OUTPUT")
       << ", Gpio Value: " << it.second.value << std::endl;
  }
  ss << "\033[K"
     << "Output Gpio States:" << std::endl;
  for (const auto& it : tmp.output_gpio_map_) {
    ss << "\033[K"
       << "Gpio Name: " << it.first << ", Gpio ID: " << it.second.id
       << ", Gpio Type: "
       << (it.second.type == ecat_slave_hw::GpioType::INPUT ? "INPUT"
                                                            : "OUTPUT")
       << ", Gpio Value: " << it.second.value << std::endl;
  }
  os << ss.str();
  return os;
}
}  // namespace ecat_slave_hw