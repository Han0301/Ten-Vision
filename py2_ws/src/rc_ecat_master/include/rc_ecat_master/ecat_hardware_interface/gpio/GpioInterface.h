/**
 * @file GpioInterface.h
 * @author Keten (2863861004@qq.com)
 * @brief 基于Ecat从站的 GPIO硬件接口
 * @version 0.1
 * @date 2025-05-08
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>

#include <memory>
#include <string>
#include <unordered_map>

namespace ecat_slave_hw {

enum class GpioType { INPUT, OUTPUT };

struct GpioData {
  uint32_t id;
  GpioType type;
  bool value;
};
// 名字 - gpio data 这样存
using GpioDataMap = std::unordered_map<std::string, GpioData>;

class GpioStateHandle {
 public:
  GpioStateHandle() = default;
  GpioStateHandle(std::string name, GpioType type, int32_t id, bool *value)
      : name_(std::move(name)), type_(type), id_(id), value_(value) {}

  std::string getName() const { return name_; }
  bool getNowState() const { return *(value_); }

 private:
  std::string name_;
  int32_t id_;
  GpioType type_;
  bool *value_{};
};

class GpioCommandHandle {
 public:
  GpioCommandHandle() = default;
  GpioCommandHandle(std::string name, GpioType type, int32_t id, bool *value)
      : name_(std::move(name)), type_(type), id_(id), value_(value) {}
  std::string getName() const { return name_; }

  bool getNowState() const {
    assert(value_);  // 断言，如果value_为空则抛出异常
    return *(value_);
  }

  void setCommand(bool value) {
    assert(value_);  // 断言，如果value_为空则抛出异常
    *(value_) = value;
  }

 private:
  std::string name_;
  int32_t id_;
  GpioType type_;
  bool *value_{};
};

class GpioStateInterface
    : public hardware_interface::HardwareResourceManager<
          GpioStateHandle, hardware_interface::DontClaimResources> {};

class GpioCommandInterface
    : public hardware_interface::HardwareResourceManager<
          GpioCommandHandle, hardware_interface::ClaimResources> {};

};  // namespace ecat_slave_hw