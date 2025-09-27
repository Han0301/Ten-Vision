/**
 * @file SerialTo32Interface.h
 * @author Keten (2863861004@qq.com)
 * @brief 嵌入式微处理器stm32的通信接口，可以通过该接口与stm32进行数据交互
 * @version 0.1
 * @date 2025-05-17
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note : 依赖包：serial_manager
 *         protocol部分 定义在config中SerialTo32.yaml
 *         当前这个硬件接口是根据upperboard协议写的
 * @versioninfo :
 */
#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>

#include <string>

namespace common {

struct RevData {
  float* float_data;
  uint8_t* uint8_data;
};

struct SendData {
  float* float_data;
  uint8_t* uint8_data;
};

class SerialTo32Handle {
 public:
  SerialTo32Handle() = default;
  explicit SerialTo32Handle(const std::string& name, float* float_data,
                            uint8_t* uint8_data, float* send_float_data,
                            uint8_t* send_uint8_t_data)
      : name_(name),
        float_data(float_data),
        uint8_t_data(uint8_data),
        send_float_data(send_float_data),
        send_uint8_t_data(send_uint8_t_data){};

  ~SerialTo32Handle() = default;
  std::string getName() const { return name_; }

  float* getFloatData() {
    assert(float_data);
    return float_data;
  }

  uint8_t* getUint8Data() {
    assert(uint8_t_data);
    return uint8_t_data;
  }

  void setFloatData(float* data) {
    assert(data);
    assert(send_float_data);
    for (int i = 0; i < 6; ++i) {
      send_float_data[i] = data[i];
    }
  }
  void setUint8Data(uint8_t* data) {
    assert(data);
    assert(send_uint8_t_data);
    for (int i = 0; i < 4; ++i) {
      send_uint8_t_data[i] = data[i];
    }
  }

  void setZeroAllData() {
    assert(send_float_data);
    assert(send_uint8_t_data);
    for (int i = 0; i < 6; ++i) {
      send_float_data[i] = 0;
    }
    for (int i = 0; i < 4; ++i) {
      send_uint8_t_data[i] = 0;
    }
  }

  void setZeroUint8Data() {
    assert(send_uint8_t_data);
    for (int i = 0; i < 4; ++i) {
      send_uint8_t_data[i] = 0;
    }
  }

  void setZeroFloatData() {
    assert(send_float_data);
    for (int i = 0; i < 6; ++i) {
      send_float_data[i] = 0;
    }
  }

 private:
  std::string name_;
  /* rev data part */
  float* float_data = {nullptr};
  uint8_t* uint8_t_data = {nullptr};
  /* send data part */
  float* send_float_data = {nullptr};
  uint8_t* send_uint8_t_data = {nullptr};
};

class SerialTo32Interface
    : public hardware_interface::HardwareResourceManager<
          SerialTo32Handle, hardware_interface::ClaimResources> {};
}  // namespace common