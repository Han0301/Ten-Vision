/**
 * @file GpioResourceManager.h
 * @author Keten (2863861004@qq.com)
 * @brief 基于Ecat从站的GPIO硬件资源管理器
 * @version 0.1
 * @date 2025-05-08
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * 实现效果应该是，从站会开发一排gpio口给上层控，上层不需要知道其在从站板子上的具体引脚号
 *         从站开放两个表：输入的gpio表 和
 * 输出的gpio表（序号都是各自的，从0开始）
 *         上层可以通过这个表来读写gpio口，主站只需要为自己要控的gpio口指定一个名字和序号即可
 *
 *        控制器的编写：通过name去获取句柄
 *
 * 组合设计模式：由于本 Gpio硬件资源管理器基于RcEcatSlave从站
 *             所以需要将从站的引用传入，在实例化时保存一个从站对象的引用
 * @versioninfo :
 */
#pragma once

#include <rc_ecat_master/ecat_hardware_interface/gpio/GpioInterface.h>

#include <ostream>
#include <rc_ecat_master/RcEcatSlave.hpp>
#include <unordered_map>
#include <vector>

namespace ecat_slave_hw {

class GpioResourceManager {
 public:
  explicit GpioResourceManager(rc_ecat_master::RcEcatSlave& slave)
      : slave_(slave) {}
  GpioResourceManager() = delete;
  ~GpioResourceManager() = default;

  bool addGpioData(const std::string& name, const GpioData& gpio_data);

  std::unordered_map<int32_t, bool> getGpioData(
      const ecat_slave_hw::GpioType& type);

  void bindToStateHandle(std::vector<GpioStateHandle>& state_handles);

  void bindToCommandHandle(std::vector<GpioCommandHandle>& command_handles);

  /**
   * @brief 放在ros-control中的 write中，准备好数据直接一次性写入
   *
   * @param gpio_data
   */
  void writeGpioData();

  /**
   * @brief 放在ros-control中的read中，返回后再解析
   *
   * @return * std::unordered_map<std::string, bool>
   */
  void readGpioData(void);

  friend std::ostream& operator<<(std::ostream& os,
                                  const GpioResourceManager& tmp);

 private:
  // 从站的引用
  rc_ecat_master::RcEcatSlave& slave_;

  // gpio data ，这是一个以 std::string 为key
  // ，GpioData为value的map，便于根据名字来获取硬件接口
  ecat_slave_hw::GpioDataMap input_gpio_map_;
  ecat_slave_hw::GpioDataMap output_gpio_map_;
};

}  // namespace ecat_slave_hw