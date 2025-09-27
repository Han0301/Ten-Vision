/**
 * @file SerialTo32ResourceManager.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-05-17
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include <rc_ecat_master/serial_hw/serial_to_32/SerialTo32ResourceManager.h>

namespace serial_hw {

bool SerialTo32ResourceManager::init() {
  // 初始化serial_member
  if (!serial_member_.init()) {
    ROS_ERROR("Failed to initialize SerialMember");
    return false;
  }
  // 要将数据结构体的指针传入
  rev_data_.float_data =
      static_cast<float*>(serial_member_.getFieldDataPtr("float_data", false));
  rev_data_.uint8_data = static_cast<uint8_t*>(
      serial_member_.getFieldDataPtr("uint8_data", false));
  send_data_.float_data =
      static_cast<float*>(serial_member_.getFieldDataPtr("float_data", true));
  send_data_.uint8_data =
      static_cast<uint8_t*>(serial_member_.getFieldDataPtr("uint8_data", true));

  return true;
}

// 读取数据
// serial_member在实例化时就会开启一个读取线程，后续可以加一个选项取消那里线程的读取，但看上去是没什么必要
// 因为不可能将其放到主线程中去读取，肯定会另开线程进行读取
void SerialTo32ResourceManager::read() {
  // 读取数据
  serial_member_.read();
}

void SerialTo32ResourceManager::write() {
  // 设置写入值
  serial_member_.write();
}
}  // namespace serial_hw