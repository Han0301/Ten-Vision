/**
 * @file serial_test_node.cpp
 * @author Keten (2863861004@qq.com)
 * @brief 测试serial manager
 * @version 0.1
 * @date 2025-05-17
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note : 测试用例，需要注意，float_data 和 uint8_t_data 以及 send~
 *          相关都是根据你的协议来设置的
 *         然后是命名空间问题，建议加上命名，不直接使用全局，以免多个serial_member参数服务器相互覆盖
 *         然后是在init时传入句柄
 * @versioninfo :
 */
#include <ros/ros.h>
#include <serial_manager/SerialManager.h>

#include <any_worker/Worker.hpp>

float* float_data = nullptr;
uint8_t* uint8_t_data = nullptr;

float* send_float_data = nullptr;
uint8_t* send_uint8_t_data = nullptr;

bool printWorkerCb(const any_worker::WorkerEvent& event) {
  if (float_data) {
    std::cout << "Float data: ";
    for (int i = 0; i < 6; ++i) {
      std::cout << float_data[i] << " ";
    }
    std::cout << std::endl;
  }
  if (uint8_t_data) {
    std::cout << "uint8_t data: ";
    for (int i = 0; i < 4; ++i) {
      std::cout << static_cast<int>(uint8_t_data[i]) << " ";
    }
    std::cout << std::endl;
  }
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "serial_test_node");
  ros::NodeHandle nh("serial_test");
  // ros::NodeHandle nh_private("~serial_test");
  serial_manager::SerialMember serial_member(nh);
  if (!serial_member.init()) {
    ROS_ERROR("Failed to initialize SerialMember");
    return -1;
  }
  float_data =
      static_cast<float*>(serial_member.getFieldDataPtr("float_data", false));
  uint8_t_data =
      static_cast<uint8_t*>(serial_member.getFieldDataPtr("uint8_data", false));
  send_float_data =
      static_cast<float*>(serial_member.getFieldDataPtr("float_data", true));
  send_uint8_t_data =
      static_cast<uint8_t*>(serial_member.getFieldDataPtr("uint8_data", true));

  // 设置打印线程
  std::shared_ptr<any_worker::Worker> printWorker =
      std::make_shared<any_worker::Worker>(
          "printWorker", 0.1, std::bind(printWorkerCb, std::placeholders::_1));
  std::shared_ptr<any_worker::Worker> writeWorker =
      std::make_shared<any_worker::Worker>(
          "writeWorker", 0.1,
          [&serial_member](const any_worker::WorkerEvent& event) -> bool {
            if (send_float_data) {
              for (int i = 0; i < 6; ++i) {
                send_float_data[i] += 0.1f;
              }
            }
            if (send_uint8_t_data) {
              for (int i = 0; i < 4; ++i) {
                send_uint8_t_data[i] += 1;
              }
            }
            // 例如调用 serial_member 的写操作
            serial_member.write();
            return true;
          });
  printWorker->start();
  writeWorker->start();
  ros::spin();

  return 0;
}
