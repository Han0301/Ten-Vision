/**
 * @file rc_ecat_node.cpp
 * @author Keten (2863861004@qq.com)
 * @brief ecat 主节点程序
 * @version 0.1
 * @date 2025-05-02
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include <ros/ros.h>
#include <soem_rsl/ethercat.h>

#include <thread>

#include "any_node/ThreadedPublisher.hpp"
#include "any_worker/Worker.hpp"
#include "rc_ecat_master/RcEcatSlave.hpp"
#include "ros/time.h"
#include "sensor_msgs/JointState.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "rc_ecat_node");
  ros::NodeHandle nh;
  const std::string busName = "enx10e04c360200";
  const std::string slaveName = "RcEcatSlave";
  const uint32_t slaveAddress = 1;

  /* 实例化ethercat主站基类 */
  std::unique_ptr<soem_interface_rsl::EthercatBusBase> bus =
      std::make_unique<soem_interface_rsl::EthercatBusBase>(busName);
  /* 实例化从站基类(使用get得到智能指针的原地址) */
  std::shared_ptr<rc_ecat_master::RcEcatSlave> slave =
      std::make_shared<rc_ecat_master::RcEcatSlave>(slaveName, bus.get(),
                                                    slaveAddress);
  /* 往主站添加从站设备 */
  bus->addSlave(slave);
  bus->startup(true);
  bus->setState(EC_STATE_OPERATIONAL);

  if (!bus->waitForState(EC_STATE_OPERATIONAL, slaveAddress)) {
    return 1;
  }
  /* 创建一个发布者,事件驱动机制，发布频率取决于publish调用频率 */
  any_node::ThreadedPublisherPtr<sensor_msgs::JointState> joint_state_publisher;
  joint_state_publisher =
      std::make_shared<any_node::ThreadedPublisher<sensor_msgs::JointState>>(
          nh.advertise<sensor_msgs::JointState>("m3508_state", 10), 50, false);

  sensor_msgs::JointState joint_state_msg;

  joint_state_msg.name = {"m3508_1"};
  /* 开启一个线程，进行数据的打印 */
  any_worker::Worker master_controller(
      "master_controller", 0.0005,
      [&](const any_worker::WorkerEvent& event) -> bool {
        auto tp = bus->getUpdateReadStamp();
        double secs =
            std::chrono::duration<double>(tp.time_since_epoch()).count();
        uint32_t sec = static_cast<uint32_t>(secs);
        uint32_t nsec = static_cast<uint32_t>((secs - sec) * 1e9);
        joint_state_msg.header.stamp.sec = sec;
        joint_state_msg.header.stamp.nsec = nsec;
        bus->updateRead();
        // joint_state_msg.position = {slave->getMotorPosition(
        //     rc_ecat_master::RcEcatSlave::CanBus::CAN0, 0)};
        // joint_state_msg.velocity = {slave->getMotorVelocity(
        //     rc_ecat_master::RcEcatSlave::CanBus::CAN0, 0)};
        // joint_state_msg.effort = {slave->getMotorEffort(
        //     rc_ecat_master::RcEcatSlave::CanBus::CAN0, 0)};
        // joint_state_publisher->publish(joint_state_msg);
        // joint_state_publisher->sendRos();
        bus->updateWrite();
        return true;
      });

  master_controller.start(47);

  /* while中不断地更新数据 */
  while (1) {
    std::cout << *slave;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  return 0;
}
