/**
 * @file SerialManager.h
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-05-16
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once
#include <XmlRpc.h>
#include <ros/ros.h>
#include <serial/serial.h>

#include <any_worker/Worker.hpp>
#include <iostream>
#include <memory>
#include <signal_handler/SignalHandler.hpp>

namespace serial_manager {

// 定义 PacketField，用于描述数据包中的每个字段
struct PacketField {
  std::string name;
  std::string type;  // "float", "bool", "uint8_t", "uint16_t", etc.
  int count;         // 字段中的数据量
  int byteSize;      // count * (每个数据类型所占字节)
  int offset;        // 字段在数据包中的偏移量
};

class SerialMember {
 public:
  SerialMember(ros::NodeHandle& nh);
  ~SerialMember();

  bool init();

  void write();

  void read();

  bool readWorkerCb(const any_worker::WorkerEvent& event);

  void handleSignal(int /* signum */);

  void* getFieldDataPtr(const std::string& fieldName, bool if_send);

 private:
  std::vector<PacketField> parsePacketFields(
      const XmlRpc::XmlRpcValue& fieldsConfig);

  // 打包发送数据
  void packetSendData();

  // 解析接收数据
  void unpacketRevData();

  bool openPort();

  void closePort();

  ros::NodeHandle nh_;
  std::string name_;
  /* serial relative */
  std::string serial_port_;
  std::string serial_com_;
  uint32_t baudrate_;
  uint32_t timeout_;
  std::unique_ptr<serial::Serial> serial_;
  serial::Timeout to_;

  /* protocol package 设定 */
  std::vector<unsigned char> header_buf_;
  std::vector<unsigned char> tail_buf_;
  std::vector<PacketField> send_data_packet_;
  std::vector<PacketField> rev_data_packet_;
  bool if_rev_crc_check{false};
  bool if_send_crc_check{false};

  std::vector<unsigned char> sendBuffer_;  // 发送buffer
  std::vector<unsigned char> revBuffer_;   // 接收buffer

  /* worker */
  double read_worker_cycle_;
  std::shared_ptr<any_worker::Worker> readWorker_{};
  double write_worker_cycle_;
  std::shared_ptr<any_worker::Worker>
      writeWorker_{};  // 设置发布线程，定时发布的接口

  /* test */
};

}  // namespace serial_manager