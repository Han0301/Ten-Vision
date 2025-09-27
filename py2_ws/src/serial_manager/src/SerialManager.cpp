/**
 * @file SerialManager.cpp
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
#include <serial_manager/SerialManager.h>

namespace serial_manager {

// CRC8 检验函数
unsigned char serial_get_crc8_value(unsigned char* data, unsigned char len) {
  unsigned char crc = 0;
  while (len--) {
    crc ^= *data++;
    for (unsigned char i = 0; i < 8; i++) {
      crc = (crc & 0x01) ? ((crc >> 1) ^ 0x8C) : (crc >> 1);
    }
  }
  return crc;
}

int getTypeSize(const std::string& type) {
  if (type == "float") {
    return sizeof(float);
  } else if (type == "bool") {
    return sizeof(bool);
  } else if (type == "uint8_t") {
    return sizeof(uint8_t);
  } else if (type == "uint16_t") {
    return sizeof(uint16_t);
  } else if (type == "int8_t") {
    return sizeof(int8_t);
  } else if (type == "int16_t") {
    return sizeof(int16_t);
  }
  return -1;  // Unknown type
}

SerialMember::SerialMember(ros::NodeHandle& nh)
    : nh_(nh), serial_(std::make_unique<serial::Serial>()) {}

SerialMember::~SerialMember() {}

bool SerialMember::init() {
  XmlRpc::XmlRpcValue config;
  if (!nh_.getParam("protocol", config)) {
    ROS_ERROR("Failed to get param: protocol");
    return false;
  }
  if (config.hasMember("name") == false) {
    ROS_ERROR("name not found in protocol");
    return false;
  }
  name_ = static_cast<std::string>(config["name"]);
  if (config.hasMember("serial_port") == false) {
    ROS_ERROR("serial_port not found in protocol");
    return false;
  }
  serial_port_ = static_cast<std::string>(config["serial_port"]);
  if (config.hasMember("baudrate") == false) {
    ROS_ERROR("baudrate not found in protocol");
    return false;
  }
  baudrate_ = static_cast<int>(config["baudrate"]);
  if (config.hasMember("timeout") == false) {
    ROS_ERROR("timeout not found in protocol");
    return false;
  }
  timeout_ = static_cast<int>(config["timeout"]);
  if (config.hasMember("read_worker_cycle") == false) {
    ROS_ERROR("read_worker_cycle not found in protocol");
    return false;
  }
  read_worker_cycle_ = static_cast<double>(config["read_worker_cycle"]);
  if (config.hasMember("if_rev_crc_check")) {
    if_rev_crc_check = static_cast<bool>(config["if_rev_crc_check"]);
  } else {
    ROS_WARN("if_rev_crc_check not found in protocol");
  }
  if (config.hasMember("if_send_crc_check")) {
    if_send_crc_check = static_cast<bool>(config["if_send_crc_check"]);
  } else {
    ROS_WARN("if_send_crc_check not found in protocol");
  }

  // 解析设置的帧头和帧尾
  XmlRpc::XmlRpcValue header = config["header"];
  if (header.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    int header_size = header.size();
    header_buf_.resize(header_size);  // 要先调整大小
    for (int i = 0; i < header_size; i++) {
      header_buf_[i] = static_cast<unsigned char>(static_cast<int>(header[i]));
    }
  } else {
    ROS_ERROR("header not found in protocol");
    return false;
  }
  XmlRpc::XmlRpcValue tailer = config["tailer"];
  if (tailer.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    int tailer_size = tailer.size();
    tail_buf_.resize(tailer_size);  // 要先调整大小
    for (int i = 0; i < tailer_size; i++) {
      tail_buf_[i] = static_cast<unsigned char>(static_cast<int>(tailer[i]));
    }
  }
  // 解析发送包字段
  if (!config.hasMember("send_packet_fields"))
    throw std::runtime_error("send_packet_fields not found");
  XmlRpc::XmlRpcValue sendFields = config["send_packet_fields"];
  send_data_packet_ = parsePacketFields(sendFields);

  // 解析接收包字段
  if (!config.hasMember("rev_packet_fields"))
    throw std::runtime_error("rev_packet_fields not found");
  XmlRpc::XmlRpcValue revFields = config["rev_packet_fields"];
  rev_data_packet_ = parsePacketFields(revFields);

  // 计算中间数据字段总字节数，针对发送包
  int dataSize = 0;
  for (const auto& field : send_data_packet_) {
    dataSize += field.byteSize;
  }
  // 计算整个包总长度
  int totalSize = header_buf_.size() + dataSize + tail_buf_.size();
  sendBuffer_.resize(totalSize);

  // 将 header 写入起始位置
  std::copy(header_buf_.begin(), header_buf_.end(), sendBuffer_.begin());

  // 将 tailer 写入末尾
  std::copy(tail_buf_.begin(), tail_buf_.end(),
            sendBuffer_.begin() + header_buf_.size() + dataSize);

  // 计算字段总字节数，针对接收包
  dataSize = 0;
  totalSize = 0;
  for (const auto& field : rev_data_packet_) {
    dataSize += field.byteSize;
  }
  totalSize = header_buf_.size() + tail_buf_.size() + dataSize;  // 包头+包尾
  revBuffer_.resize(totalSize);
  // 开启串口端口
  if (openPort()) {
    ROS_INFO("Serial port opened successfully");
  } else {
    ROS_ERROR("Failed to open serial port");
    return false;
  }

  // 创建工作线程
  readWorker_ = std::make_shared<any_worker::Worker>(
      name_ + "readWorker", read_worker_cycle_,
      std::bind(&SerialMember::readWorkerCb, this, std::placeholders::_1));

  signal_handler::SignalHandler::bindAll(&SerialMember::handleSignal, this);

  readWorker_->start(45);
  return true;
}

void SerialMember::write() {
  // 打包发送数据
  packetSendData();

  // 打印原始发送数据
  // std::stringstream ss;
  // for (const auto& byte : sendBuffer_) {
  //   ss << "0x" << std::hex << static_cast<int>(byte) << " ";
  // }
  // ROS_INFO_STREAM("Raw send data: " << ss.str());

  // 封装写操作，防止设备断开时的异常导致程序崩溃
  try {
    if (serial_->isOpen()) {
      serial_->write(sendBuffer_);
    } else {
      ROS_ERROR_STREAM_THROTTLE(1.0,
                                "Serial port is not open, cannot write data");
      // 根据需要可以标记状态或等待下次重连
    }
  } catch (const serial::IOException& e) {
    ROS_ERROR_STREAM_THROTTLE(1.0,
                              "Serial IOException in write(): " << e.what());
    serial_->close();
    ros::Duration(0.5).sleep();
    if (!openPort()) {
      ROS_ERROR("Reopen serial port failed after IOException");
    }
  } catch (const serial::SerialException& e) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "SerialException in write(): " << e.what());
    serial_->close();
    ros::Duration(0.5).sleep();
    if (!openPort()) {
      ROS_ERROR("Reopen serial port failed after SerialException");
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM_THROTTLE(1.0,
                              "Unknown exception in write(): " << e.what());
  }
}

void SerialMember::read() {
  // 读取数据
  try {
    std::string tail(reinterpret_cast<const char*>(tail_buf_.data()),
                     tail_buf_.size());
    std::string reponse = serial_->readline(revBuffer_.size(), tail);
    if (reponse.empty()) {
      // ROS_WARN_STREAM("No data received; device may be disconnected.");
      return;
    }
    std::copy(reponse.begin(), reponse.end(), revBuffer_.begin());
    // 打印原始接收数据
    // std::stringstream ss;
    // for (const auto& byte : revBuffer_) {
    //   ss << "0x" << std::hex << static_cast<int>(byte) << " ";
    // }
    // ROS_INFO_STREAM("Raw received data: " << ss.str());
  } catch (const serial::IOException& e) {  // 设备断开
    ROS_ERROR_STREAM("Serial IOException: " << e.what());
    // 尝试重新打开串口，或标记连接中断
    // 例如：closePort(); openPort();
    return;
  } catch (const std::exception& err) {
    ROS_ERROR_STREAM("Exception in read: " << err.what());
    return;
  }
  // 进行解包
  unpacketRevData();
}

bool SerialMember::readWorkerCb(const any_worker::WorkerEvent& event) {
  if (serial_->isOpen()) {
    read();
  } else {
    ROS_ERROR_STREAM_THROTTLE(0.5, "Serial port is not open");
    return false;
  }
  // 将读取到的数据打印出来进行测试
  // std::cout << "Received float data: ";
  // std::cout << rev_data.float_data[0] << " " << rev_data.float_data[1] << " "
  //           << rev_data.float_data[2] << " " << rev_data.float_data[3] << " "
  //           << rev_data.float_data[4] << " " << rev_data.float_data[5]
  //           << std::endl;
  // std::cout << "Received uint8_t data: ";
  // std::cout << static_cast<int>(rev_data.uint8_t_data[0]) << " "
  //           << static_cast<int>(rev_data.uint8_t_data[1]) << " "
  //           << static_cast<int>(rev_data.uint8_t_data[2]) << " "
  //           << static_cast<int>(rev_data.uint8_t_data[3]) << std::endl;
  return true;
}

void SerialMember::handleSignal(int /* signum */) {
  readWorker_->stop();
  if (serial_->isOpen()) {
    closePort();
  }
  ros::shutdown();
}

// if_send 为 true 时，为获取发送数据段的指针
void* SerialMember::getFieldDataPtr(const std::string& fieldName,
                                    bool if_send) {
  if (if_send)
    for (const auto& fo : send_data_packet_) {
      if (fo.name == fieldName) {
        return &sendBuffer_[fo.offset];
      }
    }
  else {
    for (const auto& fo : rev_data_packet_) {
      if (fo.name == fieldName) {
        return &revBuffer_[fo.offset];
      }
    }
  }
  ROS_ERROR("Field %s not found", fieldName.c_str());
  return nullptr;
}

// 获取 发送数据段和接收数据段中各个类型的大小及设置偏移量
std::vector<PacketField> SerialMember::parsePacketFields(
    const XmlRpc::XmlRpcValue& fieldsConfig) {
  std::vector<PacketField> fields;
  int offset = header_buf_.size();  // 从包头开始偏移
  for (int i = 0; i < fieldsConfig.size(); i++) {
    PacketField field;
    field.name = static_cast<std::string>(fieldsConfig[i]["name"]);
    field.type = static_cast<std::string>(fieldsConfig[i]["type"]);
    field.count = static_cast<int>(fieldsConfig[i]["count"]);
    field.byteSize = getTypeSize(field.type) * field.count;
    field.offset = offset;
    fields.push_back(field);
    offset += field.byteSize;  // 更新offset
  }
  return fields;
}

// 打包发送数据
void SerialMember::packetSendData() {
  int offset = header_buf_.size();  // 包头的大小
  for (const auto& field : send_data_packet_) {
    // 将发送的数据段拆解发送
    if (field.type == "float") {
      float* fieldData = static_cast<float*>(getFieldDataPtr(field.name, true));
      if (!fieldData) {
        ROS_ERROR("Failed to get field data pointer for %s",
                  field.name.c_str());
        continue;
      }
      for (int i = 0; i < field.count; i++) {
        std::memcpy(&sendBuffer_[offset], &fieldData[i], sizeof(float));
        offset += sizeof(float);
      }
    } else if (field.type == "uint8_t" && field.name == "crc_bit") {
      // 只有启用crc检验时，才会对crc位进行装填，否则装填0值
      uint8_t* fieldData =
          static_cast<uint8_t*>(getFieldDataPtr(field.name, true));
      if (!fieldData) {
        ROS_ERROR("Failed to get field data pointer for %s",
                  field.name.c_str());
        continue;
      }
      if (if_send_crc_check) {
        // CRC 校验位
        for (int i = 0; i < field.count; i++) {
          // 计算crc校验位
          fieldData[i] = serial_get_crc8_value(sendBuffer_.data(), offset);
          std::memcpy(&sendBuffer_[offset], &fieldData[i], sizeof(uint8_t));
          offset += sizeof(uint8_t);
        }
      } else {
        // 装填CRC 0值
        for (int i = 0; i < field.count; i++) {
          fieldData[i] = 0;
          std::memcpy(&sendBuffer_[offset], &fieldData[i], sizeof(uint8_t));
          offset += sizeof(uint8_t);
        }
      }
    } else if (field.type == "uint8_t" && field.name == "data_length") {
      // 装填 数据段 长度表示位
      uint8_t* fieldData =
          static_cast<uint8_t*>(getFieldDataPtr(field.name, true));
      if (!fieldData) {
        ROS_ERROR("Failed to get field data pointer for %s",
                  field.name.c_str());
        continue;
      }
      for (int i = 0; i < field.count; i++) {
        // fieldData[i] = 0;
        std::memcpy(&sendBuffer_[offset], &fieldData[i], sizeof(uint8_t));
        offset += sizeof(uint8_t);
      }
    } else if (field.type == "uint8_t") {
      uint8_t* fieldData =
          static_cast<uint8_t*>(getFieldDataPtr(field.name, true));
      if (!fieldData) {
        ROS_ERROR("Failed to get field data pointer for %s",
                  field.name.c_str());
        continue;
      }
      for (int i = 0; i < field.count; i++) {
        std::memcpy(&sendBuffer_[offset], &fieldData[i], sizeof(uint8_t));
        offset += sizeof(uint8_t);
      }
    } else if (field.type == "bool") {
      bool* fieldData = static_cast<bool*>(getFieldDataPtr(field.name, true));
      if (!fieldData) {
        ROS_ERROR("Failed to get field data pointer for %s",
                  field.name.c_str());
        continue;
      }
      for (int i = 0; i < field.count; i++) {
        std::memcpy(&sendBuffer_[offset], &fieldData[i], sizeof(bool));
        offset += sizeof(bool);
      }
    } else if (field.type == "uint16_t") {
      uint16_t* fieldData =
          static_cast<uint16_t*>(getFieldDataPtr(field.name, true));
      if (!fieldData) {
        ROS_ERROR("Failed to get field data pointer for %s",
                  field.name.c_str());
        continue;
      }
      for (int i = 0; i < field.count; i++) {
        std::memcpy(&sendBuffer_[offset], &fieldData[i], sizeof(uint16_t));
        offset += sizeof(uint16_t);
      }
    } else if (field.type == "int8_t") {
      int8_t* fieldData =
          static_cast<int8_t*>(getFieldDataPtr(field.name, true));
      if (!fieldData) {
        ROS_ERROR("Failed to get field data pointer for %s",
                  field.name.c_str());
        continue;
      }
      for (int i = 0; i < field.count; i++) {
        std::memcpy(&sendBuffer_[offset], &fieldData[i], sizeof(int8_t));
        offset += sizeof(int8_t);
      }
    } else if (field.type == "int16_t") {
      int16_t* fieldData =
          static_cast<int16_t*>(getFieldDataPtr(field.name, true));
      if (!fieldData) {
        ROS_ERROR("Failed to get field data pointer for %s",
                  field.name.c_str());
        continue;
      }
      for (int i = 0; i < field.count; i++) {
        std::memcpy(&sendBuffer_[offset], &fieldData[i], sizeof(int16_t));
        offset += sizeof(int16_t);
      }
    } else {
      ROS_ERROR("Unsupported type: %s", field.type.c_str());
    }
  }
}

// 解析接收数据
void SerialMember::unpacketRevData() {
  int offset = 0;
  for (int i = 0; i < header_buf_.size(); i++) {
    if (revBuffer_[i] != header_buf_[i]) {
      ROS_ERROR("Header mismatch");
      return;
    }
  }
  offset += header_buf_.size();
  for (int i = 0; i < tail_buf_.size(); i++) {
    if (revBuffer_[revBuffer_.size() - tail_buf_.size() + i] != tail_buf_[i]) {
      ROS_ERROR("Tail mismatch");
      return;
    }
  }
  for (const auto& field : rev_data_packet_) {
    if (field.type == "float") {
      float* fieldData =
          static_cast<float*>(getFieldDataPtr(field.name, false));
      if (!fieldData) {
        ROS_ERROR("Failed to get rev field data pointer for %s",
                  field.name.c_str());
        continue;
      }
      for (int i = 0; i < field.count; i++) {
        std::memcpy(&fieldData[i], &revBuffer_[offset], sizeof(float));
        offset += sizeof(float);
      }
    } else if (field.type == "uint8_t" && field.name == "crc_bit") {
      // 只有启用crc检验时，才会对crc位进行检查，否则跳过检查
      uint8_t* fieldData =
          static_cast<uint8_t*>(getFieldDataPtr(field.name, false));
      if (!fieldData) {
        ROS_ERROR("Failed to get rev field data pointer for %s",
                  field.name.c_str());
        continue;
      }
      if (if_rev_crc_check) {
        // 检查CRC 校验位
        uint8_t crc = serial_get_crc8_value(revBuffer_.data(), offset);
        if (crc != fieldData[0]) {
          offset += sizeof(uint8_t);
          ROS_ERROR("CRC mismatch");
          return;
        }
      } else {
        // 跳过CRC检测
      }
      offset += sizeof(uint8_t);
    } else if (field.type == "uint8_t" && field.name == "data_length") {
      uint8_t* fieldData =
          static_cast<uint8_t*>(getFieldDataPtr(field.name, false));
      if (!fieldData) {
        ROS_ERROR("Failed to get rev field data pointer for %s",
                  field.name.c_str());
        continue;
      }
      for (int i = 0; i < field.count; i++) {
        std::memcpy(&fieldData[i], &revBuffer_[offset], sizeof(uint8_t));
        offset += sizeof(uint8_t);
      }
    } else if (field.type == "uint8_t") {
      uint8_t* fieldData =
          static_cast<uint8_t*>(getFieldDataPtr(field.name, false));
      if (!fieldData) {
        ROS_ERROR("Failed to get rev field data pointer for %s",
                  field.name.c_str());
        continue;
      }
      for (int i = 0; i < field.count; i++) {
        std::memcpy(&fieldData[i], &revBuffer_[offset], sizeof(uint8_t));
        offset += sizeof(uint8_t);
      }
    } else if (field.type == "bool") {
      bool* fieldData = static_cast<bool*>(getFieldDataPtr(field.name, false));
      if (!fieldData) {
        ROS_ERROR("Failed to get rev field data pointer for %s",
                  field.name.c_str());
        continue;
      }
      for (int i = 0; i < field.count; i++) {
        std::memcpy(&fieldData[i], &revBuffer_[offset], sizeof(bool));
        offset += sizeof(bool);
      }
    } else if (field.type == "uint16_t") {
      uint16_t* fieldData =
          static_cast<uint16_t*>(getFieldDataPtr(field.name, false));
      if (!fieldData) {
        ROS_ERROR("Failed to get rev field data pointer for %s",
                  field.name.c_str());
        continue;
      }
      for (int i = 0; i < field.count; i++) {
        std::memcpy(&fieldData[i], &revBuffer_[offset], sizeof(uint16_t));
        offset += sizeof(uint16_t);
      }
    } else if (field.type == "int8_t") {
      int8_t* fieldData =
          static_cast<int8_t*>(getFieldDataPtr(field.name, false));
      if (!fieldData) {
        ROS_ERROR("Failed to get rev field data pointer for %s",
                  field.name.c_str());
        continue;
      }
      for (int i = 0; i < field.count; i++) {
        std::memcpy(&fieldData[i], &revBuffer_[offset], sizeof(int8_t));
        offset += sizeof(int8_t);
      }
    } else if (field.type == "int16_t") {
      int16_t* fieldData =
          static_cast<int16_t*>(getFieldDataPtr(field.name, false));
      if (!fieldData) {
        ROS_ERROR("Failed to get rev field data pointer for %s",
                  field.name.c_str());
        continue;
      }
      for (int i = 0; i < field.count; i++) {
        std::memcpy(&fieldData[i], &revBuffer_[offset], sizeof(int16_t));
        offset += sizeof(int16_t);
      }
    } else {
      ROS_ERROR("Unsupported rev field type: %s", field.type.c_str());
    }
  }
}

bool SerialMember::openPort() {
  try {
    serial_->setPort(serial_port_);
    serial_->setBaudrate(baudrate_);
    to_ = serial::Timeout::simpleTimeout(timeout_);
    serial_->setTimeout(to_);
    serial_->open();
  } catch (serial::IOException& e) {
    ROS_ERROR("Failed to open serial port: %s", e.what());
    return false;
  }
  if (serial_->isOpen()) {
    ROS_INFO("Serial port opened successfully");
  } else {
    ROS_ERROR("Failed to open serial port");
    return false;
  }
  return true;
}

void SerialMember::closePort() { serial_->close(); }

}  // namespace serial_manager