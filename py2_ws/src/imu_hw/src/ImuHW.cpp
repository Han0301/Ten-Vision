/**
 * @file ImuHW.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-08-06
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note : 使用差分读取计算角度
 * @versioninfo :
 */
#include <XmlRpc.h>
#include <imu_hw/ImuHW.h>


namespace imu_hw {

Imu_HW::Imu_HW(ros::NodeHandle& nh) : nh_(nh) {}

Imu_HW::~Imu_HW() {
  close(serial_fd_);
  pubWorker_->stop();
}

bool Imu_HW::init() {
  XmlRpc::XmlRpcValue config;
  if (!nh_.getParam("imu_hw", config)) {
    ROS_ERROR("Failed to get param: imu_hw");
    return false;
  }
  if (!config.hasMember("serial_port")) {
    ROS_ERROR("no param to set [serial_port]");
    return false;
  }
  if (!config.hasMember("baudrate")) {
    ROS_ERROR("no param to set [baudrate]");
    return false;
  }
  if (!config.hasMember("pub_worker_cycle")) {
    ROS_ERROR("no param to set [pub_worker_cycle]");
    return false;
  }
  baudrate_ = static_cast<int>(config["baudrate"]);
  serial_port_ = static_cast<std::string>(config["serial_port"]);
  serial_fd_ = -1;
  pub_worker_cycle_ = static_cast<double>(config["pub_worker_cycle"]);

  // serial relative
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ == -1) {
    ROS_ERROR("Open port Failed!");
    return false;
  } else {
    ROS_INFO("Open port Successfully!");
  }
  struct termios options;  // 串口配置结构体
  tcgetattr(serial_fd_, &options);
  options.c_cflag |= CLOCAL;  // 忽略调制解调器状态行
  options.c_cflag |= CREAD;   // 启用接收器
  options.c_cflag &= ~CSIZE;  // 字符长度掩码。取值为CS5, CS6, CS7或CS8
  options.c_cflag |= CS8;     // 8位数据位
  options.c_cflag &= ~PARENB;      // 校验位
  options.c_cflag &= ~CSTOPB;      // 停止位
  options.c_iflag |= IGNPAR;       // 忽略帧错误和奇偶校验错
  options.c_oflag = 0;             // 输出模式
  options.c_lflag = 0;             // 不激活终端模式
  options.c_cc[VTIME] = 0;         // 读取一个字符等待1*(1/10)s
  options.c_cc[VMIN] = 1;          // 读取字符的最少个数为1
  cfsetospeed(&options, B115200);  // 设置波特率为115200
  cfsetispeed(&options, B115200);
  tcflush(serial_fd_, TCIFLUSH);  // 清空输入缓存区
  if (tcsetattr(serial_fd_, TCSANOW, &options) !=
      0)  // TCSANOW：不等数据传输完毕就立即改变属性
  {
    std::cout << "串口设置失败！" << std::endl;
    return false;
  } else {
    std::cout << "串口设置成功！" << std::endl;
  }
  // 打开串口

  // 注册线程
  pubWorker_ = std::make_shared<any_worker::Worker>(
      "pubWorker", pub_worker_cycle_,
      std::bind(&Imu_HW::pubWorkerCb, this, std::placeholders::_1));


  // 注册发布者
  yaw_msg_publisher_ = std::make_shared<
      any_node::ThreadedPublisher<ros_ecat_msgs::PositionSensorMsg>>(
      nh_.advertise<ros_ecat_msgs::PositionSensorMsg>("/imu_data", 10),50,true);
  raw_yaw_publisher_ = std::make_shared<any_node::ThreadedPublisher<std_msgs::Float32>>(nh_.advertise<std_msgs::Float32>("/imu_raw_data",10),50,true);


  // 订阅命令
  cmd_sub_ = nh_.subscribe<ros_ecat_msgs::PositionSensorCmd>(
      "/imu_cmd", 1, &Imu_HW::ImuCmdCallback, this);

  // pubWorker_->start();
  return true;
}

bool Imu_HW::pubWorkerCb(const any_worker::WorkerEvent& event) {
  yaw_msg_.yaw_angle = final_data_;
  yaw_msg_publisher_->publish(yaw_msg_);
  return true;
}

// 返回-1 表示串口掉线
int8_t Imu_HW::unpack() {
  // read(serial_fd_, buffer, 1);
  // if (buffer[0] == 0x55) {
  //   read(serial_fd_, buffer + 1, 1);
  //   if (buffer[1] == 0x51) {
  //     read(serial_fd_, ax, 9);
  //   } else if (buffer[1] == 0x52) {
  //     read(serial_fd_, gx, 9);
  //   } else if (buffer[1] == 0x53) {
  //     read(serial_fd_, sAngle, 9);
  //   }
  //   // if(read(serial_fd_,sAngle,))
  // }

  // clear the buffer
  // tcflush(serial_fd_, TCIFLUSH);
  // 1. 读帧头，循环直到 0x55 或错误
  uint8_t hdr = 0;
  while (true) {
    ssize_t n = ::read(serial_fd_, &hdr, 1);
    if (n <= 0) {
      ROS_ERROR_THROTTLE(5, "串口读取失败或断开: %s", strerror(errno));
      return -1;
    }
    if (hdr == 0x55) break;
  }

  // 2. 一次读类型 + 6B 数据 + 2B CRC/保留 = 9 字节
  uint8_t packet[9];
  if (::read(serial_fd_, packet, sizeof(packet)) != sizeof(packet)) {
    return 1;
  }

  uint8_t type = packet[0];
  if (type != 0x53) {
    // 不是姿态帧，丢弃
    return 1;
  }

  // 3. 解析出 sAngle[0..2]
  for (int i = 0; i < 3; ++i) {
    sAngle[i] =
        static_cast<int16_t>(packet[1 + 2 * i] | (packet[2 + 2 * i] << 8));
  }
  // 4. 打印 yaw
  // ROS_INFO_STREAM_THROTTLE(0.005, "Yaw: " << sAngle[2] / 32768.0 * M_PI);
  return 0;
}

// 返回-1,表示失败;返回0，表示重连成功，继续接收完成任务
int8_t Imu_HW::recoverPort(void) {
  close(serial_fd_);
  serial_fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd_ < 0) {
    ROS_ERROR("recover error: %s", strerror(errno));
    return -1;
  }
  // 重新配置 termios（与 init 中保持一致）
  struct termios options;
  tcgetattr(serial_fd_, &options);
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_iflag |= IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;
  options.c_cc[VTIME] = 0;
  options.c_cc[VMIN] = 1;
  cfsetospeed(&options, B115200);
  cfsetispeed(&options, B115200);
  tcflush(serial_fd_, TCIFLUSH);
  if (tcsetattr(serial_fd_, TCSANOW, &options) != 0) {
    ROS_ERROR("recover tcsetattr failed: %s", strerror(errno));
    return -1;
  }
  ROS_INFO("串口重连并配置成功");
  return 0;
}

void Imu_HW::getYawData() {
  {
    std::lock_guard<std::mutex> lock(yaw_data_mutex_);
    float raw = sAngle[2] / 32768.0f * M_PI;
    if (!if_first_get_) {
      if_first_get_ = true;
      last_data_ = raw;
      final_data_ = 0.0f;
    }
    // if(abs(raw - 0.0f)  < 0.02){
    //   raw = last_data_;
    // }

    // 计算差值并处理跨越 ±π 的跳变
    float delta = raw - last_data_;
    if (delta > 2*M_PI) delta -= 2.0f * M_PI;
    if (delta < -2*M_PI) delta += 2.0f * M_PI;
    if(abs(delta / 0.002) > 4.5){
      delta = 0;
    }

    final_data_ += delta;
    if (final_data_ > M_PI) final_data_ -= 2.0 * M_PI;
    if (final_data_ < -M_PI) final_data_ += 2.0 * M_PI;
    last_data_ = raw;
    yaw_msg_.yaw_angle = final_data_;
    yaw_msg_publisher_->publish(yaw_msg_);

    raw_msg_.data = raw;
    raw_yaw_publisher_->publish(raw_msg_); 

  }
}

void Imu_HW::ImuCmdCallback(
    const ros_ecat_msgs::PositionSensorCmd::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> lock(yaw_data_mutex_);
    if (msg.get()->cmd == "RESET") {
      final_data_ = 0;
      delta_data_ = 0;
      cur_data_ = 0;
      last_data_ = 0;
      if_first_get_ = false;
    }
  }
}

}  // namespace imu_hw