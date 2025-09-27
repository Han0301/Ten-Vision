/**
 * @file PositionSensorHW.cpp
 * @author Keten (2863861004@qq.com)
 * @brief Position里程计的硬件接口
 * @version 0.1
 * @date 2025-05-26
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include <position_sensor/PositionSensorHW.h>

namespace position_sensor {

// 定义一个 no-op 删除器
// 其在被调用时将不执行任何删除操作，用它包装原始指针后，智能指针在销毁时将不会释放该指针指向的内存
struct NoOpDeleter {
  template <typename T>
  void operator()(T*) const {}
};

// 初始化智能指针
PositionSensorHW::PositionSensorHW()
    : rev_float_data_(static_cast<float*>(nullptr), NoOpDeleter()),
      send_float_data_(static_cast<float*>(nullptr), NoOpDeleter()),
      send_frame_id_(static_cast<uint8_t*>(nullptr), NoOpDeleter()),
      send_data_length_(static_cast<uint8_t*>(nullptr), NoOpDeleter()) {}

bool PositionSensorHW::init(ros::NodeHandle& nh) {
  nh_ = nh;
  serial_member_ = std::make_shared<serial_manager::SerialMember>(nh_);
  if (!serial_member_->init()) {
    ROS_ERROR_STREAM("SerialMember init failed");
    return false;
  }
  // 将指针挂载到对应数据指针上
  rev_float_data_.reset(
      static_cast<float*>(serial_member_->getFieldDataPtr("float_data", false)),
      NoOpDeleter());
  send_float_data_.reset(
      static_cast<float*>(serial_member_->getFieldDataPtr("float_data", true)),
      NoOpDeleter());
  send_frame_id_.reset(
      static_cast<uint8_t*>(serial_member_->getFieldDataPtr("frame_id", true)),
      NoOpDeleter());
  send_data_length_.reset(static_cast<uint8_t*>(serial_member_->getFieldDataPtr(
                              "data_length", true)),
                          NoOpDeleter());

  /* init the publisher */
  sensordataPublisher_ = std::make_shared<
      any_node::ThreadedPublisher<ros_ecat_msgs::PositionSensorMsg>>(
      nh_.advertise<ros_ecat_msgs::PositionSensorMsg>("position_sensor_data",
                                                      10),
      50, true);

  /* init the subscrber */
  sensorcmdSubscriber_ = nh_.subscribe(
      "position_sensor_cmd", 10, &PositionSensorHW::sensorCmdCallback, this);

  /* worker relative */
  updateWorker_ = std::make_shared<any_worker::Worker>(
      "PositionSensorWorker", 0.001,
      std::bind(&PositionSensorHW::updateWorkerCb, this,
                std::placeholders::_1));

  updateWorker_->start(45);
  return true;
}

// 订阅命令回调函数
void PositionSensorHW::sensorCmdCallback(
    const ros_ecat_msgs::PositionSensorCmd::ConstPtr& msg) {
  // 将接收到的命令数据写入到发送数据指针中
  if (msg->cmd == "RESET") {
    // 发送复位命令
    send_frame_id_.get()[0] = 0x02;     // 假设0x01表示复位命令
    send_data_length_.get()[0] = 0x08;  // 8字节 == 2个float数据
    send_float_data_.get()[0] = 0.00f;
    send_float_data_.get()[1] = 0.00f;
    ROS_INFO_STREAM("PositionSensorHW: Reset command sent.");
  } else if (msg->cmd == "UPDATE_XY") {
    // 更新x、y轴坐标
    send_frame_id_.get()[0] = 0x01;
    send_data_length_.get()[0] = 0x08;  // 8字节 == 2个float数据
    send_float_data_.get()[0] = msg->update_pose_x;
    send_float_data_.get()[1] = msg->update_pose_y;
    ROS_INFO_STREAM("PositionSensorHW: Update XY command sent."
                    << "X set to : " << msg->update_pose_x
                    << ", Y set to : " << msg->update_pose_y);
  }
  serial_member_->write();  // 写入数据到串口
}

bool PositionSensorHW::updateWorkerCb(const any_worker::WorkerEvent& event) {
  ros_ecat_msgs::PositionSensorMsg msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "position_sensor";
  msg.pose_x = rev_float_data_.get()[0];
  msg.pose_y = rev_float_data_.get()[1];
  msg.yaw_angle = rev_float_data_.get()[2];
  msg.linear_x = rev_float_data_.get()[3];
  msg.linear_y = rev_float_data_.get()[4];
  sensordataPublisher_->publish(msg);
  return true;
}

}  // namespace position_sensor