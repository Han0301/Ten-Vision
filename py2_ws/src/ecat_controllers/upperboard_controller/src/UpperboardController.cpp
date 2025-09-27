/**
 * @file UpperboardController.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-05-17
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note : 第一个uint shoot
 *         第二个go的 默认是0，1是收起来，2是立起来 3 是减，4是增
 *
 * @versioninfo :
 */
#include <XmlRpc.h>
#include <upperboard_controller/UpperboardController.h>

#include <pluginlib/class_list_macros.hpp>

namespace upperboard_controller {

bool UpperboardController::init(hardware_interface::RobotHW* robot_hw,
                                ros::NodeHandle& root_nh,
                                ros::NodeHandle& controller_nh) {
  XmlRpc::XmlRpcValue xml_rpc_value;
  if (controller_nh.getParam("name", xml_rpc_value) == false) {
    ROS_ERROR("UpperboardController: name not set");
    return false;
  }
  name_ = static_cast<std::string>(xml_rpc_value);
  if (controller_nh.getParam("ctrl_topic", xml_rpc_value) == false) {
    ROS_ERROR("UpperboardController: ctrl_topic not set");
    return false;
  }
  upperboard_cmd_topic_ = static_cast<std::string>(xml_rpc_value);
  upperboard_cmd_sub_ = root_nh.subscribe<ros_ecat_msgs::UpperBoardCmd>(
      upperboard_cmd_topic_, 1, &UpperboardController::upperboardCmdCallback,
      this);
  // 获取句柄
  serial_to_32_handle_ =
      robot_hw->get<common::SerialTo32Interface>()->getHandle(name_);
  serial_to_32_handle_.setZeroAllData();
  return true;
}

void UpperboardController::update(const ros::Time& time,
                                  const ros::Duration& period) {
  {
    std::lock_guard<std::mutex> lock(upperboard_cmd_mutex_);
    auto msg = upperboard_cmd_buffer_.readFromRT();
    setCommandToBoard(*msg);
  }
}

void UpperboardController::upperboardCmdCallback(
    const ros_ecat_msgs::UpperBoardCmd::ConstPtr& msg) {
  {
    std::lock_guard<std::mutex> lock(upperboard_cmd_mutex_);
    upperboard_cmd_buffer_.writeFromNonRT(*msg);
  }
}

void UpperboardController::setCommandToBoard(
    ros_ecat_msgs::UpperBoardCmd& msg) {
  // 设置发送数据
  uint8_t send_uint_data[4] = {0};
  send_uint_data[0] = msg.shoot;
  if (msg.raiseUp == 1)
    send_uint_data[1] = RAISE_UP_NET;
  else if (msg.putDown == 1)
    send_uint_data[1] = PUT_DOWN_NET;
  else if (msg.go1PoseUp == 1)
    send_uint_data[1] = GO1_POSE_UP;
  else if (msg.go1PoseDown == 1)
    send_uint_data[1] = GO1_POSE_DOWN;
  else
    send_uint_data[1] = 0;
  float send_float_data[6] = {0};
  if (msg.sendShootMsg == 1) {
    send_float_data[0] = 22.3;
  }
  serial_to_32_handle_.setFloatData(send_float_data);
  serial_to_32_handle_.setUint8Data(send_uint_data);
}

}  // namespace upperboard_controller
PLUGINLIB_EXPORT_CLASS(upperboard_controller::UpperboardController,
                       controller_interface::ControllerBase)
