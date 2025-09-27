/**
 * @file ops9_controller.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-05-13
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note : 可以通过发布话题的方式执行命令
 *         记得要带上：ops9设备名 + 命令
 *         RESET
 *         UPDATE_X
 *         UPDATE_Y
 *         UPDATE_YAW
 *         UPDATE_X_Y
 *         UPDATE_YAW_X_Y
 * @versioninfo :
 */
#include <ops9_controller/Ops9Controller.h>

#include <pluginlib/class_list_macros.hpp>

namespace ops9_controller {

bool Ops9Controller::init(hardware_interface::RobotHW *robot_hw,
                          ros::NodeHandle &root_nh,
                          ros::NodeHandle &controller_nh) {
  XmlRpc::XmlRpcValue xml_rpc_value;
  if (!controller_nh.getParam("name", name_)) {
    ROS_ERROR("Ops9Controller: name not set");
    return false;
  }
  if (!controller_nh.getParam("if_pub_data", if_pub_data_)) {
    ROS_ERROR("Ops9Controller: if_pub_data not set");
    return false;
  }
  if (!controller_nh.getParam("pub_topic", pub_topic_)) {
    ROS_ERROR("Ops9Controller: pub_topic not set");
    return false;
  }
  if (!controller_nh.getParam("ctrl_topic", ctrl_topic_)) {
    ROS_ERROR("Ops9Controller: ctrl_topic not set");
    return false;
  }
  ops9_cmd_sub_ = controller_nh.subscribe<ros_ecat_msgs::Ops9Command>(
      ctrl_topic_, 1, &Ops9Controller::ops9CmdCallback, this);
  if (if_pub_data_) {
    ops9_data_publisher_ = std::make_shared<
        any_node::ThreadedPublisher<ros_ecat_msgs::ActionData>>(
        controller_nh.advertise<ros_ecat_msgs::ActionData>(pub_topic_, 1), 50);
    ROS_WARN_STREAM("ops9: " << name_ << " data publisher is created!");
  }
  ops9_handle_ = robot_hw->get<common::Ops9Interface>()->getHandle(name_);

  return true;
}

void Ops9Controller::update(const ros::Time &time,
                            const ros::Duration &period) {}

void Ops9Controller::ops9CmdCallback(
    const ros_ecat_msgs::Ops9Command::ConstPtr &msg) {
  auto ops9_cmd = *msg;
  if (ops9_cmd.name != name_) {
    return;
  }
  if (ops9_cmd.cmd == "RESET") {
    // 处理RESET命令
    if (ops9_handle_.setResetState())
      ROS_INFO_STREAM("ops9: " << name_ << " set the reset state command!");
  } else if (ops9_cmd.cmd == "UPDATE_X") {
    // 处理UPDATE_X命令
    if (ops9_handle_.updatePosX(ops9_cmd.update_x))
      ROS_INFO_STREAM("ops9: " << name_ << " set the update x:"
                               << ops9_cmd.update_x << " command!");
  } else if (ops9_cmd.cmd == "UPDATE_Y") {
    // 处理UPDATE_Y命令
    if (ops9_handle_.updatePosY(ops9_cmd.update_y))
      ROS_INFO_STREAM("ops9: " << name_ << " set the update y:"
                               << ops9_cmd.update_y << " command!");
  } else if (ops9_cmd.cmd == "UPDATE_YAW") {
    // 处理UPDATE_YAW命令
    if (ops9_handle_.updateYawAngle(ops9_cmd.update_yaw))
      ROS_INFO_STREAM("ops9: " << name_ << " set the update yaw:"
                               << ops9_cmd.update_yaw << " command!");
  } else if (ops9_cmd.cmd == "UPDATE_X_Y") {
    // 处理UPDATE_XY命令
    if (ops9_handle_.updatePosXY(ops9_cmd.update_x, ops9_cmd.update_y))
      ROS_INFO_STREAM("ops9: " << name_ << " set the update x and y:"
                               << ops9_cmd.update_x << ", " << ops9_cmd.update_y
                               << " command!");
  } else if (ops9_cmd.cmd == "UPDATE_YAW_X_Y") {
    // 处理UPDATE_YAW_X_Y命令
    if (ops9_handle_.updateYawPosXY(ops9_cmd.update_yaw, ops9_cmd.update_x,
                                    ops9_cmd.update_y))
      ROS_INFO_STREAM("ops9: " << name_ << " set the update yaw and x and y:"
                               << ops9_cmd.update_yaw << ", "
                               << ops9_cmd.update_x << ", " << ops9_cmd.update_y
                               << " command!");
  }
}

}  // namespace ops9_controller
PLUGINLIB_EXPORT_CLASS(ops9_controller::Ops9Controller,
                       controller_interface::ControllerBase)
