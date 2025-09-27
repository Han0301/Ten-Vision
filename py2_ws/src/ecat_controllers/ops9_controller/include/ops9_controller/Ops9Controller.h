/**
 * @file Ops9Controller.h
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-05-13
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once
#include <XmlRpcValue.h>
#include <controller_interface/controller.h>
#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/TransformStamped.h>
#include <rc_ecat_master/common/hardware_interface/Ops9Interface.h>
#include <ros/ros.h>
#include <ros_ecat_msgs/ActionData.h>
#include <ros_ecat_msgs/Ops9Command.h>

#include <any_node/ThreadedPublisher.hpp>

namespace ops9_controller {

class Ops9Controller : public controller_interface::MultiInterfaceController<
                           common::Ops9Interface> {
 public:
  Ops9Controller() = default;
  ~Ops9Controller() override = default;
  /**
   * @brief 读取配置，初始化订阅者和发布者（如果有的话）
   *
   * @param robot_hw
   * @param root_nh
   * @param controller_nh
   * @return true
   * @return false
   */
  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh,
            ros::NodeHandle &controller_nh) override;
  void update(const ros::Time &time, const ros::Duration &period) override;

  void ops9CmdCallback(const ros_ecat_msgs::Ops9Command::ConstPtr &msg);

 private:
  std::string name_;
  bool if_pub_data_;
  std::string pub_topic_;
  std::string ctrl_topic_;
  ros::Subscriber ops9_cmd_sub_;
  any_node::ThreadedPublisherPtr<ros_ecat_msgs::ActionData>
      ops9_data_publisher_;
  /* ros-control */
  common::Ops9Handle ops9_handle_;
};
}  // namespace ops9_controller