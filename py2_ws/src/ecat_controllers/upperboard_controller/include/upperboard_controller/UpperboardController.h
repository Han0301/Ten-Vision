/**
 * @file UpperboardController.h
 * @author Keten (2863861004@qq.com)
 * @brief 上层板控制器
 * @version 0.1
 * @date 2025-05-17
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note : 将负责接受命令，进行数据的发送
 *         接收串口数据打包的信息
 * @versioninfo :
 */
#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <rc_ecat_master/common/hardware_interface/SerialTo32Interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>
#include <ros_ecat_msgs/UpperBoardCmd.h>

#include <any_node/ThreadedPublisher.hpp>
#include <mutex>

namespace upperboard_controller {

#define RAISE_UP_NET 1
#define PUT_DOWN_NET 2
#define GO1_POSE_DOWN 3
#define GO1_POSE_UP 4

class UpperboardController
    : public controller_interface::MultiInterfaceController<
          common::SerialTo32Interface> {
 public:
  UpperboardController() = default;
  ~UpperboardController() override = default;

  bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& root_nh,
            ros::NodeHandle& controller_nh) override;

  void update(const ros::Time& time, const ros::Duration& period) override;

  void upperboardCmdCallback(const ros_ecat_msgs::UpperBoardCmd::ConstPtr& msg);

  void setCommandToBoard(ros_ecat_msgs::UpperBoardCmd& msg);

 private:
  std::string name_;
  common::SerialTo32Handle serial_to_32_handle_;
  // sub relative
  ros::Subscriber upperboard_cmd_sub_;
  std::string upperboard_cmd_topic_;
  std::mutex upperboard_cmd_mutex_;
  realtime_tools::RealtimeBuffer<ros_ecat_msgs::UpperBoardCmd>
      upperboard_cmd_buffer_;
};
}  // namespace upperboard_controller