/**
 * @file defense_controller.h
 * @author py <997276894@qq.com>
 * @brief  防守机构控制器
 * @version 0.1
 * @date  2025-07-15
 * 
 * @copyright Copyright (c)  2025 
 * 
 * @attention  
 * @note  
 */
#pragma once
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <unordered_map>
#include <control_msgs/JointControllerState.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <mutex>
#include <cmath>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <ros_ecat_msgs/RemoteState.h>

#include "ctrl_common/PID/IncrementalPID.h"


namespace defense_controller{

/*
  后续可以使用多接口控制器，只要换个继承就行了
*/
class DefenseController : public controller_interface::Controller<hardware_interface::EffortJointInterface>
{
public:
    DefenseController() = default;
    ~DefenseController() override = default;
    bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
        ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

    void update(const ros::Time &time, const ros::Duration &period) override;
    void RemoteStateCallback(const ros_ecat_msgs::RemoteStateConstPtr &msg); 
    void stopping(const ros::Time & /*time*/) override;

private:
    ros::NodeHandle controller_nh_;
    ros::NodeHandle root_nh_;

    ros::Subscriber remote_state_sub_;

    realtime_tools::RealtimeBuffer<ros_ecat_msgs::RemoteState> remote_state_buffer_;

    std::unordered_map<std::string,hardware_interface::JointHandle> joints_map_;
    std::unordered_map<std::string,ctrl_common::IncrementalPID> motor_pid_;

    std::mutex remote_mutex_;

    double rise_speed_;
    double keep_speed_;
    double action_time_;
    double cmd_vel_;
    bool action_flag_ = {false};
    bool timing_idle_ = {true}; //计时空闲标志位-> true:计时运动结束 空闲状态 ； flase：正在计时运动
    double action_start_time_;
};
}