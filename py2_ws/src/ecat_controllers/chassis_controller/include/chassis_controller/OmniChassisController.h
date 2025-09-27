/**
 * @file OmniChassisController.h
 * @author Keten (2863861004@qq.com)
 * @brief 四全向轮底盘控制器
 * @version 0.1
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once
#include <XmlRpcValue.h>
#include <control_msgs/JointControllerState.h>
#include <control_toolbox/pid.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include <rc_ecat_master/RcEcatHardwareInterface.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <ros_ecat_msgs/PositionSensorMsg.h>
#include <ros_ecat_msgs/RemoteState.h>

#include "ctrl_common/PID/IncrementalPID.h"
#include "ctrl_common/PID/PositionPID.h"
#include "ctrl_common/MoveBase/YawAdjust.h"


namespace chassis_controller {

/*
  后续可以使用多接口控制器，只要换个继承就行了
*/

class OmniChassisController : public controller_interface::Controller<
                                  hardware_interface::EffortJointInterface> {
 public:
  OmniChassisController() = default;
  ~OmniChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

  void update(const ros::Time &time, const ros::Duration &period) override;

  std::vector<double> MoveJoint();

  double QuaternionYaw();

  double GetYawFromBasket();

  float GetYawFromYolo();

  void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg);

  void OdomCallback(const nav_msgs::OdometryConstPtr &msg);
  void PositionCallback(const ros_ecat_msgs::PositionSensorMsgConstPtr &msg);
  void RemoteStateCallback(const ros_ecat_msgs::RemoteStateConstPtr &msg);
  void YoloCallback(const std_msgs::Float32ConstPtr &msg);

  void lfVelCallback(const std_msgs::Float64ConstPtr &msg);
  void rfVelCallback(const std_msgs::Float64ConstPtr &msg);
  void lbVelCallback(const std_msgs::Float64ConstPtr &msg);
  void rbVelCallback(const std_msgs::Float64ConstPtr &msg);

  /**
   * 调参阶段用于获取动态更改的参数
   */
  void UpdateParameters();

  geometry_msgs::Twist forwardKinematics(void);

 private:
  ros::NodeHandle controller_nh_;
  std::unordered_map<std::string,hardware_interface::JointHandle> joints_;
  double chassis_radius_;
  double wheel_radius_;
  ros::Subscriber cmd_vel_sub_;
  std::unordered_map<std::string, double> wheel_cmd_map_;
  std::unordered_map<std::string, double> wheel_vel_map_;
  geometry_msgs::Twist cmd_vel_;
  std::string command_source_frame_;
  std::mutex cmd_vel_mutex_;
  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_vel_buffer_;
  std::unordered_map<std::string, ctrl_common::IncrementalPID> wheel_pid_map_;

  bool yaw_adjust_flag_ = false;//是否启用航向角调整，默认为关
  double PI = 3.1415926;
  ctrl_common::YawAdjust yaw_adjust_; //yaw轴调整pid
  ctrl_common::PositionPID yolo_adjust_;
  ros::Subscriber odom_sub_;//获取当前机器人坐标及姿态
  ros::Subscriber remote_state_sub_;//获取遥控器状态
  ros::Subscriber position_sub_;
  ros::Subscriber yolo_sub_;

  double target_yaw_ = 0.0;//目标航向角，因暂时无规划器，现先直接赋值调试
  nav_msgs::Odometry odom_;
  ros_ecat_msgs::ActionData action_;
  ros_ecat_msgs::RemoteState remote_state_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> odom_buffer_;
  realtime_tools::RealtimeBuffer<ros_ecat_msgs::PositionSensorMsg> position_buffer_;
  realtime_tools::RealtimeBuffer<std_msgs::Float64> left_front_buffer_;
  realtime_tools::RealtimeBuffer<ros_ecat_msgs::RemoteState> remote_state_buffer_;
  realtime_tools::RealtimeBuffer<std_msgs::Float32> yolo_offset_buffer_;

  ros::Time yolo_time_stamp_;

  std::mutex odom_mutex_;
  double yaw_adjust_value_;

  //To Debug
  ros::Subscriber lf_cmd_vel_sub_;
  ros::Subscriber rf_cmd_vel_sub_;
  ros::Subscriber lb_cmd_vel_sub_;
  ros::Subscriber rb_cmd_vel_sub_;
  double lf_cmd_debug;
  // double joint_cmd[4];
};

}  // namespace chassis_controller