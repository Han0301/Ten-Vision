/**
 * @file SwerveChassisController.h
 * @author py <997276894@qq.com>
 * @brief  舵轮控制器
 * @version 0.1
 * @date  2025-05-24
 *
 * @copyright Copyright (c)  2025
 *
 * @attention
 * @note
 */

#pragma once

#include <XmlRpcValue.h>
#include <controller_interface/multi_interface_controller.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <nav_msgs/Odometry.h>
#include <rc_ecat_master/ecat_hardware_interface/gpio/GpioInterface.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros_ecat_msgs/RemoteState.h>
#include <ros_ecat_msgs/VescEcatRosMsg.h>
#include <ros_ecat_msgs/PositionSensorMsg.h>
#include <ros_ecat_msgs/PositionSensorCmd.h>
#include <ros_ecat_msgs/PboxMsg.h>
#include <ros_ecat_msgs/InitState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <vision_msgs/BoundingBox2DArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <any_node/ThreadedPublisher.hpp>

#include <unordered_map>

#include "ctrl_common/MoveBase/YawAdjust.h"
#include "ctrl_common/PID/IncrementalPID.h"
#include "ctrl_common/PID/PositionPID.h"
#include "ctrl_common/LM.h"

namespace chassis_controller {
class SwerveChassisController
    : public controller_interface::MultiInterfaceController<
          ecat_slave_hw::GpioStateInterface,ecat_slave_hw::GpioCommandInterface,
          hardware_interface::EffortJointInterface> {
 public:
  SwerveChassisController() = default;
  ~SwerveChassisController() = default;

  bool init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh,
            ros::NodeHandle &controller_nh) override;

  void update(const ros::Time &time, const ros::Duration &period) override;

  void SwerveCal();
  std::vector<double> WheelCal();
  void SwerveStateGet();
  void MinorArcOptimzation();
  void SwerveInit();
  void Parking();
  bool Forward();
  void AccelerationPlanning(geometry_msgs::Twist *cmd);
  void CameraOffset(double yaw);
  void BasketOffset(const std::vector<double>& arr_x,const std::vector<double>& arr_y,const std::vector<double>& arr_yaw);
  void calculateCenter();

  void cmdVelCallback(const geometry_msgs::TwistConstPtr &msg);
  void DribbleVelCallback(const geometry_msgs::TwistConstPtr &msg);
  void OdomCallback(const nav_msgs::OdometryConstPtr &msg);
  void initCallback(const std_msgs::String::ConstPtr &msg);
  void debugCallback(const std_msgs::Float32::ConstPtr &msg);
  void vescCallback(const ros_ecat_msgs::VescEcatRosMsg::ConstPtr &msg);
  void imuCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
  void RemoteStateCallback(const ros_ecat_msgs::RemoteStateConstPtr &msg);
  void PartnerCallback(const ros_ecat_msgs::PositionSensorMsgConstPtr &msg);
  void CameraCallback(const vision_msgs::BoundingBox2DArrayConstPtr &msg);
  void PositionCallback(const ros_ecat_msgs::PositionSensorMsgConstPtr &msg);

  void stopping(const ros::Time & /*time*/) override;

  void AngleLimit(double *angle) {
    // if(*angle>M_PI)
    // {
    //     *angle-=2*M_PI;
    //     AngleLimit(angle);
    // }
    // else if(*angle<-M_PI)
    // {
    //     *angle+=2*M_PI;
    //     AngleLimit(angle);
    // }

    while (*angle > M_PI) *angle -= 2.0 * M_PI;
    while (*angle < -M_PI) *angle += 2.0 * M_PI;
  }

 private:
  ros::NodeHandle controller_nh_;
  double triangle_base_;
  double vertex_to_ChassisCenter_L_;
  double base_to_ChassisCenter_B_;
  double wheel_radius_;
  double steer_check_vel_;
  double light_door_skew_;
  std::unordered_map<std::string, hardware_interface::JointHandle>
      wheel_joints_;
  std::unordered_map<std::string, hardware_interface::JointHandle>
      swerve_joints_;
  ecat_slave_hw::GpioCommandHandle lock_RGB_joint_;
  std::unordered_map<std::string, ctrl_common::IncrementalPID>
      swerve_increment_pid_;  // 舵向电机增量式pid
  std::unordered_map<std::string, ctrl_common::PositionPID>
      swerve_position_pid_;  // 舵向电机角度环pid

  std::unordered_map<std::string, ecat_slave_hw::GpioStateHandle> steer_io_;
  ctrl_common::YawAdjust yaw_pid_;
  ctrl_common::PositionPID basket_pid_;
  bool yaw_adjust_flag_ = false;  // 是否开启航向角调整，默认为关闭
  bool is_init_{false};           // 是否初始化过舵轮
  bool is_start_init_{false};      // 是否开始初始化舵轮
  bool parking_lock_{false};      // 停车锁状态，true为锁定，false为解锁
  std::unordered_map<std::string, bool> steer_io_state_;  // 舵轮IO状态缓存

  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber init_sub_;
  ros::Subscriber debug_sub_;
  ros::Subscriber vesc_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber remote_state_sub_;
  ros::Subscriber upperboard_cmd_sub_;
  ros::Subscriber partner_sub_;
  ros::Subscriber camera_sub_;
  ros::Subscriber dribble_vel_sub_;
  ros::Subscriber position_sub_;

  std::mutex cmd_vel_mutex_;
  std::mutex odom_mutex_;
  std::mutex init_mutex_;
  std::mutex imu_mutex_;
  std::mutex partner_mutex_;

  double wheel_reverse_ = 1.0;  // 轮向电机反转系数，正数为正转，负数为反转

  std::unordered_map<std::string, double> wheel_cmd_map_;   // 轮向电机输出
  std::unordered_map<std::string, double> swerve_cmd_map_;  // 舵向电机输出

  std::unordered_map<std::string, double> swerve_pos_cur_;  // 舵向电机当前角度
  std::unordered_map<std::string, double> swerve_vel_cur_;  // 舵向电机当前速度
  std::unordered_map<std::string, double>
      swerve_cmd_pos_;  // 舵向电机期望角度输出
  std::unordered_map<std::string, double>
      wheel_cmd_vel_;  // 舵向电机期望速度输出
  std::unordered_map<std::string, double> swerve_error_map_;

  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> cmd_vel_buffer_;
  realtime_tools::RealtimeBuffer<geometry_msgs::Twist> dribble_vel_buffer_;
  realtime_tools::RealtimeBuffer<ros_ecat_msgs::PositionSensorMsg> position_buffer_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> odom_buffer_;
  realtime_tools::RealtimeBuffer<std_msgs::Float32> debug_buffer_;
  realtime_tools::RealtimeBuffer<geometry_msgs::Vector3Stamped> imu_buffer_;
  realtime_tools::RealtimeBuffer<ros_ecat_msgs::RemoteState> remote_state_buffer_;
  realtime_tools::RealtimeBuffer<ros_ecat_msgs::PositionSensorMsg> partner_buffer_;
  realtime_tools::RealtimeBuffer<vision_msgs::BoundingBox2DArray> camera_buffer_;

  any_node::ThreadedPublisherPtr<ros_ecat_msgs::PboxMsg> pbox_state_pub_;
  any_node::ThreadedPublisherPtr<ros_ecat_msgs::InitState> init_state_pub_;
  any_node::ThreadedPublisherPtr<std_msgs::Float32> debug_pos_pub_;
  any_node::ThreadedPublisherPtr<ros_ecat_msgs::PositionSensorCmd> position_reset_pub_;

  struct steer_joint_pos {
    double last_pos = 0;     // 上次位置
    double current_pos = 0;  // 当前舵轮位置
    double diff_pos = 0;     // 差分位置
    double final_pos = 0;    // 最终位置
  };
  std::unordered_map<std::string, steer_joint_pos>
      swerve_joint_pos_;  // 舵向电机位置缓存

  ctrl_common::YawAdjust yaw_adjust_;  // yaw轴调整pid

  //加速度规划部分
  struct cmd_vel_data {
    double linear_x = 0.0;  // 线速度
    double linear_y = 0.0;  // 侧向速度
    double angular_z = 0.0; // 航向角速度
  };
  double last_cmd_time_;  // 上次命令时间戳
  cmd_vel_data cmd_vel_;
  cmd_vel_data last_speed_;  // 上次命令速度数据
  double start_max_ = 0.5;  // 最大加速度
  double brake_max_ = 0.5;       // 最大减速度
  double forward_acceleration_ = 0.5;  // 起步加速度
  double reverse_acceleration_ = 0.5;   // 刹车加速度
  double x_acceleration_;
  double y_acceleration_;

  bool camera_init_= {false};
  bool forward_flag = {false};

  bool camera_offset_finish_ = {false};
  bool basket_offset_finish_ = {false};
  bool last_remote_set_ = {false};
  bool last_finish_set_ = {false};
  double camera_offset_,basket_x_,basket_y_;
  bool imu_yaw_init = {false} , radar_init_={false} ;
  uint8_t imu_init_count_,radar_init_count_,radar_pos_count_;
  double by_,dx_,dy_,radar_x_,radar_y_,imu_yaw_;
  std::vector<double> basket_arr_x_;
  std::vector<double> basket_arr_y_;
  std::vector<double> basket_arr_yaw_;
  std::vector<double> basket_position_x_;
  std::vector<double> basket_position_y_;
  ros_ecat_msgs::InitState init_state_msg_;
  ros_ecat_msgs::PboxMsg pbox_data_;

  std::unique_ptr<LMOptimizer> optiimizer_;
  double initial_guessed_x_ , initial_guessed_y_;
  std::vector<Measurement> measurements_basket_;
  Params initialParams;
  Params optimized_;
};
}  // namespace chassis_controller