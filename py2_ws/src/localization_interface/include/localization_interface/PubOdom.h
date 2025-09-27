/**
 * @file PubOdom.h
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-05-09
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note : action 数据必须换成ros标准
 *         角度范围值域（-pi ~ pi）
 *         右手定则 -- > x轴正向方向同向为0弧度
 *         逆时针旋转度数增加
 * @versioninfo :
 */
#pragma once

#include <XmlRpc.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros_ecat_msgs/ActionData.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <any_node/ThreadedPublisher.hpp>
#include <any_worker/Worker.hpp>
#include <chrono>
#include <mutex>
#include <signal_handler/SignalHandler.hpp>
#include <string>

namespace localization_interface {

class OdomPublisher {
 public:
  explicit OdomPublisher(ros::NodeHandle& nh);
  ~OdomPublisher() = default;

  bool updateOdomCb();

  void handleSignal(int /* signum */);

  void radarCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  void actionCallback(const ros_ecat_msgs::ActionData::ConstPtr& msg);

 private:
  void publishStaticTransform();

  void publishDynamicTransform();

  std::string radar_type_;
  std::string wheel_odometer_type_;
  std::string radar_topic_name_;
  std::string action_topic_name_;
  std::string radar_frame_;
  std::string action_frame_;
  std::string odom_topic_name_;
  struct offset {
    double x_offset;
    double y_offset;
  };
  offset radar_offset_;
  offset action_offset_;
  offset odom_offset_;

  /* ros-relative */
  ros::NodeHandle& nh_;

  /* tf2 transform */
  tf2_ros::StaticTransformBroadcaster radar_static_broadcaster_;
  tf2_ros::StaticTransformBroadcaster action_static_broadcaster_;
  tf2_ros::StaticTransformBroadcaster odom_static_broadcaster_;
  bool if_chassis_center_ = false;
  tf2_ros::TransformBroadcaster dynamic_broadcaster_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  /* publisher */
  nav_msgs::Odometry odom_;
  any_node::ThreadedPublisherPtr<nav_msgs::Odometry> odom_publisher_;
  std::shared_ptr<any_worker::Worker> update_worker_;
  double pub_cycle_ = 0.01;

  /* subscriber */
  ros::Subscriber radar_subscriber_;
  ros::Subscriber action_subscriber_;

  /* for wheel_odometry */
  struct odometry_data {
    std_msgs::Header header;
    double x;
    double y;
    double yaw_angle;
  };
  odometry_data last_action_data_{};   // 上次的原始数据
  odometry_data delta_action_data_{};  // 差分运算的过程值
  odometry_data now_action_data_{};    // 本次的action数据值

  geometry_msgs::PoseStamped real_pos_;     // action系的结果
  geometry_msgs::TwistStamped real_twist_;  // action系的速度

  geometry_msgs::PoseStamped real_radar_pos_;  // 雷达系的结果

  geometry_msgs::PoseStamped real_base_link_pos_;     // 真正写入base_link
  geometry_msgs::PoseStamped base_link_pos_;
  geometry_msgs::PoseStamped base_link_in_map;
   geometry_msgs::PoseStamped base_link_in_radar;
  geometry_msgs::TwistStamped real_base_link_twist_;  //
  geometry_msgs::TwistStamped base_link_twist_;  //

  double yaw_angle_ = 0;

  std::mutex real_pos_mutex_;
  std::mutex real_twist_mutex_;

  tf2::Matrix3x3 action_rotation_matrix_;
  tf2::Vector3 action_translation_;
  tf2::Matrix3x3 radar_rotation_matrix_;
  tf2::Vector3 radar_translation_;
  bool static_transform_initialized_ = false;  // 静态发布是否已经初始化
};
}  // namespace localization_interface