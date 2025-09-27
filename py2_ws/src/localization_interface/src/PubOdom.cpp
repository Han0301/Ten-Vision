/**
 * @file PubOdom.cpp
 * @author Keten (2863861004@qq.com) Jelly
 * @brief
 * @version 0.2
 * @date 2025-06-13
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note : 静态变换获取一次之后可以不用再重复获取！
 *         坐标变换框架：
 *         1. action系转base_link系<-no use
 *         2. 雷达系转base_link系
 *        TODO：only pub odom which has chassis_center coordinate
 *              
 *
 *
 * @versioninfo :
 */
#include <localization_interface/PubOdom.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace localization_interface {

OdomPublisher::OdomPublisher(ros::NodeHandle& nh)
    : nh_(nh), tf_listener_(tf_buffer_) {
  /* read the yaml to config */
  XmlRpc::XmlRpcValue xml_rpc_value;
  if (nh_.getParam("radar", xml_rpc_value)) {
    radar_type_ = static_cast<std::string>(xml_rpc_value["name"]);
    radar_topic_name_ = static_cast<std::string>(xml_rpc_value["topic_name"]);
    radar_offset_.x_offset = static_cast<double>(xml_rpc_value["x_offset"]);
    radar_offset_.y_offset = static_cast<double>(xml_rpc_value["y_offset"]);
    radar_frame_ = static_cast<std::string>(xml_rpc_value["frame_id"]);
  }
  if (nh_.getParam("wheel_odometer", xml_rpc_value)) {
    wheel_odometer_type_ = static_cast<std::string>(xml_rpc_value["name"]);
    action_topic_name_ = static_cast<std::string>(xml_rpc_value["topic_name"]);
    action_offset_.x_offset = static_cast<double>(xml_rpc_value["x_offset"]);
    action_offset_.y_offset = static_cast<double>(xml_rpc_value["y_offset"]);
    action_frame_ = static_cast<std::string>(xml_rpc_value["frame_id"]);
  }
  if (nh_.getParam("odom", xml_rpc_value)) {
    odom_topic_name_ = static_cast<std::string>(xml_rpc_value["topic_name"]);
    odom_offset_.x_offset = static_cast<double>(xml_rpc_value["x_offset"]);
    odom_offset_.y_offset = static_cast<double>(xml_rpc_value["y_offset"]);
    if_chassis_center_ = static_cast<bool>(xml_rpc_value["if_chassis_center"]);
    pub_cycle_ = static_cast<double>(xml_rpc_value["pub_cycle_"]);
  }

  /* then pub the static transform */
  publishStaticTransform();

  /* pub init */
  odom_publisher_ =
      std::make_shared<any_node::ThreadedPublisher<nav_msgs::Odometry>>(
          nh_.advertise<nav_msgs::Odometry>("odom", 10), 50, true);
  // update_worker_ = std::make_shared<any_worker::Worker>(
      // "update_odom", pub_cycle_,
      // std::bind(&OdomPublisher::updateOdomCb, this, std::placeholders::_1));

  /* sub init */
  radar_subscriber_ =
      nh_.subscribe(radar_topic_name_, 10, &OdomPublisher::radarCallback, this);
  action_subscriber_ = nh_.subscribe(action_topic_name_, 10,
                                     &OdomPublisher::actionCallback, this);

  /* bind the ending function to signal handler */
  signal_handler::SignalHandler::bindAll(&OdomPublisher::handleSignal, this);

  /* 设置action传感器的frame_id */
  real_twist_.header.frame_id = action_frame_;
  real_pos_.header.frame_id = action_frame_;
  /* 设置odom的frame_id 及 子frame_id */
  odom_.header.frame_id = "odom";
  odom_.child_frame_id = "base_link";
  // 首次发布前初始化
  odom_.pose.pose.position.x = 0.0;
  odom_.pose.pose.position.y = 0.0;
  odom_.pose.pose.position.z = 0.0;
  odom_.pose.pose.orientation.w = 1.0;  // 单位四元数

  // 同时确保 real_base_link_pos_ 被初始化为合法值
  real_base_link_pos_.pose.orientation = odom_.pose.pose.orientation;
  base_link_pos_.pose.orientation = odom_.pose.pose.orientation;
  /* pub the dynamic transform first */
  publishDynamicTransform();
  
  /* start the data handle function */
  // update_worker_->start();
}

// bool OdomPublisher::updateOdomCb(const any_worker::WorkerEvent& event) {
bool OdomPublisher::updateOdomCb() {
  // 进行动态变换
  publishDynamicTransform();
  // 获取当前时间
  ros::Time current_time = ros::Time::now();
  
  // 获取雷达在map坐标系下的位姿
 
  {
    std::lock_guard<std::mutex> lock(real_pos_mutex_);
    base_link_in_radar.pose.position.x = radar_offset_.x_offset;  // 这是base_link在radar下的位姿
    base_link_in_radar.pose.position.y = radar_offset_.y_offset;
    base_link_in_radar.header.frame_id = "base_link";
    base_link_in_radar.header.stamp = current_time;  // 更新时间戳
  }

  try {
    
    // 第1步：转换base_link位姿到map坐标系
    tf2::doTransform(base_link_in_radar, base_link_in_map, 
                    tf_buffer_.lookupTransform("map",radar_frame_, ros::Time(0)));

    // 第2步：转换base_link在map下的位姿到odom坐标系
    tf2::doTransform(base_link_in_map, base_link_pos_, 
                    tf_buffer_.lookupTransform("odom", "map", ros::Time(0)));

    // // 更新存储的base_link位姿
    // base_link_pos_ = base_link_in_odom;

  } catch (tf2::TransformException& ex) {
    ROS_WARN_STREAM("TF转换失败: " << ex.what());
    // 使用上一次的有效数据（可选）
  }
  
  
  // 准备ODom消息
  odom_.header.stamp = current_time;
  odom_.pose.pose = base_link_pos_.pose;       // base_link在odom下的位姿
  odom_.twist.twist = base_link_twist_.twist;  // 速度数据
  /* 发布odom话题 */
  odom_publisher_->publish(odom_);

  return true;
}

void OdomPublisher::handleSignal(int /* signum */) {
  update_worker_->stop();
  odom_publisher_->shutdown();
  ros::shutdown();
}

void OdomPublisher::radarCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  // 直接复制雷达传感器数据（雷达坐标系与 base_link 一致）
  geometry_msgs::PoseStamped radar_data = *msg;

  // 如有配置偏置，则对位置进行修正（可选）
  // radar_data.pose.position.x -= radar_offset_.x_offset;
  // radar_data.pose.position.y -= radar_offset_.y_offset;

  {
    std::lock_guard<std::mutex> lock(real_pos_mutex_);
    real_base_link_pos_.header.stamp = radar_data.header.stamp;
    real_base_link_pos_.pose.position = radar_data.pose.position;
    real_base_link_pos_.pose.orientation = radar_data.pose.orientation;
  }
  updateOdomCb();
}


void OdomPublisher::actionCallback(
    const ros_ecat_msgs::ActionData::ConstPtr& msg) {
  /* calculate the dt */
  double dt =
      (now_action_data_.header.stamp - last_action_data_.header.stamp).toSec();
  if (dt < 0.0001) {
    dt = 1e-6;  // 防止除以0
  }
  // 先将action的data进行装填
  geometry_msgs::PoseStamped input;
  input.header.stamp = msg->header.stamp;
  input.header.frame_id = action_frame_;
  input.pose.position.x = msg->pose_x;
  input.pose.position.y = msg->pose_y;
  input.pose.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, msg->yaw_angle);
  q.normalize();
  // 将构造的四元数赋值到 orientation 中
  input.pose.orientation.x = q.x();
  input.pose.orientation.y = q.y();
  input.pose.orientation.z = q.z();
  input.pose.orientation.w = q.w();
  geometry_msgs::PoseStamped output;
  try {
    tf_buffer_.transform(input, output, "map",
                         ros::Duration(0.1));  // 允许100ms时间容差
  } catch (tf2::TransformException& ex) {
    ROS_WARN("tf transform failed! : %s", ex.what());
  }
  geometry_msgs::PoseStamped converted = output;
  double old_x = output.pose.position.x;
  double old_y = output.pose.position.y;
  // 这里并非是处理偏置的地方，只是说由于静态变换+ops9发布数据的关系
  converted.pose.position.x = -old_y;  // 新 x = - 原 y
  converted.pose.position.y = old_x - action_offset_.x_offset;  // 新 y = 原 x
  {
    std::lock_guard<std::mutex> lock(real_pos_mutex_);
    real_base_link_pos_.header.stamp = converted.header.stamp;
    real_base_link_pos_.pose.position = converted.pose.position;
    real_base_link_pos_.pose.orientation = converted.pose.orientation;
  }
  {
    //
  }
}

void OdomPublisher::publishStaticTransform() {
  /* static transform for odom to map 发布odom到map的坐标变换 */
  tf2::Quaternion quat;

  geometry_msgs::TransformStamped odom_transform;
  odom_transform.header.stamp = ros::Time::now();
  odom_transform.header.frame_id = "map";  // 源坐标系
  odom_transform.child_frame_id = "odom";  // 目标坐标系
  odom_transform.transform.translation.x = odom_offset_.x_offset;
  odom_transform.transform.translation.y = odom_offset_.y_offset;
  quat.setRPY(0, 0, 0);
  odom_transform.transform.rotation.x = quat.x();
  odom_transform.transform.rotation.y = quat.y();
  odom_transform.transform.rotation.z = quat.z();
  odom_transform.transform.rotation.w = quat.w();
  odom_static_broadcaster_.sendTransform(odom_transform);

  geometry_msgs::TransformStamped radar_static_transform;
  radar_static_transform.header.stamp = ros::Time::now();
  radar_static_transform.header.frame_id = radar_frame_;
  radar_static_transform.child_frame_id = "base_link";
  radar_static_transform.transform.translation.x = radar_offset_.x_offset;  // 0.35米
  radar_static_transform.transform.translation.y = radar_offset_.y_offset;
  radar_static_transform.transform.translation.z = 0.0;
  radar_static_transform.transform.rotation.w = 1.0;  // 无旋转
  odom_static_broadcaster_.sendTransform(radar_static_transform);
}

void OdomPublisher::publishDynamicTransform() {
  // 构造动态变换消息，利用最新的 odom 数据发布
  geometry_msgs::TransformStamped dynamic_transform;
  dynamic_transform.header.stamp = ros::Time::now();
  dynamic_transform.header.frame_id = "map";
  dynamic_transform.child_frame_id = radar_frame_;
  dynamic_transform.transform.translation.x = real_base_link_pos_.pose.position.x;
  dynamic_transform.transform.translation.y = real_base_link_pos_.pose.position.y;
  dynamic_transform.transform.translation.z = 0.0;
  dynamic_transform.transform.rotation = real_base_link_pos_.pose.orientation;
  // 发布动态变换
  dynamic_broadcaster_.sendTransform(dynamic_transform);
}

}  // namespace localization_interface