/**
 * @file PubOdom.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-05-09
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note : 静态变换获取一次之后可以不用再重复获取！
 *         坐标变换框架：
 *         1. action系转base_link系
 *         2. 雷达系转base_link系
 *        TODO：将odom系下的坐标差分得到速度
 *              发布ops9所需话题消息进行数据刷新
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
  update_worker_ = std::make_shared<any_worker::Worker>(
      "update_odom", pub_cycle_,
      std::bind(&OdomPublisher::updateOdomCb, this, std::placeholders::_1));

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
  /* pub the dynamic transform first */
  publishDynamicTransform();

  /* start the data handle function */
  update_worker_->start();
}

bool OdomPublisher::updateOdomCb(const any_worker::WorkerEvent& event) {
  // 进行动态变换
  publishDynamicTransform();

  odom_.header.stamp = ros::Time::now();
  odom_.pose.pose = real_base_link_pos_.pose;       // 装填姿态数据
  odom_.twist.twist = real_base_link_twist_.twist;  // 装填速度数据
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
  radar_data.pose.position.x -= radar_offset_.x_offset;
  radar_data.pose.position.y -= radar_offset_.y_offset;

  {
    std::lock_guard<std::mutex> lock(real_pos_mutex_);
    real_base_link_pos_.header.stamp = radar_data.header.stamp;
    real_base_link_pos_.pose.position = radar_data.pose.position;
    real_base_link_pos_.pose.orientation = radar_data.pose.orientation;
  }
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
}

void OdomPublisher::publishDynamicTransform() {
  // 构造动态变换消息，利用最新的 odom 数据发布
  geometry_msgs::TransformStamped dynamic_transform;
  dynamic_transform.header.stamp = ros::Time::now();
  dynamic_transform.header.frame_id = "odom";
  dynamic_transform.child_frame_id = "base_link";
  dynamic_transform.transform.translation.x = odom_.pose.pose.position.x;
  dynamic_transform.transform.translation.y = odom_.pose.pose.position.y;
  dynamic_transform.transform.translation.z = 0.0;
  dynamic_transform.transform.rotation = odom_.pose.pose.orientation;
  // 发布动态变换
  dynamic_broadcaster_.sendTransform(dynamic_transform);
}

}  // namespace localization_interface