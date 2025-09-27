/**
 * @file upperboard_app_node.cpp
 * @author Keten (2863861004@qq.com)
 * @brief 取代upperboard 控制器，直接外挂到此进程中
 * @version 0.1
 * @date 2025-05-18
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>
#include <ros_ecat_msgs/UpperBoardCmd.h>
#include <ros_ecat_msgs/PositionSensorMsg.h>
#include <nav_msgs/Odometry.h>
#include <serial_manager/SerialManager.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>

#include <any_worker/Worker.hpp>
#include <memory>
#include <cmath>

#define RAISE_UP_NET 1
#define PUT_DOWN_NET 2
#define GO1_POSE_DOWN 3
#define GO1_POSE_UP 4

/* global variables */
std::shared_ptr<serial_manager::SerialMember> serial_member;
float* float_data = nullptr;
uint8_t* uint8_t_data = nullptr;

float* send_float_data = nullptr;
uint8_t* send_uint8_t_data = nullptr;

float shoot_cmd = 0.f;
float basket_distance_ = 0.0f;

void UpperboardCmdCallback(const ros_ecat_msgs::UpperBoardCmd::ConstPtr& msg) {
  // 处理上层板命令
  send_uint8_t_data[0] = msg->shoot;
  if (msg->raiseUp == 1) {
    send_uint8_t_data[1] = RAISE_UP_NET;
  } else if (msg->putDown == 1) {
    send_uint8_t_data[1] = PUT_DOWN_NET;
  } else if (msg->go1PoseUp == 1) {
    send_uint8_t_data[1] = GO1_POSE_UP;
  } else if (msg->go1PoseDown == 1) {
    send_uint8_t_data[1] = GO1_POSE_DOWN;
  } else {
    send_uint8_t_data[1] = 0;
  }
  if (msg->sendShootMsg == 1)
    send_float_data[0] = shoot_cmd;
  else
    send_float_data[0] = 0.f;
  ROS_INFO_STREAM_THROTTLE(
      0.5,
      "UpperboardCmdCallback: "
          << "send_uint8_t_data[0] = " << static_cast<int>(send_uint8_t_data[0])
          << " send_uint8_t_data[1] = "
          << static_cast<int>(send_uint8_t_data[1])
          << " send_float_data[0] = " << send_float_data[0]);
  serial_member->write();
}

void ShootCmdCallback(const std_msgs::Float32::ConstPtr& msg) {
  // 处理射击命令
  shoot_cmd = static_cast<float>(msg->data);
  ROS_INFO_STREAM_THROTTLE(
      0.5, "ShootCmdCallback: send_float_data[0] = " << send_float_data[0]);
  ROS_INFO_STREAM_THROTTLE(
      0.5, "ShootCmdCallback: msg_data = " << msg->data);
}

void DistanceCmdCallback(const ros_ecat_msgs::PositionSensorMsgConstPtr& msg)
{
  basket_distance_ = sqrt(msg->pose_x * msg->pose_x + msg->pose_y * msg->pose_y);
  basket_distance_ = basket_distance_ / 1000;
  double cmd = 0.1583*std::pow(basket_distance_,4) - 1.875*std::pow(basket_distance_,3) + 7.849*std::pow(basket_distance_,2) - 10.23*basket_distance_ + 16.13;
  // shoot_cmd = cmd;
  // ROS_INFO_STREAM("shoot_cmd:"<<shoot_cmd);
}
void DistanceCmdCallback_radar(const nav_msgs::Odometry::ConstPtr& msg)
{
  const double x_radar = msg->pose.pose.position.x;
  const double y_radar = msg->pose.pose.position.y;
  double pos_distance = sqrt(std::pow(x_radar,2)+std::pow(y_radar,2));
  ROS_INFO_STREAM("pos_distance:"<<pos_distance);
  // double cmd = 3.4596*pos_distance + 6.1212;
  // double cmd = 3.4596*pos_distance + 5.6212;
  // double cmd = 3.2597*pos_distance + 6.0043;
  //  double cmd = 3.2105*pos_distance + 6.1219; 
    //  double cmd = 4.2368*pos_distance + 6.3946; 
    //  double cmd = 3.877*pos_distance + 6.3439; 
  // double cmd = 3.444*pos_distance +6.496;
    // double cmd = 3.444*pos_distance +7.296;
    // double cmd = 3.7296*pos_distance + 6.317;
    double cmd = 3.7296*pos_distance + 6.517;

   shoot_cmd = cmd;
  // shoot_cmd = 28.0;
  ROS_INFO_STREAM("shoot_cmd:"<<shoot_cmd);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "upperboard_app_node");
  ros::NodeHandle nh("upper_board");
  serial_member = std::make_shared<serial_manager::SerialMember>(nh);
  if (!serial_member->init()) {
    ROS_ERROR("Failed to initialize SerialMember");
    return -1;
  }
  /*  */
  float_data =
      static_cast<float*>(serial_member->getFieldDataPtr("float_data", false));
  uint8_t_data = static_cast<uint8_t*>(
      serial_member->getFieldDataPtr("uint8_data", false));
  send_float_data =
      static_cast<float*>(serial_member->getFieldDataPtr("float_data", true));
  send_uint8_t_data =
      static_cast<uint8_t*>(serial_member->getFieldDataPtr("uint8_data", true));

  for (int i = 0; i < 6; i++) {
    send_float_data[i] = 0;
  }
  for (int i = 0; i < 4; i++) {
    send_uint8_t_data[i] = 0;
  }

  ros::Subscriber upperboard_cmd_sub_ =
      nh.subscribe<ros_ecat_msgs::UpperBoardCmd>("upperboard_cmd", 10,
                                                 UpperboardCmdCallback);

  ros::Subscriber shoot_cmd_sub_ =
      nh.subscribe<std_msgs::Float32>("shoot_cmd", 10, ShootCmdCallback);
    

  // ros::Subscriber distance_cmd_sub_ = nh.subscribe<std_msgs::Float64>("distance",10,DistanceCmdCallback);

  // ros::Subscriber position_data_sub_ = nh.subscribe<ros_ecat_msgs::PositionSensorMsg>("/position_sensor/position_sensor_data",10,DistanceCmdCallback);
  ros::Subscriber radar_data_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom",10,DistanceCmdCallback_radar);
  
  ros::spin();
  return 0;
}