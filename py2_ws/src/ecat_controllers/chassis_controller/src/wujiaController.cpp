/**
 * @file wujiaController.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include <chassis_controller/wujiaController.h>

#include <pluginlib/class_list_macros.hpp>

namespace chassis_controller 
{

bool wujiaController::init(
    hardware_interface::EffortJointInterface *effort_joint_interface,
    ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh){
  ROS_INFO("this ok");
  controller_nh_ = controller_nh;

  if (!controller_nh.getParam("chassis_radius", chassis_radius_)) {
    ROS_ERROR("Chassis radius not set");
    return false;
  }
  if (!controller_nh.getParam("wheel_radius", wheel_radius_)) {
    ROS_ERROR("Wheel radius not set");
    return false;
  }
  if (!controller_nh.getParam("ctrl_cmd", command_source_frame_)) {
    ROS_ERROR("Command source frame not set");
    return false;
  }
  // clear the joints_

  XmlRpc::XmlRpcValue wheels;
  controller_nh.getParam("wheels", wheels);
  for (auto &wheel : wheels) {
    ros::NodeHandle nh_wheel =
        ros::NodeHandle(controller_nh, "wheels/" + wheel.first);
    std::string joint_name;
    if (!nh_wheel.getParam("joint", joint_name)) {
      ROS_ERROR("Joint name not set");
      return false;
    }

    ROS_INFO_STREAM("name:"<<joint_name);
    joints_.insert(std::make_pair(joint_name,hardware_interface::JointHandle()));
    joints_[joint_name] =effort_joint_interface->getHandle(joint_name);

    XmlRpc::XmlRpcValue wheel_pid;
    if (!nh_wheel.getParam("pid", wheel_pid)) {
      ROS_ERROR("Joint motor pid not set");
      return false;
    }
    double p = static_cast<double>(wheel_pid["p"]);
    double i = static_cast<double>(wheel_pid["i"]);
    double d = static_cast<double>(wheel_pid["d"]);
    double i_max = static_cast<double>(wheel_pid["i_max"]);
    double output_max = static_cast<double>(wheel_pid["output_max"]);
    double deadzone = static_cast<double>(wheel_pid["deadzone"]);
    wheel_pid_map_[joint_name] = ctrl_common::IncrementalPID();
    wheel_pid_map_[joint_name].init(p, i, d, i_max, output_max, deadzone);
  }

  XmlRpc::XmlRpcValue yaw_pid;
  if(controller_nh.getParam("yaw_adjust", yaw_pid))
  {
    double yaw_p = static_cast<double>(yaw_pid["pid"]["p"]);
    double yaw_i = static_cast<double>(yaw_pid["pid"]["i"]);
    double yaw_d = static_cast<double>(yaw_pid["pid"]["d"]);
    double yaw_i_max = static_cast<double>(yaw_pid["pid"]["i_max"]);
    double yaw_output_max = static_cast<double>(yaw_pid["pid"]["output_max"]);
    double yaw_deadzone = static_cast<double>(yaw_pid["pid"]["deadzone"]);
    yaw_adjust_.init(yaw_p, yaw_i, yaw_d, yaw_i_max, yaw_output_max, yaw_deadzone);
    // yaw_adjust_flag_ = true;
  }

  // XmlRpc::XmlRpcValue yolo_pid;
  // if(controller_nh.getParam("basket_adjust", yolo_pid))
  // {
  //   double p = static_cast<double>(yolo_pid["pid"]["p"]);
  //   double i = static_cast<double>(yolo_pid["pid"]["i"]);
  //   double d = static_cast<double>(yolo_pid["pid"]["d"]);
  //   double i_max = static_cast<double>(yolo_pid["pid"]["i_max"]);
  //   double output_max = static_cast<double>(yolo_pid["pid"]["output_max"]);
  //   double deadzone = static_cast<double>(yolo_pid["pid"]["deadzone"]);
  //   yolo_adjust_.init(p, i, d, i_max, output_max, deadzone);
  // }

  /* sub the cmd_vel */
  cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>(
      command_source_frame_, 1, &wujiaController::cmdVelCallback, this);
  // odom_sub_ = root_nh.subscribe<nav_msgs::Odometry>(
  //     "odom", 1, &wujiaController::OdomCallback, this);
  // position_sub_ = root_nh.subscribe<ros_ecat_msgs::PositionSensorMsg>(
  //     "position_sensor/position_sensor_data", 1, &wujiaController::PositionCallback, this);
  remote_state_sub_=root_nh.subscribe<ros_ecat_msgs::RemoteState>(
      "/remote_state", 1, &wujiaController::RemoteStateCallback, this);
  // yolo_sub_=root_nh.subscribe<std_msgs::Float32>("/yolo/offset",1,&wujiaController::YoloCallback,this);
    // //调试所用话题
  // lf_cmd_vel_sub_=controller_nh.subscribe<std_msgs::Float64>(
  //     "left_wheel_joint/cmd_vel", 1, &wujiaController::lfVelCallback, this);
  // rf_cmd_vel_sub_=controller_nh.subscribe<std_msgs::Float64>(
  //     "left_wheel_joint/cmd_vel", 1, &wujiaController::rfVelCallback, this);
  // lb_cmd_vel_sub_=controller_nh.subscribe<std_msgs::Float64>(
  //     "right_wheel_joint/cmd_vel", 1, &wujiaController::lbVelCallback, this);
  // rb_cmd_vel_sub_=controller_nh.subscribe<std_msgs::Float64>(
  //     "front_wheel_joint/cmd_vel", 1, &wujiaController::rbVelCallback, this);
  
  
  return true;
}

void wujiaController::update(const ros::Time &time,
                                   const ros::Duration &period) {
  
  std::vector<double> joint_cmd;
  

  // 计算轮子速度
  joint_cmd = MoveJoint();
  
  //更新电机速度
  wheel_vel_map_["left_wheel_joint"] = joints_["left_wheel_joint"].getVelocity();
  wheel_vel_map_["right_wheel_joint"] = joints_["right_wheel_joint"].getVelocity();
  wheel_vel_map_["front_wheel_joint"] = joints_["front_wheel_joint"].getVelocity();
  // wheel_vel_map_["left_wheel_joint"] = joints_["left_wheel_joint"].getVelocity();
  
  //从参数服务器更新当前pid参数，用于动态调参
  // UpdateParameters();
  
  
  //pid输出
  wheel_cmd_map_["left_wheel_joint"] = 
      wheel_pid_map_["left_wheel_joint"].PIDCalculate(wheel_vel_map_["left_wheel_joint"], joint_cmd[0]);
  wheel_cmd_map_["right_wheel_joint"] =
      wheel_pid_map_["right_wheel_joint"].PIDCalculate(wheel_vel_map_["right_wheel_joint"], joint_cmd[1]);
  wheel_cmd_map_["front_wheel_joint"] =
      wheel_pid_map_["front_wheel_joint"].PIDCalculate(wheel_vel_map_["front_wheel_joint"], joint_cmd[2]);
  // wheel_cmd_map_["left_wheel_joint"] =
  //     wheel_pid_map_["left_wheel_joint"].PIDCalculate(wheel_vel_map_["left_wheel_joint"], joint_cmd[3]);

  // 更新电机速度
  for (const auto &entry : joints_) {
    const std::string &joint_name = entry.first;
    if (wheel_cmd_map_.find(joint_name) != wheel_cmd_map_.end()) {
      joints_[joint_name].setCommand(wheel_cmd_map_[joint_name]);
      // joints_[joint_name].update(time, period);
    }
  }
}

std::vector<double> wujiaController::MoveJoint() {
  std::vector<double> joint_cmd(3, 0.0);

  std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
  geometry_msgs::Twist *cmd_vel = cmd_vel_buffer_.readFromRT();
  ros_ecat_msgs::PositionSensorMsg *position_data = position_buffer_.readFromRT();
  ros_ecat_msgs::RemoteState *remote_state = remote_state_buffer_.readFromRT();
  std_msgs::Float32* yolo_error = yolo_offset_buffer_.readFromRT();


  double Yaw;
  double yaw_target_pos;
  float yaw_target_yolo;
  double yaw_adjust_cmd;
  yaw_adjust_flag_ = remote_state->yaw_lock;

  ros::Time yolo_now = ros::Time::now();
  double yolo_now_time = yolo_now.sec + yolo_now.nsec * 1000000;
  double yolo_nsec_time = yolo_time_stamp_.sec + yolo_time_stamp_.nsec*1000000;

  //   if(yaw_adjust_flag_)
  //   {
  //     if(yolo_error->data!=0)
  //     // if(yolo_now_time - yolo_nsec_time <= 0.1)
  //     {
  //       yaw_adjust_cmd = yolo_adjust_.PIDCalculateByError(-yolo_error->data);
  //       ROS_INFO_STREAM("YOLO_cmd:"<<yaw_adjust_cmd);
  //     }
  //     else
  //     {
  //       Yaw = position_data->yaw_angle;
  //       Yaw = -Yaw;
  //       double basket_yaw = atan2(position_data->pose_y, position_data->pose_x);
    
  //       if(position_data->pose_x>0)basket_yaw= -(M_PI/2.0 + basket_yaw);
  //       else if(position_data->pose_x<0) basket_yaw= -(basket_yaw+M_PI/2.0);
  //       else basket_yaw=0;

  //       basket_yaw = basket_yaw/M_PI * 180.0;
  //       ROS_INFO_STREAM("basket yaw:"<<basket_yaw);


  //       yaw_adjust_cmd = yaw_adjust_.YawAdjustCalculate(Yaw,basket_yaw);
  //     }
  //     cmd_vel->angular.z=0;//avoid angular controller
  //     // ROS_INFO_STREAM("yaw:"<<Yaw<<"yaw_cmd:"<<yaw_adjust_cmd);
  //   }
  //   else yaw_adjust_cmd = 0.0;


  //   int i = 0;

  //   joint_cmd[0] =
  //       static_cast<double>((-1.) *
  //                           (cmd_vel->linear.x * (-1.) * cos(M_PI / 4) +
  //                            cmd_vel->linear.y * sin(M_PI / 4) +
  //                            (cmd_vel->angular.z+yaw_adjust_cmd) * chassis_radius_) /
  //                           wheel_radius_);

  //   joint_cmd[1] =
  //       static_cast<double>((-1.) *
  //                           (cmd_vel->linear.x * (-1.) * cos(M_PI / 4) -
  //                            cmd_vel->linear.y * sin(M_PI / 4) +
  //                            (cmd_vel->angular.z + yaw_adjust_cmd) * chassis_radius_) /
  //                           wheel_radius_);

  //   joint_cmd[2] = static_cast<double>((-1.) *
  //                                      (cmd_vel->linear.x * cos(M_PI / 4) -
  //                                       cmd_vel->linear.y * sin(M_PI / 4) +
  //                                       (cmd_vel->angular.z + yaw_adjust_cmd) * chassis_radius_) /
  //                                      wheel_radius_);

  //   joint_cmd[3] = static_cast<double>((-1.) *
  //                                      (cmd_vel->linear.x * cos(M_PI / 4) +
  //                                       cmd_vel->linear.y * sin(M_PI / 4) +
  //                                       (cmd_vel->angular.z+yaw_adjust_cmd) * chassis_radius_) /
  //                                      wheel_radius_);
  // }

    joint_cmd[0] =
      static_cast<double>((1.) *
                          (cmd_vel->linear.x  / (cos(M_PI / 6)*2) +
                            cmd_vel->linear.y / 3.0 +
                            (cmd_vel->angular.z*(-1.0)) * chassis_radius_) /
                          wheel_radius_);
    joint_cmd[1] =
        static_cast<double>((1.) *
                            (cmd_vel->linear.x * (-1) / (cos(M_PI / 6)*2) +
                             cmd_vel->linear.y / 3.0 +
                             (cmd_vel->angular.z*(-1.0)) * chassis_radius_) /
                            wheel_radius_ );

    joint_cmd[2] = static_cast<double>((-1.) *
                                       (cmd_vel->linear.x * 0 -
                                        cmd_vel->linear.y * 2 * (-1.0) / 3.0 +
                                        (cmd_vel->angular.z*(-1.0)) * chassis_radius_) /
                                       wheel_radius_);

    // joint_cmd[3] = static_cast<double>((-1.) *
    //                                    (cmd_vel->linear.x * cos(M_PI / 4) +
    //                                     cmd_vel->linear.y * sin(M_PI / 4) +
    //                                     (cmd_vel->angular.z+yaw_adjust_cmd) * chassis_radius_) /
    //                                    wheel_radius_);
  return joint_cmd;
}

// double wujiaController::QuaternionYaw()
// {
//   nav_msgs::Odometry *odom_data = odom_buffer_.readFromRT();
//   tf2::Quaternion q(odom_data->pose.pose.orientation.x,odom_data->pose.pose.orientation.y,odom_data->pose.pose.orientation.z,odom_data->pose.pose.orientation.w);
//   q.normalize();

//   double roll,pitch,Yaw;
//   tf2::Matrix3x3(q).getRPY(roll,pitch,Yaw);
//   Yaw = (double)(Yaw/PI*180.0);//change to rad
//   return Yaw;
// }

// double wujiaController::GetYawFromBasket()
// {
//   //确保机器人在篮筐下方reset定位码盘
//   nav_msgs::Odometry *odom_data = odom_buffer_.readFromRT();

//   double basket_yaw = atan2(odom_data->pose.pose.position.y, odom_data->pose.pose.position.x);
//   if (basket_yaw < 0) {
//     basket_yaw += 2 * M_PI; // 确保角度在0到2π之间
//   }
//   return basket_yaw;
// }

// float wujiaController::GetYawFromYolo()
// {
  
// }

  void wujiaController::UpdateParameters() {

    XmlRpc::XmlRpcValue wheels;
    controller_nh_.getParam("wheels", wheels);
    for(auto &wheel:wheels)
    {
      std::string joint_name = wheel.second["joint"];
      XmlRpc::XmlRpcValue wheel_pid=wheel.second["pid"];
      double p = static_cast<double>(wheel_pid["p"]);
      double i = static_cast<double>(wheel_pid["i"]);
      double d = static_cast<double>(wheel_pid["d"]);
      double i_max = static_cast<double>(wheel_pid["i_max"]);
      double output_max = static_cast<double>(wheel_pid["output_max"]);
      double deadzone = static_cast<double>(wheel_pid["deadzone"]);
      wheel_pid_map_[joint_name].init(p, i, d, i_max, output_max, deadzone);
    }

    if(yaw_adjust_flag_)
    {
      XmlRpc::XmlRpcValue yaw_pid;
      controller_nh_.getParam("yaw_adjust", yaw_pid);
      double yaw_p = static_cast<double>(yaw_pid["p"]);
      double yaw_i = static_cast<double>(yaw_pid["i"]);
      double yaw_d = static_cast<double>(yaw_pid["d"]);
      double yaw_i_max = static_cast<double>(yaw_pid["i_max"]);
      double yaw_output_max = static_cast<double>(yaw_pid["output_max"]);
      double yaw_deadzone = static_cast<double>(yaw_pid["deadzone"]);
      yaw_adjust_.init(yaw_p, yaw_i, yaw_d, yaw_i_max, yaw_output_max, yaw_deadzone);
    }
  }

  void wujiaController::cmdVelCallback(
      const geometry_msgs::TwistConstPtr &msg) {
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    cmd_vel_buffer_.writeFromNonRT(*msg);
  }

  // void wujiaController::OdomCallback(
  //     const nav_msgs::OdometryConstPtr &msg) {
  //   std::lock_guard<std::mutex> lock(odom_mutex_);
  //   odom_buffer_.writeFromNonRT(*msg);
  //   yolo_time_stamp_ = ros::Time::now();
  // }

  // void wujiaController::PositionCallback(const ros_ecat_msgs::PositionSensorMsgConstPtr &msg)
  // {
  //   position_buffer_.writeFromNonRT(*msg);
  // }

  void wujiaController::RemoteStateCallback(const ros_ecat_msgs::RemoteStateConstPtr &msg) 
  {
    remote_state_buffer_.writeFromNonRT(*msg);
  }

  // void wujiaController::YoloCallback(const std_msgs::Float32ConstPtr &msg)
  // {
  //   yolo_offset_buffer_.writeFromNonRT(*msg);
  // }

  // geometry_msgs::Twist forwardKinematics(void) {}

  // void wujiaController::lfVelCallback(const std_msgs::Float64ConstPtr &msg)
  // {
  //   // left_front_buffer_.writeFromNonRT(*msg);
  //   // wheel_vel_map_["left_wheel_joint"] = joints_["left_wheel_joint"].getVelocity();
  //   // double lf_cmd_vel = wheel_pid_map_["left_wheel_joint"].PIDCalculate(wheel_vel_map_["left_wheel_joint"], msg->data);
  //   // ROS_INFO_STREAM("vel_cmd:"<<lf_cmd_vel);
  //   // joints_["left_wheel_joint"].setCommand(lf_cmd_vel);
  //   // joint_cmd[0]=msg->data;
    
  // }

  // void wujiaController::rfVelCallback(const std_msgs::Float64ConstPtr &msg)
  // {
  //   wheel_vel_map_["left_wheel_joint"] = joints_["left_wheel_joint"].getVelocity();
  //   double rf_cmd_vel = wheel_pid_map_["left_wheel_joint"].PIDCalculate(wheel_vel_map_["left_wheel_joint"], msg->data);
  //   joints_["left_wheel_joint"].setCommand(rf_cmd_vel);
  // }

  // void wujiaController::lbVelCallback(const std_msgs::Float64ConstPtr &msg)
  // {
  //   wheel_vel_map_["right_wheel_joint"] = joints_["right_wheel_joint"].getVelocity();
  //   double lb_cmd_vel = wheel_pid_map_["right_wheel_joint"].PIDCalculate(wheel_vel_map_["right_wheel_joint"], msg->data);
  //   joints_["right_wheel_joint"].setCommand(lb_cmd_vel);
  // }

  // void wujiaController::rbVelCallback(const std_msgs::Float64ConstPtr &msg)
  // {
  //   wheel_vel_map_["front_wheel_joint"] = joints_["front_wheel_joint"].getVelocity();
  //   double rb_cmd_vel = wheel_pid_map_["front_wheel_joint"].PIDCalculate(wheel_vel_map_["front_wheel_joint"], msg->data);
  //   joints_["front_wheel_joint"].setCommand(rb_cmd_vel);
  // }

}  // namespace chassis_controller

PLUGINLIB_EXPORT_CLASS(chassis_controller::wujiaController,
                       controller_interface::ControllerBase)
