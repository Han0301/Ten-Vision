/**
 * @file SwerveChassisController
 * @author py <997276894@qq.com>
 * @brief  底盘舵轮控制器
 * @version 0.1
 * @date  2025-05-24
 *
 * @copyright Copyright (c)  2025
 *
 * @attention
 * @note
 */

#include "chassis_controller/SwerveChassisController.h"

#include <pluginlib/class_list_macros.hpp>

namespace chassis_controller
{

  bool SwerveChassisController::init(hardware_interface::RobotHW *robot_hw,
                                     ros::NodeHandle &root_nh,
                                     ros::NodeHandle &controller_nh)
  {
    if (!controller_nh.getParam("triangle_base", triangle_base_))
    {
      ROS_ERROR("Triangle base not set");
      return false;
    }

    if (!controller_nh.getParam("vertex_to_ChassisCenter_L",
                                vertex_to_ChassisCenter_L_))
    {
      ROS_ERROR("Vertex to chassis center not set");
      return false;
    }

    if (!controller_nh.getParam("base_to_ChassisCenter_B",
                                base_to_ChassisCenter_B_))
    {
      ROS_ERROR("Base to chassis center not set");
      return false;
    }

    if (!controller_nh.getParam("wheel_radius", wheel_radius_))
    {
      ROS_ERROR("Wheel radius not set");
      return false;
    }

    if (!controller_nh.getParam("steer_calibration_vel", steer_check_vel_))
    {
      ROS_ERROR("Steer check velocity not set");
      return false;
    }

    if (!controller_nh.getParam("light_door_skew", light_door_skew_))
    {
      ROS_ERROR("Light door skewing not set");
      return false;
    }

    if (!controller_nh.getParam("forward_acceleration", forward_acceleration_))
    {
      ROS_ERROR("start acceleration not set");
      return false;
    }

    if (!controller_nh.getParam("reverse_acceleration", reverse_acceleration_))
    {
      ROS_ERROR("brake acceleration not set");
      return false;
    }

    if (!controller_nh.getParam("x_acceleration", x_acceleration_))
    {
      ROS_ERROR("X acceleration not set");
      return false;
    }

    if (!controller_nh.getParam("y_acceleration", y_acceleration_))
    {
      ROS_ERROR("Y acceleration not set");
      return false;
    }

    if (!controller_nh.getParam("basket_x", basket_x_))
    {
      ROS_ERROR("basket x  not set");
      return false;
    }

    // if (!controller_nh.getParam("basket_y", basket_y_)) {
    //   ROS_ERROR("basket y  not set");
    //   return false;
    // }

    if (!controller_nh.getParam("camera_center", camera_offset_))
    {
      ROS_ERROR("camera center  not set");
      return false;
    }

    initialParams.xc = initial_guessed_x_;
    initialParams.yc = initial_guessed_y_;
    // camera_offset_ = ros::param::param("camera_offset",320.0);
    // basket_x_ = ros::param::param("basket_x",0.0);
    basket_y_ = ros::param::param("basket_y", -3.67);

    std::string joint_name;
    if (!controller_nh.getParam("lock_RGB_joint", joint_name))
    {
      ROS_INFO("lock joint has not set");
      return false;
    }
    lock_RGB_joint_ = robot_hw->get<ecat_slave_hw::GpioCommandInterface>()->getHandle(joint_name);
    // if (!controller_nh.getParam("yaml_offset_path", yaml_path_)) {
    //   ROS_ERROR("yaml path not set");
    //   return false;
    // }

    // if (!controller_nh.getParam("camera_offset", camera_offset_)) {
    //   ROS_ERROR("camera offset not set");
    //   return false;
    // }

    // if (!controller_nh.getParam("basket_x", basket_x_)) {
    //   ROS_ERROR("basket x not set");
    //   return false;
    // }

    // if (!controller_nh.getParam("basket_y", basket_y_)) {
    //   ROS_ERROR("basket y not set");
    //   return false;
    // }

    XmlRpc::XmlRpcValue wheels;
    controller_nh.getParam("wheels", wheels);
    for (auto &wheel : wheels)
    {
      // 获取轮向电机的关节句柄6.84
      ros::NodeHandle nh_wheel =
          ros::NodeHandle(controller_nh, "wheels/" + wheel.first);
      std::string wheel_joint_name;
      if (!nh_wheel.getParam("wheel_joint", wheel_joint_name))
      {
        ROS_ERROR_STREAM("Wheel joint name not set" << wheel.first);
        return false;
      }
      wheel_joints_.insert(
          std::make_pair(wheel_joint_name, hardware_interface::JointHandle()));
      wheel_joints_[wheel_joint_name] =
          robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(
              wheel_joint_name);

      // 获取舵向电机的关节句柄
      std::string swerve_joint_name;
      if (!nh_wheel.getParam("swerve_joint", swerve_joint_name))
      {
        ROS_ERROR("Swerve joint name no set");
        return false;
      }
      swerve_joints_.insert(
          std::make_pair(swerve_joint_name, hardware_interface::JointHandle()));
      swerve_joints_[swerve_joint_name] =
          robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(
              swerve_joint_name);

      // 获取舵向电机pid参数
      XmlRpc::XmlRpcValue increment_pid;
      if (!nh_wheel.getParam("swerve_increment_pid", increment_pid))
      {
        ROS_ERROR("Swerve incremental PID not set");
        return false;
      }
      double p = static_cast<double>(increment_pid["p"]);
      double i = static_cast<double>(increment_pid["i"]);
      double d = static_cast<double>(increment_pid["d"]);
      double i_max = static_cast<double>(increment_pid["i_max"]);
      double output_max = static_cast<double>(increment_pid["output_max"]);
      double deadzone = static_cast<double>(increment_pid["deadzone"]);
      swerve_increment_pid_.insert(
          std::make_pair(swerve_joint_name, ctrl_common::IncrementalPID()));
      swerve_increment_pid_[swerve_joint_name].init(p, i, d, i_max, output_max,
                                                    deadzone);

      XmlRpc::XmlRpcValue position_pid;
      if (!nh_wheel.getParam("swerve_position_pid", position_pid))
      {
        ROS_ERROR("Swerve position PID not set");
        return false;
      }
      p = static_cast<double>(position_pid["p"]);
      i = static_cast<double>(position_pid["i"]);
      d = static_cast<double>(position_pid["d"]);
      i_max = static_cast<double>(position_pid["i_max"]);
      output_max = static_cast<double>(position_pid["output_max"]);
      deadzone = static_cast<double>(position_pid["deadzone"]);
      swerve_position_pid_.insert(
          std::make_pair(swerve_joint_name, ctrl_common::PositionPID()));
      swerve_position_pid_[swerve_joint_name].init(p, i, d, i_max, output_max,
                                                   deadzone);

      std::string steer_io;
      if (!nh_wheel.getParam("steer_io", steer_io))
      {
        ROS_ERROR("Steer IO not set for joint");
        return false;
      }
      steer_io_.insert(
          std::make_pair(swerve_joint_name, ecat_slave_hw::GpioStateHandle()));
      steer_io_[swerve_joint_name] =
          robot_hw->get<ecat_slave_hw::GpioStateInterface>()->getHandle(steer_io);

      steer_io_state_.insert(
          std::make_pair(swerve_joint_name, true)); // 初始化舵轮IO状态为false
      swerve_joint_pos_.insert(
          std::make_pair(swerve_joint_name, steer_joint_pos()));
    }

    // 获取航向角调整PID
    XmlRpc::XmlRpcValue yaw_pid;
    if (!controller_nh.getParam("yaw_adjust", yaw_pid))
    {
      ROS_WARN("Swerve YAW PID not set");
      yaw_adjust_flag_ = false;
    }
    else
    {
      double yaw_p = static_cast<double>(yaw_pid["pid"]["p"]);
      double yaw_i = static_cast<double>(yaw_pid["pid"]["i"]);
      double yaw_d = static_cast<double>(yaw_pid["pid"]["d"]);
      double yaw_i_max = static_cast<double>(yaw_pid["pid"]["i_max"]);
      double yaw_output_max = static_cast<double>(yaw_pid["pid"]["output_max"]);
      double yaw_deadzone = static_cast<double>(yaw_pid["pid"]["deadzone"]);
      yaw_pid_.init(yaw_p, yaw_i, yaw_d, yaw_i_max, yaw_output_max, yaw_deadzone);
    }

    XmlRpc::XmlRpcValue basket_pid;
    if (!controller_nh.getParam("basket_adjust", basket_pid))
    {
      ROS_WARN("Swerve yolo basket PID not set");
    }
    else
    {
      double yaw_p = static_cast<double>(basket_pid["pid"]["p"]);
      double yaw_i = static_cast<double>(basket_pid["pid"]["i"]);
      double yaw_d = static_cast<double>(basket_pid["pid"]["d"]);
      double yaw_i_max = static_cast<double>(basket_pid["pid"]["i_max"]);
      double yaw_output_max = static_cast<double>(basket_pid["pid"]["output_max"]);
      double yaw_deadzone = static_cast<double>(basket_pid["pid"]["deadzone"]);
      basket_pid_.init(yaw_p, yaw_i, yaw_d, yaw_i_max, yaw_output_max, yaw_deadzone);
    }

    cmd_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>(
        "cmd_vel", 1, &SwerveChassisController::cmdVelCallback, this);
    odom_sub_ = root_nh.subscribe<nav_msgs::Odometry>(
        "odom", 1, &SwerveChassisController::OdomCallback, this);

    partner_sub_ = root_nh.subscribe<ros_ecat_msgs::PositionSensorMsg>(
        "/partner_position", 1, &SwerveChassisController::PartnerCallback, this);

    init_sub_ = controller_nh.subscribe<std_msgs::String>(
        "init_steer", 1, &SwerveChassisController::initCallback, this);

    debug_sub_ = controller_nh.subscribe<std_msgs::Float32>(
        "vel_debug", 1, &SwerveChassisController::debugCallback, this);

    // vesc_sub_ = root_nh.subscribe<ros_ecat_msgs::VescEcatRosMsg>("vesc_msg", 1,
    // &SwerveChassisController::vescCallback, this);

    imu_sub_ = root_nh.subscribe<geometry_msgs::Vector3Stamped>(
        "/mahony/euler", 1, &SwerveChassisController::imuCallback, this);

    remote_state_sub_ = root_nh.subscribe<ros_ecat_msgs::RemoteState>(
        "/remote_state", 1, &SwerveChassisController::RemoteStateCallback, this);

    camera_sub_ = root_nh.subscribe<vision_msgs::BoundingBox2DArray>(
        "/yolo/detections", 1, &SwerveChassisController::CameraCallback, this);

    dribble_vel_sub_ = root_nh.subscribe<geometry_msgs::Twist>(
        "/dribble_control/cmd_vel", 1, &SwerveChassisController::DribbleVelCallback, this);
        
    position_sub_ = root_nh.subscribe<ros_ecat_msgs::PositionSensorMsg>(
      "/imu_data", 1, &SwerveChassisController::PositionCallback, this);

    pbox_state_pub_ = std::make_shared<any_node::ThreadedPublisher<ros_ecat_msgs::PboxMsg>>(
        root_nh.advertise<ros_ecat_msgs::PboxMsg>("/pbox_data", 1), 15, true);

    init_state_pub_ = std::make_shared<any_node::ThreadedPublisher<ros_ecat_msgs::InitState>>(
        root_nh.advertise<ros_ecat_msgs::InitState>("/init_state", 1), 15, true);

    debug_pos_pub_ = std::make_shared<any_node::ThreadedPublisher<std_msgs::Float32>>(
        root_nh.advertise<std_msgs::Float32>("pos_debug", 1), 15, true);

    position_reset_pub_ = std::make_shared<any_node::ThreadedPublisher<ros_ecat_msgs::PositionSensorCmd>>(
      root_nh.advertise<ros_ecat_msgs::PositionSensorCmd>("/imu_cmd", 1), 15, true);


    // 初始化时间戳
    ros::Time init_time = ros::Time::now();
    last_cmd_time_ = init_time.toSec();
    is_start_init_ = true;
    return true;
  }

  void SwerveChassisController::update(const ros::Time &time,
                                       const ros::Duration &period)
  {
    SwerveStateGet();
    if (is_init_)
    {
      SwerveCal();
      // 劣弧优化
      MinorArcOptimzation();

      std::vector<double> position_output(3, 0.0);

      position_output[0] =
          swerve_position_pid_["steer_front_joint"].PIDCalculateByError(
              swerve_error_map_["one_front"]);
      position_output[1] =
          swerve_position_pid_["steer_left_rear_joint"].PIDCalculateByError(
              swerve_error_map_["left_back"]);
      position_output[2] =
          swerve_position_pid_["steer_right_rear_joint"].PIDCalculateByError(
              swerve_error_map_["right_back"]);

      // ROS_INFO_STREAM("error: "<<swerve_error_map_["one_front"]<<"position
      // output: "<<position_output[0]);
      // 内环增量式计算
      swerve_cmd_map_["steer_front_joint"] =
          swerve_increment_pid_["steer_front_joint"].PIDCalculate(
              swerve_vel_cur_["steer_front_joint"], -1.0 * position_output[0]);
      swerve_cmd_map_["steer_left_rear_joint"] =
          swerve_increment_pid_["steer_left_rear_joint"].PIDCalculate(
              swerve_vel_cur_["steer_left_rear_joint"],
              -1.0 * position_output[1]);
      swerve_cmd_map_["steer_right_rear_joint"] =
          swerve_increment_pid_["steer_right_rear_joint"].PIDCalculate(
              swerve_vel_cur_["steer_right_rear_joint"],
              -1.0 * position_output[2]);

      // ROS_INFO_STREAM("target speed: "<<position_output[0]<<"output : "<<swerve_cmd_map_["steer_front_joint"]);
      // ROS_INFO_STREAM_THROTTLE(0.5,"front:
      // "<<swerve_cmd_map_["steer_front_joint"]<<"left:
      // "<<swerve_cmd_map_["steer_left_rear_joint"]<<"right:
      // "<<swerve_cmd_map_["steer_right_rear_joint"]);
      // ROS_INFO_STREAM_THROTTLE(0.5,"front:
      // "<<swerve_error_map_["one_front"]<<"left:
      // "<<swerve_error_map_["left_back"]<<"right:
      // "<<swerve_error_map_["right_back"]);
      // 将轮向电机输出从m/s转换为rad/s
      wheel_cmd_map_["drive_front_joint"] =
          wheel_cmd_vel_["one_front"] / wheel_radius_;
      wheel_cmd_map_["drive_left_rear_joint"] =
          wheel_cmd_vel_["left_back"] / wheel_radius_;
      wheel_cmd_map_["drive_right_rear_joint"] =
          -wheel_cmd_vel_["right_back"] / wheel_radius_;

      for (auto &it : swerve_joints_)
      {
        std::string joint_name = it.first;
        if (swerve_cmd_map_.find(joint_name) != swerve_cmd_map_.end())
        {
          it.second.setCommand(swerve_cmd_map_[joint_name]);
        }
        else
        {
          ROS_WARN_STREAM("Swerve command for joint " << joint_name
                                                      << " not found.");
        }
      }


      // if(wheel_cmd_map_["one_front"]!=0)
      // {
      //     if(swerve_error_map_["one_front"]<=0.02 &&
      //     swerve_error_map_["one_front"]>=-0.02)wheel_joints_["drive_front_joint"].setCommand(wheel_cmd_map_["drive_front_joint"]);
      // }
      // else wheel_joints_["drive_front_joint"].setCommand(0);

      // if(wheel_cmd_map_["left_back"]!=0)
      // {
      //     if(swerve_error_map_["left_back"]<=0.02 &&
      //     swerve_error_map_["left_back"]>=-0.02)wheel_joints_["drive_left_rear_joint"].setCommand(wheel_cmd_map_["drive_left_rear_joint"]);
      // }
      // else wheel_joints_["drive_left_rear_joint"].setCommand(0);

      // if(wheel_cmd_map_["right_back"]!=0)
      // {
      //     if(swerve_error_map_["right_back"]<=0.02 &&
      //     swerve_error_map_["right_back"]>=-0.02)wheel_joints_["drive_right_rear_joint"].setCommand(wheel_cmd_map_["drive_right_rear_joint"]);
      // }
      // else wheel_joints_["drive_right_rear_joint"].setCommand(0);

      for (auto &it : wheel_joints_)
      {
        std::string joint_name = it.first;
        if (wheel_cmd_map_.find(joint_name) != wheel_cmd_map_.end())
        {
          it.second.setCommand(wheel_cmd_map_[joint_name]);
        }
        else
        {
          ROS_WARN_STREAM("Wheel command for joint " << joint_name
                                                     << " not found.");
        }
      }

      // ROS_INFO_STREAM_THROTTLE(0.1,"target pos:
      // "<<swerve_cmd_pos_["right_back"]<<"error:
      // "<<swerve_error_map_["right_back"]<<"now pos:
      // "<<swerve_pos_cur_["right_back"]);
    }
    else
    {
      SwerveInit();
      pbox_data_.swerve_mode = 0;
    }
    pbox_state_pub_->publish(pbox_data_);
  }

  void SwerveChassisController::SwerveStateGet()
  {
    for (auto &swerve : swerve_joints_)
    {
      // 获取位置
      swerve_joint_pos_[swerve.first].current_pos = swerve.second.getPosition();
      swerve_joint_pos_[swerve.first].current_pos *= -1;
      swerve_joint_pos_[swerve.first].diff_pos =
          swerve_joint_pos_[swerve.first].current_pos -
          swerve_joint_pos_[swerve.first].last_pos;
      swerve_joint_pos_[swerve.first].last_pos =
          swerve_joint_pos_[swerve.first].current_pos;
      swerve_joint_pos_[swerve.first].final_pos +=
          swerve_joint_pos_[swerve.first].diff_pos;

      // AngleLimit(&swerve_joint_pos_[swerve.first].final_pos);
      // if(swerve_joint_pos_[swerve.first].final_pos<0)swerve_joint_pos_[swerve.first].final_pos
      // +=2.0*M_PI;

      // 获取速度
      swerve_vel_cur_[swerve.first] = swerve.second.getVelocity();
    }
    swerve_pos_cur_["one_front"] =
        swerve_joint_pos_["steer_front_joint"].final_pos;
    // AngleLimit(&swerve_pos_cur_["one_front"]);
    swerve_pos_cur_["left_back"] =
        swerve_joint_pos_["steer_left_rear_joint"].final_pos;
    swerve_pos_cur_["right_back"] =
        swerve_joint_pos_["steer_right_rear_joint"].final_pos;

    std_msgs::Float32 debug_pos;
    debug_pos.data = swerve_pos_cur_["one_front"];
    debug_pos_pub_->publish(debug_pos);
    ROS_INFO_STREAM_THROTTLE(0.5,"front:"<<swerve_joint_pos_["steer_front_joint"].final_pos<<"left:"<<swerve_joint_pos_["steer_left_rear_joint"].final_pos<<"right:"<<swerve_joint_pos_["steer_right_rear_joint"].final_pos);
    // ROS_INFO_STREAM_THROTTLE(0.5,"front:"<<wheel_joints_["steer_front_joint"].getVelocity()<<"left:"<<wheel_joints_["drive_left_rear_joint"].getVelocity()<<"right:"<<wheel_joints_["drive_right_rear_joint"].getVelocity());
  }

  void SwerveChassisController::SwerveCal()
  {
    {
      std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
      geometry_msgs::Twist *cmd_vel = cmd_vel_buffer_.readFromRT();
      geometry_msgs::Twist *cmd_set_vel = cmd_vel;
      geometry_msgs::Twist *dribble_vel = dribble_vel_buffer_.readFromRT();
      geometry_msgs::Vector3Stamped *imu_data = imu_buffer_.readFromNonRT();
      ros_ecat_msgs::RemoteState *remote_state =
          remote_state_buffer_.readFromRT();
      nav_msgs::Odometry *position_data = odom_buffer_.readFromRT();
      ros_ecat_msgs::PositionSensorMsg *partner_data = partner_buffer_.readFromRT();
      vision_msgs::BoundingBox2DArray *camera_data = camera_buffer_.readFromRT();
      ros_ecat_msgs::PositionSensorMsg  *position_imu = position_buffer_.readFromRT();
      uint8_t yaw_adjust_flag_ = remote_state->yaw_lock;
      double w_;
      double Yaw = imu_data->vector.z;
      double yaw_adjust_cmd;

      // /*********************初始化打点相关********************* */
      // if(!camera_offset_finish_ && camera_init_)
      // {
      //   if(remote_state->set_offset)
      //   {
      //     auto& box = camera_data->boxes[0];
      //     CameraOffset(box.center.x);
      //   }
      //   if(remote_state->set_finish)
      //   {
      //     if(!last_finish_set_)
      //     {
      //       camera_offset_finish_ = true;
      //       last_finish_set_ = true;
      //       ros::param::set("camera_offset", camera_offset_);
      //     }
      //   }
      //   else last_finish_set_ = false;
      // }
      if (!basket_offset_finish_)
      {
        if (remote_state->set_offset)
        {
          basket_y_ = position_data->pose.pose.position.y;
        }

        if (remote_state->set_finish)
        {
          if (!last_finish_set_)
          {
            basket_offset_finish_ = true;
            last_finish_set_ = true;
            ros::param::set("basket_y", basket_y_);
            ros_ecat_msgs::PositionSensorCmd imu_cmd;
            imu_cmd.cmd = "RESET";
            position_reset_pub_->publish(imu_cmd);
          }
        }
        else
          last_finish_set_ = false;
      }
      init_state_msg_.init_camera_flag = camera_offset_finish_;
      init_state_msg_.init_basket_flag = basket_offset_finish_;
      init_state_msg_.camera_center = camera_offset_;
      init_state_msg_.basket_x = basket_x_;
      init_state_msg_.basket_y = basket_y_;
      init_state_pub_->publish(init_state_msg_);

      if (camera_data && camera_init_)
      {
        auto &box = camera_data->boxes[0];
        if (box.center.x == -1.0 && box.center.y == 0.0)
          pbox_data_.camera_flag = 0;
        else
          pbox_data_.camera_flag = 1;
      }

      // 加速度规划
      // AccelerationPlanning(cmd_vel);

      if (yaw_adjust_flag_ == 1)
      {
        Yaw = imu_data->vector.z; // 航向角

        double basket_yaw = atan2(basket_y_ - position_data->pose.pose.position.y, basket_x_ - position_data->pose.pose.position.x);

        // if (position_data->pose.pose.position.y < 0)
        //   basket_yaw = (M_PI / 2.0 + basket_yaw);
        // else if (position_data->pose.pose.position.y > 0)
        //   basket_yaw = (basket_yaw + M_PI / 2.0);
        // else
        //   basket_yaw = 0;

        // basket_yaw = round(basket_yaw / M_PI * 180.0 - 1.5);
        basket_yaw = basket_yaw / M_PI * 180.0;
        yaw_adjust_cmd = yaw_pid_.YawAdjustCalculate(Yaw, basket_yaw);
        ROS_INFO_STREAM("basket yaw:" << basket_yaw << " yaw output: " << yaw_adjust_cmd);

        // 完成锁框后驻车
        // if (!yaw_adjust_cmd) {
        //   Parking();
        //   return;
        // }
        // if(yaw_adjust_cmd<0)yaw_adjust_cmd-=0.5;
        // else if(yaw_adjust_cmd>0)yaw_adjust_cmd+=0.5;
        w_ = yaw_adjust_cmd;
        AccelerationPlanning(cmd_set_vel);

        pbox_data_.swerve_mode = 3;
        if (w_ == 0)
        {
          pbox_data_.lock_backet_flag = true;
          lock_RGB_joint_.setCommand(true);
        }
        else
        {
          pbox_data_.lock_backet_flag = false;
          lock_RGB_joint_.setCommand(false);
        }
      }
      else if (yaw_adjust_flag_ == 2)
      {
        Yaw = imu_data->vector.z; // 航向角

        double partner_yaw = atan2(partner_data->pose_x - position_data->pose.pose.position.x,
                                   partner_data->pose_y - position_data->pose.pose.position.y);

        if ((position_data->pose.pose.position.y - partner_data->pose_y) < 0)
          partner_yaw = -(M_PI / 2.0 + partner_yaw);
        else if ((position_data->pose.pose.position.y - partner_data->pose_y) > 0)
          partner_yaw = -(partner_yaw + M_PI / 2.0);
        else
          partner_yaw = 0;

        partner_yaw = round(partner_yaw / M_PI * 180.0);

        yaw_adjust_cmd = yaw_pid_.YawAdjustCalculate(Yaw, partner_yaw);
        ROS_INFO_STREAM("partner yaw:" << partner_yaw << " yaw output: " << yaw_adjust_cmd);
        w_ = yaw_adjust_cmd;
        AccelerationPlanning(cmd_set_vel);
        pbox_data_.swerve_mode = 2;
        if (w_ == 0)
          lock_RGB_joint_.setCommand(true);
        else
          lock_RGB_joint_.setCommand(false);
      }
      else if (yaw_adjust_flag_ == 3)
      {
        if (camera_init_)
        {
          auto &box = camera_data->boxes[0];
          if (box.center.x == -1.0 && box.center.y == 0.0)
          {
            ROS_INFO("camera lose basket");
            w_ = cmd_vel->angular.z; // 没看到篮筐先交给手操
          }
          else
          {
            w_ = basket_pid_.PIDCalculateByError(camera_offset_ - box.center.x);
            ROS_INFO_STREAM("error: " << 320.0 - box.center.x << " out: " << w_);
          }
          if (w_ == 0 && box.center.x != -1.0)
          {
            pbox_data_.lock_backet_flag = true;
            lock_RGB_joint_.setCommand(true);
          }
          else
          {
            pbox_data_.lock_backet_flag = false;
            lock_RGB_joint_.setCommand(false);
          }
        }
        else
        {
          w_ = cmd_vel->angular.z;
          lock_RGB_joint_.setCommand(false);
        }
        AccelerationPlanning(cmd_set_vel);

        pbox_data_.swerve_mode = 4;
      }
      else if (yaw_adjust_flag_ == 0 && remote_state->dribble_flag)
      {
        // if(!forward_flag)
        // {
        //   forward_flag = true;
        //   if(!Forward())return;
        // }
        // Yaw = imu_data->vector.z;
        AccelerationPlanning(dribble_vel);
        // w_ = yaw_pid_.YawAdjustCalculate(Yaw, 0.0);
        w_ = 0.0;
        pbox_data_.swerve_mode = 1;
        lock_RGB_joint_.setCommand(false);
      }
      else
      {
        w_ = cmd_vel->angular.z;
        AccelerationPlanning(cmd_set_vel);
        pbox_data_.swerve_mode = 1;
        pbox_data_.lock_backet_flag = false;
        forward_flag = false;
        lock_RGB_joint_.setCommand(false);
      }

      if (remote_state->brake_flag)
      {
        Parking();
        return;
      }

      double W = triangle_base_ / 2;

      if (remote_state->world_flag)
      {
        // double imu_yaw = imu_data->vector.z/180.0*M_PI;
        double imu_yaw = position_imu->yaw_angle;
        ROS_INFO_STREAM_THROTTLE(0.5,"yaw:  " << imu_yaw);
        double Bx = cos(imu_yaw) * cmd_vel_.linear_x + sin(imu_yaw) * cmd_vel_.linear_y;
        double By = -sin(imu_yaw) * cmd_vel_.linear_x + cos(imu_yaw) * cmd_vel_.linear_y;
        cmd_vel_.linear_x = Bx;
        cmd_vel_.linear_y = By;
        // ROS_INFO_STREAM("x: "<<cmd_set_vel->linear.x<<"y: "<<cmd_set_vel->linear.y);
      }

      if (cmd_vel_.linear_x != 0 || cmd_vel_.linear_y != 0 || w_ != 0)
      {
        swerve_cmd_pos_["one_front"] =
            std::atan2(cmd_vel_.linear_y + w_ * 0.244,
                       cmd_vel_.linear_x); // 前轮
        swerve_cmd_pos_["left_back"] =
            std::atan2(cmd_vel_.linear_y - w_ * 0.244,
                       cmd_vel_.linear_x - w_ * W); // 左后
        swerve_cmd_pos_["right_back"] =
            std::atan2(cmd_vel_.linear_y - w_ * 0.244,
                       cmd_vel_.linear_x + w_ * 0.244); // 右后
      }

      wheel_cmd_vel_["one_front"] =
          sqrt(cmd_vel_.linear_x * cmd_vel_.linear_x +
               (cmd_vel_.linear_y + w_ * 0.244) *
                   (cmd_vel_.linear_y + w_ * 0.244));
      wheel_cmd_vel_["left_back"] =
          sqrt((cmd_vel_.linear_x - w_ * W) * (cmd_vel_.linear_x - w_ * W) +
               (cmd_vel_.linear_y - w_ * 0.244) *
                   (cmd_vel_.linear_y - w_ * 0.244));
      wheel_cmd_vel_["right_back"] =
          sqrt((cmd_vel_.linear_x + w_ * W) * (cmd_vel_.linear_x + w_ * W) +
               (cmd_vel_.linear_y - w_ * 0.244) *
                   (cmd_vel_.linear_y - w_ * 0.244));

      // ROS_INFO_STREAM_THROTTLE(0.5,"front: "<<wheel_cmd_vel_["one_front"]<<"
      // left: "<<wheel_cmd_vel_["left_back"]<<"right:
      // "<<wheel_cmd_vel_["right_back"]); ROS_INFO_STREAM_THROTTLE(0.5,"front:
      // "<<swerve_cmd_pos_["one_front"]<<"left:
      // "<<swerve_cmd_pos_["left_back"]<<"right:
      // "<<swerve_cmd_pos_["right_back"]);
    }
  }

  void SwerveChassisController::AccelerationPlanning(geometry_msgs::Twist *cmd)
  {
    // double cmd_speed = sqrt(cmd->linear.x * cmd->linear.x + cmd->linear.y * cmd->linear.y);
    // double output_speed;
    ros::Time now_cmd_time = ros::Time::now();
    double dt = now_cmd_time.toSec() - last_cmd_time_;
    // if(!cmd_speed)
    // {
    //   output_speed = cmd_speed;
    //   cmd_vel_.linear_x = 0.0; // vx
    //   cmd_vel_.linear_y = 0.0; // vy
    //   last_speed_ = output_speed;
    //   last_cmd_time_ = now_cmd_time.toSec();
    //   return;
    // }

    if (cmd->linear.x - last_speed_.linear_x >= x_acceleration_ * dt)
    {
      cmd_vel_.linear_x = last_speed_.linear_x + x_acceleration_ * dt;
    }
    else if (cmd->linear.x - last_speed_.linear_x <= -x_acceleration_ * dt)
    {
      cmd_vel_.linear_x = last_speed_.linear_x - x_acceleration_ * dt;
    }
    else
      cmd_vel_.linear_x = cmd->linear.x;

    if (cmd->linear.y - last_speed_.linear_y >= y_acceleration_ * dt)
    {
      cmd_vel_.linear_y = last_speed_.linear_y + y_acceleration_ * dt;
    }
    else if (cmd->linear.y - last_speed_.linear_y <= -y_acceleration_ * dt)
    {
      cmd_vel_.linear_y = last_speed_.linear_y - y_acceleration_ * dt;
    }
    else
      cmd_vel_.linear_y = cmd->linear.y;

    last_speed_.linear_x = cmd_vel_.linear_x;
    last_speed_.linear_y = cmd_vel_.linear_y;
    last_cmd_time_ = now_cmd_time.toSec();

    /* */
    // double speed_unit[2] = {cmd->linear.y,
    //                         cmd->linear.x};     //[vy,vx] 横坐标y轴，纵坐标x轴
    // speed_unit[0] = speed_unit[0]/cmd_speed;  // 单位速度向量
    // speed_unit[1] = speed_unit[1]/cmd_speed;

    // // ROS_INFO_STREAM_THROTTLE(0.5,"dt: "<<dt);
    // // 加速变化过大
    // if (cmd_speed - last_speed_ > 0 &&
    //     fabs(cmd_speed - last_speed_) > forward_acceleration_*dt)
    //     {
    //       output_speed =
    //           last_speed_ + forward_acceleration_*dt;  // 计算加速后的速度
    //       ROS_INFO_STREAM_THROTTLE(0.5,"has used acceleration, output: "<<output_speed);

    //     }
    // else if (cmd_speed - last_speed_ < 0 &&
    //          fabs(cmd_speed - last_speed_) > reverse_acceleration_*dt)
    //          {
    //             output_speed = last_speed_ - reverse_acceleration_*dt;
    //             ROS_INFO_STREAM_THROTTLE(0.5,"has used acceleration output: "<<output_speed);
    //          }
    // else
    //   output_speed = cmd_speed;

    // // ROS_INFO_STREAM_THROTTLE(0.5,"has used acceleration, x: "<<cmd_speed);
    // cmd_vel_.linear_x = output_speed*speed_unit[1];  // vx
    // cmd_vel_.linear_y = output_speed*speed_unit[0];  // vy
    // // ROS_INFO_STREAM_THROTTLE(0.5,"has used acceleration, x: "<<cmd_vel_.linear_x);
    // last_speed_ = output_speed;
    // last_cmd_time_ = now_cmd_time.toSec();
  }

  void SwerveChassisController::MinorArcOptimzation()
  {
    for (auto &swerve_cmd : swerve_cmd_pos_)
    {
      std::string joint_name;
      joint_name = swerve_cmd.first;

      double swerve_error;
      swerve_error = swerve_cmd.second - swerve_pos_cur_[joint_name];

      AngleLimit(&swerve_error); // 将误差范围规划为(-pi,pi)
      wheel_reverse_ = -1.0;     //

      if (swerve_error > M_PI / 2 && swerve_error <= M_PI)
      {
        wheel_reverse_ = 1.0; // 轮向电机反转
        wheel_cmd_vel_[joint_name] = -wheel_cmd_vel_[joint_name];
        // swerve_cmd_pos_[joint_name] = swerve_pos_cur_[joint_name] +
        // swerve_error - M_PI;
        swerve_error = swerve_error - M_PI;
      }
      else if (swerve_error >= -M_PI && swerve_error < -M_PI / 2)
      {
        wheel_reverse_ = 1.0; // 轮向电机反转
        wheel_cmd_vel_[joint_name] = -wheel_cmd_vel_[joint_name];
        // swerve_cmd_pos_[joint_name] = swerve_pos_cur_[joint_name] +
        // swerve_error + M_PI;
        swerve_error = swerve_error + M_PI;
      }
      swerve_error_map_[joint_name] = swerve_error;
    }
  }

  void SwerveChassisController::SwerveInit()
  {

    // // position_output[0] =
    // // swerve_position_pid_["steer_front_joint"].PIDCalculateByError(swerve_error_map_["one_front"]);
    // std_msgs::Float32* vel = debug_buffer_.readFromRT();
    // swerve_cmd_map_["steer_front_joint"] =
    // swerve_increment_pid_["steer_front_joint"].PIDCalculate(swerve_vel_cur_["steer_front_joint"],vel->data);
    // // swerve_cmd_map_["steer_front_joint"] =
    // // swerve_increment_pid_["steer_front_joint"].PIDCalculate(swerve_vel_cur_["steer_front_joint"],swerve_position_pid_["steer_front_joint"].PIDCalculate(swerve_joint_pos_["steer_front_joint"].final_pos,vel->data));
    // swerve_joints_["steer_front_joint"].setCommand(swerve_cmd_map_["steer_front_joint"]);//
    // // 设置舵向电机的命令
    // ROS_INFO_STREAM("output:"<<swerve_cmd_map_["steer_front_joint"]);

    if (is_start_init_)
    {
      // 开始做舵轮初始化动作,以一个较慢的速度进行旋转
      for (auto &cur_vel : swerve_joints_)
      {
        std::string joint_name = cur_vel.first;

        bool swerve_io_state =
            steer_io_[joint_name].getNowState(); // 获取舵轮io状态

        // ROS_INFO_STREAM("IO:" << swerve_io_state);
        if (!swerve_io_state)
        {
          swerve_joint_pos_[joint_name].final_pos = light_door_skew_;
          steer_io_state_[joint_name] = swerve_io_state;
          // 立刻停下
          swerve_cmd_map_[joint_name] =
              swerve_increment_pid_[joint_name].PIDCalculate(
                  swerve_vel_cur_[joint_name], 0.0);
          ROS_INFO_STREAM("Swerve joint " << joint_name << " is finished");
        }
        else
        {
          if (!steer_io_state_[joint_name])
            swerve_cmd_map_[joint_name] = swerve_increment_pid_[joint_name].PIDCalculate(swerve_vel_cur_[joint_name], 0.0);
          else
            swerve_cmd_map_[joint_name] = swerve_increment_pid_[joint_name].PIDCalculate(swerve_vel_cur_[joint_name], steer_check_vel_);
        }
        cur_vel.second.setCommand(swerve_cmd_map_[joint_name]);
      }

      if (steer_io_state_["steer_front_joint"] == false &&
          steer_io_state_["steer_left_rear_joint"] == false &&
          steer_io_state_["steer_right_rear_joint"] == false)
      {
        is_init_ = true;        // 设置舵轮初始化完成
        is_start_init_ = false; // 重置开始初始化标志
        ROS_INFO("SwerveController: steer wheel initialized");
        Parking(); // 停车锁
      }
    }
    else
    {
      double pos_out[3];
      double out[3];
      double front_swerve = swerve_joint_pos_["steer_front_joint"].final_pos;
      double left_swerve = swerve_joint_pos_["steer_left_rear_joint"].final_pos;
      double right_swerve = swerve_joint_pos_["steer_right_rear_joint"].final_pos;
      AngleLimit(&front_swerve);
      AngleLimit(&left_swerve);
      AngleLimit(&right_swerve);

      pos_out[0] = swerve_position_pid_["steer_front_joint"].PIDCalculateByError((light_door_skew_ + 0.2 - front_swerve));
      pos_out[1] = swerve_position_pid_["steer_left_rear_joint"].PIDCalculateByError((light_door_skew_ + 0.2 - left_swerve));
      pos_out[2] = swerve_position_pid_["steer_right_rear_joint"].PIDCalculateByError((light_door_skew_ + 0.2 - right_swerve));

      out[0] = swerve_increment_pid_["steer_front_joint"].PIDCalculate(
          swerve_vel_cur_["steer_front_joint"], -1.0 * pos_out[0]);
      out[1] = swerve_increment_pid_["steer_left_rear_joint"].PIDCalculate(
          swerve_vel_cur_["steer_left_rear_joint"], -1.0 * pos_out[1]);
      out[2] = swerve_increment_pid_["steer_right_rear_joint"].PIDCalculate(
          swerve_vel_cur_["steer_right_rear_joint"], -1.0 * pos_out[2]);

      swerve_joints_["steer_front_joint"].setCommand(out[0]);
      swerve_joints_["steer_left_rear_joint"].setCommand(out[1]);
      swerve_joints_["steer_right_rear_joint"].setCommand(out[2]);

      if (pos_out[0] == 0.0 && pos_out[1] == 0.0 && pos_out[2] == 0.0)
      {
        if (swerve_vel_cur_["steer_front_joint"] == 0.0 && swerve_vel_cur_["steer_left_rear_joint"] == 0.0 && swerve_vel_cur_["steer_right_rear_joint"] == 0.0)
          is_start_init_ = true;
      }
    }
  }

  void SwerveChassisController::Parking()
  {
    // 绝对驻车姿势
    swerve_cmd_pos_["one_front"] = 0.0;
    swerve_cmd_pos_["left_back"] = M_PI / 2.0;
    swerve_cmd_pos_["right_back"] = M_PI / 2.0;
    wheel_cmd_vel_["one_front"] = 0.0;
    wheel_cmd_vel_["left_back"] = 0.0;
    wheel_cmd_vel_["right_back"] = 0.0;
  }

  bool SwerveChassisController::Forward()
  {
    if (fabs(swerve_joint_pos_["steer_front_joint"].final_pos) >= 0.3 || fabs(swerve_joint_pos_["steer_left_rear_joint"].final_pos) >= 0.3 || fabs(swerve_joint_pos_["steer_right_rear_joint"].final_pos) >= 0.3)
    {
      swerve_cmd_pos_["one_front"] = 0.0;
      swerve_cmd_pos_["left_back"] = 0.0;
      swerve_cmd_pos_["right_back"] = 0.0;
      wheel_cmd_vel_["one_front"] = 0.0;
      wheel_cmd_vel_["left_back"] = 0.0;
      wheel_cmd_vel_["right_back"] = 0.0;
      return false;
    }
    else
      return true;

    ROS_INFO_STREAM("front: " << swerve_joint_pos_["steer_front_joint"].final_pos << "left_back:" << swerve_joint_pos_["steer_left_rear_joint"].final_pos << "right_back" << swerve_joint_pos_["steer_right_rear_joint"].final_pos);
  }

  void SwerveChassisController::cmdVelCallback(
      const geometry_msgs::TwistConstPtr &msg)
  {
    if (is_init_)
    {
      std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
      cmd_vel_buffer_.writeFromNonRT(*msg);
    }
  }

  void SwerveChassisController::CameraOffset(double camera_offset)
  {
    if (!camera_offset_finish_)
    {
      camera_offset_ = camera_offset;
    }
    return;
  }

  void SwerveChassisController::BasketOffset(const std::vector<double> &arr_x, const std::vector<double> &arr_y, const std::vector<double> &arr_yaw)
  {
    if (arr_x.size() != arr_y.size())
    {
      ROS_ERROR("basket set lack of position data");
      return;
    }
    else
    {
      if (arr_x.size() != arr_yaw.size())
      {
        ROS_ERROR("basket set lack of yaw data");
        return;
      }
    }

    if (arr_x.size() < 2)
    {
      ROS_INFO("just one set data");
      return;
    }

    // basket_position_x_.clear();
    // basket_position_y_.clear();

    // for (int i = arr_x.size()-1; i >=1; i--)
    // {
    //   if(fabs(arr_yaw[i])==M_PI/2 || fabs(arr_yaw[i])==M_PI)
    //   {
    //     ROS_ERROR("YAW angle is error");
    //   }

    //   double k1 = tan(arr_yaw[i]);
    //   double k2 = tan(arr_yaw[i-1]);

    //   double x = (k1*arr_x[i]-arr_y[i]-k2*arr_x[i-1]+arr_y[i-1])/(k1-k2);
    //   double y = k1*x + (arr_y[i]-k1*arr_x[i]);
    //   basket_position_x_.push_back(x);
    //   basket_position_y_.push_back(y);
    // }

    // if(basket_position_x_.size()<2)
    // {
    //   basket_x_ = basket_position_x_[0];
    //   basket_y_ = basket_position_y_[0];
    // }
    // else
    // {
    // }

    calculateCenter();
  }

  void SwerveChassisController::OdomCallback(
      const nav_msgs::OdometryConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    odom_buffer_.writeFromNonRT(*msg);
    if (!radar_init_)
    {
      radar_init_count_++;
      dx_ += msg->pose.pose.position.x;
      dy_ += msg->pose.pose.position.y;
      if (radar_init_count_ == 10)
      {
        dx_ /= 10.0;
        dy_ /= 10.0;
        initialParams.dx = dx_;
        initialParams.dy = dy_;
        radar_init_ = true;
      }
    }
  }

  void SwerveChassisController::PartnerCallback(
      const ros_ecat_msgs::PositionSensorMsgConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(partner_mutex_);
    partner_buffer_.writeFromNonRT(*msg);
  }

  void SwerveChassisController::initCallback(
      const std_msgs::String::ConstPtr &msg)
  {
    if (msg->data == "START")
    {
      ROS_INFO("SimpleSwerveController: init callback received");
      // 进行舵轮舵向电机的初始化
      {
        std::lock_guard<std::mutex> lock(init_mutex_);
        is_init_ = false; // 重置舵轮初始化状态
        steer_io_state_["steer_front_joint"] = true;
        steer_io_state_["steer_left_rear_joint"] = true;
        steer_io_state_["steer_right_rear_joint"] = true;
        // is_start_init_ = true;
      }
    }
  }

  void SwerveChassisController::calculateCenter()
  {

    // --- 最小二乘法计算 ---
    double sum_A_sq = 0.0f;
    double sum_B_sq = 0.0f;
    double sum_AB = 0.0f;
    double sum_AC = 0.0f;
    double sum_BC = 0.0f;

    for (uint8_t i = 0; i < basket_arr_x_.size(); i++)
    {
      // ROS的yaw已经是标准数学角度，直接转弧度即可

      double yaw_rad = basket_arr_yaw_[i];

      // 基于ROS坐标系 (x轴为0°, 逆时针为正) 建立直线方程
      // 方向向量 d = (cos(yaw), sin(yaw))
      // 法向量 n = (-sin(yaw), cos(yaw))
      // 直线方程: -sin(yaw)*(X - x) + cos(yaw)*(Y - y) = 0
      // 整理为 Ax + By = C:
      // A = -sin(yaw), B = cos(yaw)
      // C = -sin(yaw)*robot_x + cos(yaw)*robot_y

      double cos_yaw = std::cos(yaw_rad);
      double sin_yaw = std::sin(yaw_rad);

      double A = -sin_yaw;
      double B = cos_yaw;
      double C = A * basket_arr_x_[i] + B * basket_arr_y_[i];

      // 累加最小二乘法矩阵的元素
      sum_A_sq += A * A;
      sum_B_sq += B * B;
      sum_AB += A * B;
      sum_AC += A * C;
      sum_BC += B * C;
    }

    // 求解 2x2 线性方程组
    double det = sum_A_sq * sum_B_sq - sum_AB * sum_AB;

    if (std::abs(det) < 1e-6f)
    {
      ROS_ERROR("the basket line is ");
      return; // 矩阵奇异，所有线平行
    }

    double inv_det = 1.0f / det;
    basket_x_ = inv_det * (sum_B_sq * sum_AC - sum_AB * sum_BC);
    basket_y_ = inv_det * (sum_A_sq * sum_BC - sum_AB * sum_AC);

    return;
  }

  void SwerveChassisController::debugCallback(
      const std_msgs::Float32::ConstPtr &msg)
  {
    debug_buffer_.writeFromNonRT(*msg);
  }

  void SwerveChassisController::vescCallback(
      const ros_ecat_msgs::VescEcatRosMsg::ConstPtr &msg)
  {
    ROS_INFO_STREAM_THROTTLE(0.5, "front: " << msg->velocity[0]
                                            << "left: " << msg->velocity[1]
                                            << "right: " << msg->velocity[2]);
  }

  void SwerveChassisController::imuCallback(
      const geometry_msgs::Vector3Stamped::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    imu_buffer_.writeFromNonRT(*msg);
    if (!imu_yaw_init)
    {
      imu_init_count_++;
      by_ += msg->vector.z;
      if (imu_init_count_ == 10)
      {
        by_ /= 10.0;
        initialParams.by = by_;
        imu_yaw_init = true;
      }
    }
  }

  void SwerveChassisController::RemoteStateCallback(
      const ros_ecat_msgs::RemoteStateConstPtr &msg)
  {
    remote_state_buffer_.writeFromNonRT(*msg);
  }

  void SwerveChassisController::CameraCallback(
      const vision_msgs::BoundingBox2DArrayConstPtr &msg)
  {
    camera_buffer_.writeFromNonRT(*msg);
    camera_init_ = true;
  }

  void SwerveChassisController::DribbleVelCallback(const geometry_msgs::TwistConstPtr &msg)
  {
    dribble_vel_buffer_.writeFromNonRT(*msg);
  }

  void SwerveChassisController::PositionCallback(const ros_ecat_msgs::PositionSensorMsgConstPtr &msg)
  {
    position_buffer_.writeFromNonRT(*msg);
  }

  void SwerveChassisController::stopping(const ros::Time & /*time*/)
  {
    ROS_INFO("SwerveChassisController: stopping");
    for (auto &cur_vel : swerve_joints_)
    {
      cur_vel.second.setCommand(0.0);
    }

    for (auto &cur_vel : wheel_joints_)
    {
      cur_vel.second.setCommand(0.0);
    }
  }

} // namespace chassis_controller
PLUGINLIB_EXPORT_CLASS(chassis_controller::SwerveChassisController,
                       controller_interface::ControllerBase)
