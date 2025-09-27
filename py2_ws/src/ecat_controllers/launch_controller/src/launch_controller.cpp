/**
 * @file launch_controller.cpp
 * @author py <997276894@qq.com>
 * @brief  发射机构控制器
 * @version 0.1
 * @date  2025-07-17 19:25:50 
 * 
 * @copyright Copyright (c)  2025 
 * 
 * @attention  
 * @note  
 */

#include "launch_controller/launch_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace launch_controller
{

bool LaunchController::init(hardware_interface::RobotHW *robot_hw,ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
{
    controller_nh_ = controller_nh;
    
    if(!controller_nh.getParam("mechanical_zero_position",zero_pos_))
    {
        ROS_INFO("Mechanical zero position has not set");
        return false;
    }

    if(!controller_nh.getParam("max_distance",max_distance_))
    {
        ROS_INFO("Max distance has not set");
        return false;
    }

    if(!controller_nh.getParam("reset_time",reset_time_))
    {
        ROS_INFO("reset time has not set");
        return false;
    }

    if(!controller_nh.getParam("speed_offset",speed_offset_))
    {
        ROS_INFO("speed offset has not set");
        return false;
    }

    if(!controller_nh.getParam("dribble_distance",dribble_distance_))
    {
        ROS_INFO("dribble distance has not set");
        return false;
    }

    std::string joint_name;
    if(!controller_nh.getParam("encoder_joint",joint_name))
    {
        ROS_INFO("encoder joint has not set");
        return false;
    }
    encoder_joint_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(joint_name);

    if(!controller_nh.getParam("motor_joint",joint_name))
    {
        ROS_INFO("motor joint has not set");
        return false;
    }
    motor_joint_ = robot_hw->get<hardware_interface::EffortJointInterface>()->getHandle(joint_name);

    if(!controller_nh.getParam("trigger_joint",joint_name))
    {
        ROS_INFO("trigger joint has not set");
        return false;
    }
    trigger_joint_ = robot_hw->get<ecat_slave_hw::GpioCommandInterface>()->getHandle(joint_name);

    if(!controller_nh.getParam("RGB_joint",joint_name))
    {
        ROS_INFO("RGB joint has not set");
        return false;
    }
    RGB_joint_ = robot_hw->get<ecat_slave_hw::GpioCommandInterface>()->getHandle(joint_name);

    XmlRpc::XmlRpcValue pid;
    if(!controller_nh.getParam("position_pid", pid)) 
    {
        ROS_WARN("PID not set");
        return false;
    } 
    else 
    {
        double yaw_p = static_cast<double>(pid["p"]);
        double yaw_i = static_cast<double>(pid["i"]);
        double yaw_d = static_cast<double>(pid["d"]);
        double yaw_i_max = static_cast<double>(pid["i_max"]);
        double yaw_output_max = static_cast<double>(pid["output_max"]);
        death_zone_ = static_cast<double>(pid["deadzone"]);
        position_pid_.init(yaw_p, yaw_i, yaw_d, yaw_i_max, yaw_output_max, death_zone_);
    }


    remote_state_sub_=root_nh.subscribe<ros_ecat_msgs::RemoteState>(
        "/remote_state", 1, &LaunchController::RemoteStateCallback, this);

    odom_sub_ = root_nh.subscribe<nav_msgs::Odometry>(
        "/odom", 1, &LaunchController::OdomCallback, this);
    camera_sub_ = root_nh.subscribe<vision_msgs::BoundingBox2DArray>(
        "/yolo/detections", 1, &LaunchController::CameraCallback, this);
    distance_cmd_sub_ = root_nh.subscribe<std_msgs::Float32MultiArray>(
        "/distance_cmd", 1, &LaunchController::DistanceCmdCallback, this);

    dribble_shoot_pub_ = root_nh.subscribe<std_msgs::Bool>(
        "/dribble_control/shoot_control", 1, &LaunchController::DribbleCallback, this);

    launch_state_pub_ = std::make_shared<any_node::ThreadedPublisher<std_msgs::UInt8>>(
        root_nh.advertise<std_msgs::UInt8>("/launch_state", 1), 50, true);

    radar_cmd_pub = std::make_shared<any_node::ThreadedPublisher<std_msgs::Float32>>(
        root_nh.advertise<std_msgs::Float32>("/radar_shoot_cmd", 1), 50, true);

    camera_cmd_pub = std::make_shared<any_node::ThreadedPublisher<std_msgs::Float32>>(
        root_nh.advertise<std_msgs::Float32>("/camera_shoot_cmd", 1), 50, true);

    init_state_sub_ = root_nh.subscribe<ros_ecat_msgs::InitState>("/init_state", 1, &LaunchController::InitStateCallback, this);

    return true;
    
}

void LaunchController::update(const ros::Time &time, const ros::Duration &period)
{
    encoder_data_ = encoder_joint_.getPosition();
    ros_ecat_msgs::RemoteState *remote_state = remote_state_buffer_.readFromRT();
    std_msgs::Bool *dribble_shoot = dribble_buffer_.readFromRT();


    switch(launch_state_)
    {
        case RESET:
            trigger_joint_.setCommand(true); //扳机保持释放
            RGB_joint_.setCommand(false);
            if(encoder_data_!=0)
            {
                target_distance_ = zero_pos_;
            }
            else
            {
                target_distance_ = 0.0f;
            }

            //防止冲顶
            if(encoder_data_ <= -0.016)
            {
                target_distance_ = encoder_data_;
            }

            //判断是否回复到位
            if(fabs(target_distance_ - encoder_data_)<=0.005)
            {
                trigger_joint_.setCommand(false); //扳机立起,卡住发射机构
                
                target_distance_ = encoder_data_;
                // position_pid_.IntegrationClean();
                position_pid_.IntegrationClean();
                //运球模式下的出射
                if(dribble_shoot->data && remote_state->dribble_flag)
                {
                    launch_state_ = MOVE;
                    target_distance_ = dribble_distance_;
                }
                else if(remote_state->shoot_flag)//射球模式
                {
                    launch_state_ = MOVE;
                    if(remote_state->camera_shoot)target_distance_ =  camera_distance_;
                    else target_distance_ = radar_distance_;
                }

                motor_joint_.setCommand(0.0);
                // ROS_INFO("yes yes yes oh yes");
                return;
            }
        break;

        case MOVE:
            
            //防止拉伸过多
            if(target_distance_>=max_distance_ || target_distance_ <=0.001)
            {
                target_distance_ = 0.0;
                launch_state_ = RESET;
                ROS_WARN("Target distance error!");
                break;
            }
            RGB_joint_.setCommand(true);
            PointCheck(time);
            // if(fabs(encoder_data_ - target_distance_)<=death_zone_)
            // {
            //     launch_state_ = SHOOT;
                
            // }
        break;

        case SHOOT:
            trigger_joint_.setCommand(true); //扳机释放，发射
            RGB_joint_.setCommand(false);
            launch_state_ = WAIT_RESET;
            brake_time_ = time.toSec();
        break;

        case WAIT_RESET:
            if((time.toSec()-brake_time_)>=reset_time_)launch_state_ = RESET;
        break;
    }

    out_put_ = position_pid_.PIDCalculate(encoder_data_,target_distance_);

    
    if(fabs(out_put_)<=speed_offset_)
    {
        if(out_put_)out_put_+=speed_offset_;
        else out_put_-=speed_offset_;
    }
    
    //something checking measures

    //aviod motor disconnected
    DisconnectProction(time);

    //avoid encoder data error
    if(encoder_data_>0.40 || encoder_data_<-0.05)
    {
        out_put_=0;
        motor_joint_.setCommand(out_put_);
        ROS_WARN_STREAM("Encoder data error: "<<encoder_data_);
        return;
    }
    // ROS_INFO_STREAM("encoder: "<<encoder_data_<<"out: "<<out_put_<<"pos_distance: "<<pos_distance_);
    motor_joint_.setCommand(out_put_);

    std_msgs::UInt8 state;
    state.data = launch_state_;
    launch_state_pub_->publish(state);//发送状态
    // motor_joint_.setCommand(3.14);
}

void LaunchController::PointCheck(const ros::Time &time)
{
    if(!check_flag_)
    {
        check_time_ = time.toSec();
        check_flag_ = true;
    }
    
    if((time.toSec()-check_time_)<=0.05)
    {
        if(fabs(encoder_data_ - target_distance_)>death_zone_)
        {
            check_flag_ = false;
            return;
        }
    }
    else
    {
        launch_state_ = SHOOT;
        check_flag_ = false;
    }
}

void LaunchController::DisconnectProction(const ros::Time &time)
{
    if(out_put_>speed_offset_ || out_put_<-speed_offset_)
    {
        if(last_position_==encoder_data_ && last_velocity_==motor_joint_.getVelocity())
        {
            if(last_silence_time_==0.0)last_silence_time_ = time.toSec();
            if((time.toSec()-last_silence_time_)>=2.0)
            {
                out_put_=0;
                launch_state_ = RESET;
                position_pid_.IntegrationClean();
                ROS_INFO("Shooting motor is disconnected!");
            }
        }
        else
        {
            last_silence_time_ = 0.0;
        }
    }
    else last_silence_time_ = 0.0;

    last_position_ = encoder_data_;
    last_velocity_ = motor_joint_.getVelocity();
    
}

void LaunchController::RemoteStateCallback(const ros_ecat_msgs::RemoteStateConstPtr &msg)
{
    remote_state_buffer_.writeFromNonRT(*msg);
}

void LaunchController::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    ros_ecat_msgs::InitState *init_state_msg =init_state_buffer_.readFromRT();
  double x_radar = msg->pose.pose.position.x;
  double y_radar = msg->pose.pose.position.y;
  double distance = sqrt(std::pow((x_radar-init_state_msg->basket_x),2)+std::pow(y_radar-init_state_msg->basket_y,2));
//   double distance = sqrt(std::pow(x_radar,2)+std::pow(y_radar,2));
  radar_distance_ = 3.5816 * distance + 7.6434;
//   radar_distance_ = 3.52*(sqrt(std::pow((x_radar-init_state_msg->basket_x),2)+std::pow(y_radar-init_state_msg->basket_y,2))) + 5.51;//投篮曲线
  radar_distance_ = radar_distance_ * 0.01;
  ROS_INFO_STREAM("pos_distance: "<<distance);
//   ROS_INFO_STREAM("radar_cmd: "<<radar_distance_);

  std_msgs::Float32 radar_msg;
  radar_msg.data = radar_distance_;
  radar_cmd_pub->publish(radar_msg);
}

void LaunchController::CameraCallback(const vision_msgs::BoundingBox2DArrayConstPtr &msg)
{
    auto& box = msg->boxes[0];
    if(!(box.center.x==-1.0 && box.center.y==0.0))
    {
        // pos_distance_ = (-0.0174*box.center.y + 11.0261)/(-0.2392*box.center.y + 100.0) + 0.004;
        // camera_distance_  = 0.0048*std::exp(0.01*box.center.y)+0.1121;
        camera_distance_ = 0.00000005194 * std::pow(box.center.y,3) - 0.00003812 * std::pow(box.center.y,2) + 0.01674 * box.center.y + 10.44;
        camera_distance_ = 0.01 * camera_distance_;
        // ROS_INFO_STREAM("camera_cmd: "<<camera_distance_);
    }
    else camera_distance_ = 0.0;

    std_msgs::Float32 camera_msg;
    camera_msg.data = camera_distance_;
    camera_cmd_pub->publish(camera_msg);
}

void LaunchController::DistanceCmdCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
{
    radar_distance_= msg->data[0];
    camera_distance_= msg->data[1];
}

void LaunchController::DribbleCallback(const std_msgs::Bool::ConstPtr &msg)
{
    dribble_buffer_.writeFromNonRT(*msg);
}

void LaunchController::InitStateCallback(const ros_ecat_msgs::InitState::ConstPtr& msg)
{
  init_state_buffer_.writeFromNonRT(*msg);
}

void LaunchController::stopping(const ros::Time & /*time*/)
{
    motor_joint_.setCommand(0.0);
}
}

PLUGINLIB_EXPORT_CLASS(launch_controller::LaunchController,
                       controller_interface::ControllerBase)
