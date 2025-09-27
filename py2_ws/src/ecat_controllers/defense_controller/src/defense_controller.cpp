/**
 * @file defense_controller.cpp
 * @author py <997276894@qq.com>
 * @brief  防守控制器
 * @version 0.1
 * @date  2025-07-15 17:51:01 
 * 
 * @copyright Copyright (c)  2025 
 * 
 * @attention  
 * @note  
 */


#include <defense_controller/defense_controller.h>
#include <pluginlib/class_list_macros.hpp>

namespace defense_controller{

bool DefenseController::init(hardware_interface::EffortJointInterface *effort_joint_interface,ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
{
    controller_nh_ = controller_nh;

    if(!controller_nh.getParam("rise_speed",rise_speed_))
    {
        ROS_INFO("rise speed has not set");
        return false;
    }

    if(!controller_nh.getParam("action_time",action_time_))
    {
        ROS_INFO("action time has not set");
        return false;
    }
    
    if(!controller_nh.getParam("keep_speed",keep_speed_))
    {
        ROS_INFO("keep speed has not set");
        return false;
    }

    XmlRpc::XmlRpcValue motors;
    controller_nh.getParam("motor",motors);
    for(auto &motor : motors)
    {
        ros::NodeHandle nh = ros::NodeHandle(controller_nh,"motor/" + motor.first);
        std::string joint_name;
        if(!nh.getParam("motor_joint",joint_name))
        {
            ROS_ERROR("joint name not set");
        }

        // ROS_INFO_THROTTLE("joint name: "<<joint_name);
        joints_map_.insert(std::make_pair(joint_name,hardware_interface::JointHandle()));
        joints_map_[joint_name] = effort_joint_interface->getHandle(joint_name);

        XmlRpc::XmlRpcValue pid;
        if (!nh.getParam("increment_pid", pid)) {
            ROS_ERROR("Joint motor pid not set");
            return false;
        }
        double p = static_cast<double>(pid["p"]);
        double i = static_cast<double>(pid["i"]);
        double d = static_cast<double>(pid["d"]);
        double i_max = static_cast<double>(pid["i_max"]);
        double output_max = static_cast<double>(pid["output_max"]);
        double deadzone = static_cast<double>(pid["deadzone"]);
        motor_pid_[joint_name] = ctrl_common::IncrementalPID();
        motor_pid_[joint_name].init(p,i,d,i_max,output_max,deadzone);        
    }

    remote_state_sub_=root_nh.subscribe<ros_ecat_msgs::RemoteState>(
        "/remote_state", 1, &DefenseController::RemoteStateCallback, this);

    ros::Time init_time = ros::Time::now();
    action_start_time_ = init_time.toSec();
    return true;
}

void DefenseController::update(const ros::Time &time,const ros::Duration &period) 
{
    {
        std::lock_guard<std::mutex> lock(remote_mutex_);
        ros_ecat_msgs::RemoteState* remote_state = remote_state_buffer_.readFromRT();
        double motor_output[2];
        double now_motor_speed;
        ros::Time now_time = ros::Time::now();

        // ROS_INFO_STREAM("remote: "<<remote_state->net_rise<<"action_flag: "<<action_flag_);
        // bool net_rising = false;
        // net_rising = remote_state->net_rise;

        if(remote_state->net_rise && remote_state->net_rise!=action_flag_)
        {
            if(timing_idle_)
            {
                action_start_time_ = now_time.toSec();
                timing_idle_ = false;
            }

            if((now_time.toSec() - action_start_time_) >= action_time_)
            {
                cmd_vel_ = keep_speed_;
                timing_idle_ = true; //恢复计时空闲状态
                action_flag_ = remote_state->net_rise;
            }
            else cmd_vel_ = rise_speed_;
            
            
        }
        else if(!remote_state->net_rise && remote_state->net_rise!=action_flag_)
        {
            if(timing_idle_)
            {
                action_start_time_ = now_time.toSec();
                timing_idle_ = false; //计时进行状态
            }

            if((now_time.toSec() - action_start_time_) >= action_time_)
            {
                cmd_vel_ = 0.0;
                timing_idle_ = true; //恢复计时空闲状态
                action_flag_ = remote_state->net_rise;
            return;
            }
            else cmd_vel_ = -rise_speed_;
        }
        if ((cmd_vel_ == rise_speed_ && !remote_state->net_rise && remote_state->net_rise == action_flag_) || 
            (cmd_vel_ == -rise_speed_ && remote_state->net_rise && remote_state->net_rise == action_flag_))
        {
            cmd_vel_ = 0.0;
            timing_idle_ = true;
            action_flag_ = !remote_state->net_rise;
        }

        now_motor_speed = joints_map_["rising_left_joint"].getVelocity();
        joints_map_["rising_left_joint"].setCommand(motor_pid_["rising_left_joint"].PIDCalculate(now_motor_speed, -cmd_vel_)*3.33);

        now_motor_speed = joints_map_["rising_right_joint"].getVelocity();
        joints_map_["rising_right_joint"].setCommand(motor_pid_["rising_right_joint"].PIDCalculate(now_motor_speed, cmd_vel_)*3.33);


        
    }
}


void DefenseController::RemoteStateCallback(const ros_ecat_msgs::RemoteStateConstPtr &msg)
{
    remote_state_buffer_.writeFromNonRT(*msg);
    std::lock_guard<std::mutex> lock(remote_mutex_);
}


void DefenseController::stopping(const ros::Time & /*time*/) 
{
    ROS_INFO("defense controller: stopping");
    joints_map_["rising_left_joint"].setCommand(0.0);
    joints_map_["rising_right_joint"].setCommand(0.0);
}

}
PLUGINLIB_EXPORT_CLASS(defense_controller::DefenseController,
                       controller_interface::ControllerBase)