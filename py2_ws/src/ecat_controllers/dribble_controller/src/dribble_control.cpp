/**
 * @file dribble_control.cpp
 * @author py <997276894@qq.com>
 * @brief  运球控制
 * @version 0.1
 * @date  2025-07-23
 * 
 * @copyright Copyright (c)  2025 
 * 
 * @attention  
 * @note  
 */
#include <dribble_controller/dribble_control.h>

namespace dribble_control
{

bool DribbleControl::init(ros::NodeHandle& nh)
{
    if(!nh.getParam("forward_speed",forward_speed_))
    {
        ROS_ERROR("forward speed not set");
        return false;
    }

    if(!nh.getParam("back_speed",back_speed_))
    {
        ROS_ERROR("back speed not set");
        return false;
    }

    if(!nh.getParam("forward_time",forward_time_))
    {
        ROS_ERROR("fordward time not set");
        return false;
    }

    launch_state_sub_=nh.subscribe<std_msgs::UInt8>(
        "/launch_state", 1, &DribbleControl::LaunchCallback, this);

    cmd_vel_pub_ =
       std::make_shared<any_node::ThreadedPublisher<geometry_msgs::Twist>>(
           nh.advertise<geometry_msgs::Twist>("/dribble_control/cmd_vel", 10), 50, true);

    remote_state_sub_ = nh.subscribe<ros_ecat_msgs::RemoteState>(
        "/remote_state", 1, &DribbleControl::RemoteStateCallback, this);

    dribble_shoot_pub_ = std::make_shared<any_node::ThreadedPublisher<std_msgs::Bool>>(
        nh.advertise<std_msgs::Bool>("/dribble_control/shoot_control", 10), 50, true);

    update_worker_ = std::make_shared<any_worker::Worker>(
        "updateWorker", 0.001,
        std::bind(&DribbleControl::updateWorkerCb, this, std::placeholders::_1));
    update_worker_->start(45);
    return true;
}

void DribbleControl::update(const ros::Time& time)
{
    std_msgs::UInt8 *launch_state = launch_state_buffer_.readFromRT();
    ros_ecat_msgs::RemoteState *remote_state = remote_state_buffer_.readFromRT();
    std_msgs::Bool shoot_flag;
    geometry_msgs::Twist cmd_vel;
    if(remote_state->dribble_flag)
    {
        switch (dribble_state_)
        {
        case BACK_MOVE:
            if(!time_cal_)
            {
                last_time_ = time.toSec();
                time_cal_ = true;
            }
            cmd_vel.linear.x = -back_speed_;

            if((time.toSec() - last_time_)>1.0) //后退一秒
            {
                time_cal_ = false;
                shoot_flag.data = true;
                dribble_state_ = WAIT_SHOOT;
            }
            break;

        case WAIT_SHOOT:
            shoot_flag.data = true;
            //处于move/wait_reset
            if(launch_state->data==2 || launch_state->data==3)
            {
                cmd_vel.linear.x = 0;
                dribble_state_ = WAIT_MOVE;
            }
            else cmd_vel.linear.x = -back_speed_;
            break;
        
            case WAIT_MOVE:
                if(!time_cal_)
                {
                    last_time_ = time.toSec();
                    time_cal_ = true;
                }
                cmd_vel.linear.x = 0.0;

                if((time.toSec() - last_time_)>0.7)
                {
                    time_cal_ = false;
                    dribble_state_ = FORWARD_MOVE;
                }
            break;

            break;
        
        case FORWARD_MOVE:
            if(!time_cal_)
            {
                last_time_ = time.toSec();
                time_cal_ = true;
            }
            cmd_vel.linear.x = forward_speed_;

            if((time.toSec() - last_time_)>forward_time_)
            {
                time_cal_ = false;
                dribble_state_ = STOP;
            }
            break;
        
        case STOP:
            if(!time_cal_)
            {
                last_time_ = time.toSec();
                time_cal_ = true;
            }
            cmd_vel.linear.x = 0.0;
            if((time.toSec() - last_time_)>1.0) //停一秒
            {
                time_cal_ = false;
                dribble_state_ = BACK_MOVE;
            }
        default:
            break;
        }
    }
    else 
    {
        dribble_state_ = BACK_MOVE;
        cmd_vel.linear.x = 0.0;
        time_cal_ = false;
    }
    cmd_vel_pub_->publish(cmd_vel);
    dribble_shoot_pub_->publish(shoot_flag);
    
}

bool DribbleControl::updateWorkerCb(const any_worker::WorkerEvent& event) {
    update(ros::Time::now());
    // unpack();
    return true;
}

void DribbleControl::LaunchCallback(const std_msgs::UInt8::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(launch_state_mutex_);
    launch_state_buffer_.writeFromNonRT(*msg);
}

void DribbleControl::RemoteStateCallback(const ros_ecat_msgs::RemoteStateConstPtr &msg)
{
    remote_state_buffer_.writeFromNonRT(*msg);
}

}