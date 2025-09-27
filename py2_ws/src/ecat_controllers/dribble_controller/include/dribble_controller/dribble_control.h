#pragma once

#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros_ecat_msgs/RemoteState.h>
#include <any_node/ThreadedPublisher.hpp>
#include <any_worker/Worker.hpp>

namespace dribble_control{

class DribbleControl
{

public:
    bool init(ros::NodeHandle& nh);
    void update(const ros::Time& time);

    void LaunchCallback(const std_msgs::UInt8::ConstPtr &msg);
    void RemoteStateCallback(const ros_ecat_msgs::RemoteStateConstPtr &msg);
    bool updateWorkerCb(const any_worker::WorkerEvent& event);

private:
    std::shared_ptr<any_worker::Worker> update_worker_;
    any_node::ThreadedPublisherPtr<geometry_msgs::Twist> cmd_vel_pub_;
    any_node::ThreadedPublisherPtr<std_msgs::Bool> dribble_shoot_pub_;
    ros::Subscriber launch_state_sub_;
    ros::Subscriber remote_state_sub_;
    
    realtime_tools::RealtimeBuffer<std_msgs::UInt8> launch_state_buffer_;
    realtime_tools::RealtimeBuffer<ros_ecat_msgs::RemoteState> remote_state_buffer_;

    std::mutex launch_state_mutex_;

    double forward_speed_ , back_speed_ , forward_time_;
    double last_time_;
    uint8_t dribble_state_;
    bool time_cal_ = {false};//计时
    enum DribbleState {
        BACK_MOVE,
        WAIT_SHOOT,
        WAIT_MOVE,
        FORWARD_MOVE,
        STOP
    };
};
}