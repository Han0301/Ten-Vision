/**
 * @file launch_controller.cpp
 * @author py <997276894@qq.com>
 * @brief  发射机构控制器
 * @version 0.1
 * @date  2025-07-17 
 * 
 * @copyright Copyright (c)  2025 
 * 
 * @attention  
 * @note  
 */
#pragma once
#include <ros/ros.h>
#include <XmlRpcValue.h>
#include <unordered_map>
#include <control_msgs/JointControllerState.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/Twist.h>
#include <hardware_interface/joint_command_interface.h>
#include <rc_ecat_master/ecat_hardware_interface/gpio/GpioInterface.h>
#include <controller_interface/multi_interface_controller.h>
#include <realtime_tools/realtime_buffer.h>
#include <std_msgs/Float32MultiArray.h>
#include <mutex>
#include <cmath>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <nav_msgs/Odometry.h>
#include <vision_msgs/BoundingBox2DArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>
#include <ros_ecat_msgs/RemoteState.h>
#include <ros_ecat_msgs/InitState.h>
#include <ros_ecat_msgs/PositionSensorMsg.h>
#include <any_node/ThreadedPublisher.hpp>

#include "ctrl_common/PID/PositionPID.h"


namespace launch_controller{

/*
  后续可以使用多接口控制器，只要换个继承就行了
*/
class LaunchController : public controller_interface::MultiInterfaceController<ecat_slave_hw::GpioCommandInterface,hardware_interface::EffortJointInterface>
{
public:
    LaunchController() = default;
    ~LaunchController() override = default;
    bool init(hardware_interface::RobotHW *robot_hw,
        ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

    void update(const ros::Time &time, const ros::Duration &period) override;

    void PointCheck(const ros::Time &time);
    void DisconnectProction(const ros::Time &time);

    void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void CameraCallback(const vision_msgs::BoundingBox2DArrayConstPtr &msg);
    void DistanceCmdCallback(const std_msgs::Float32MultiArray::ConstPtr &msg);
    void RemoteStateCallback(const ros_ecat_msgs::RemoteStateConstPtr &msg);
    void InitStateCallback(const ros_ecat_msgs::InitStateConstPtr &msg);
    void DribbleCallback(const std_msgs::Bool::ConstPtr &msg);
    void stopping(const ros::Time & /*time*/) override;

private:
    ros::NodeHandle controller_nh_;

    hardware_interface::JointHandle encoder_joint_;
    hardware_interface::JointHandle motor_joint_;
    ecat_slave_hw::GpioCommandHandle trigger_joint_;
    ecat_slave_hw::GpioCommandHandle RGB_joint_;

    ros::Subscriber remote_state_sub_;
    realtime_tools::RealtimeBuffer<ros_ecat_msgs::RemoteState> remote_state_buffer_;
    ros::Subscriber odom_sub_;
    realtime_tools::RealtimeBuffer<ros_ecat_msgs::RemoteState> odom_buffer_;
    ros::Subscriber camera_sub_;
    realtime_tools::RealtimeBuffer<vision_msgs::BoundingBox2DArray> camera_buffer_;
    ros::Subscriber distance_cmd_sub_; //To debug
    realtime_tools::RealtimeBuffer<ros_ecat_msgs::RemoteState> distance_cmd_buffer_;
    ros::Subscriber init_state_sub_;
    realtime_tools::RealtimeBuffer<ros_ecat_msgs::InitState> init_state_buffer_;


    any_node::ThreadedPublisherPtr<std_msgs::UInt8> launch_state_pub_;
    any_node::ThreadedPublisherPtr<std_msgs::Float32> radar_cmd_pub;
    any_node::ThreadedPublisherPtr<std_msgs::Float32> camera_cmd_pub;

    ros::Subscriber dribble_shoot_pub_;
    realtime_tools::RealtimeBuffer<std_msgs::Bool> dribble_buffer_;
    double dribble_distance_;
    
    ctrl_common::PositionPID position_pid_;
    double zero_pos_ , max_distance_ , reset_time_ ,speed_offset_;
    double encoder_data_;
    double target_distance_ = 0.0;
    double radar_distance_ , camera_distance_;
    double death_zone_;
    double brake_time_;
    double out_put_;
    double last_position_ , last_velocity_ ,last_silence_time_;

    double check_time_;
    bool check_flag_ = {false}; 

    /*发射机构动作状态*/
    enum State {
        RESET, //发射机构回位
        MOVE, //发射机构下拉
        SHOOT, //发射
        WAIT_RESET //等待回位，防止回位过快干扰发射
    };
    uint8_t launch_state_ = {RESET};
    

};
}