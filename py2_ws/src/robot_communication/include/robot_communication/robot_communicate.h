/**
 * @file robot_communicate.h
 * @author py <997276894@qq.com>
 * @brief   两车通讯头文件
 * @version 0.1
 * @date  2025-05-27
 * 
 * @copyright Copyright (c)  2025 
 * 
 * @attention  
 * @note  
 */
#pragma once

#include <ros/ros.h>
#include <serial/serial.h>
#include "ros_ecat_msgs/PositionSensorMsg.h"
#include <nav_msgs/Odometry.h>
#include <any_node/ThreadedPublisher.hpp>
#include <any_worker/Worker.hpp>
namespace robot_communicate
{
class RobotCommunicate
{
public:
    RobotCommunicate() = default;

    bool init(ros::NodeHandle& nh);
    void read(const ros::Time &time);
    void write();
    void write_2();

    void PostionCallback(const ros_ecat_msgs::PositionSensorMsg& msg);
    void RadarCallback(const nav_msgs::Odometry& msg);

    bool readWorkerCb(const any_worker::WorkerEvent& event);
    void read();
    void unpack();

    union POS_DATA {
        uint8_t data[4];
        float PosData;
    } PosData;

    union POS_DATA_Y {
        uint8_t data[4];
        float PosData;
    } PosDataY;

private:
    ros::NodeHandle nh_;
    std::mutex position_mutex_;
    std::mutex radar_mutex_;
    ros::Subscriber position_sub_;
    ros::Subscriber radar_sub_;
    std::shared_ptr<any_worker::Worker> read_worker_;
    any_node::ThreadedPublisherPtr<ros_ecat_msgs::PositionSensorMsg>
      partner_position_publisher_;
    ros_ecat_msgs::PositionSensorMsg position_msg_;
    nav_msgs::Odometry radar_msg_;
    serial::Serial serial_;
    uint8_t buffer_[16];
    uint8_t buffer_rx[24];
    uint8_t fsm_flag_;
    uint8_t index_;
};
}




