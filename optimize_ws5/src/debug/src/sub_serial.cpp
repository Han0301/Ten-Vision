#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <cstring>  // 或者 #include <string.h>
#include <iostream>
#include <thread>
#include "rc_msgs/serial.h"

#define MAX_DATA_LENGTH 5
union data // 数据域联合体(允许以不同方式访问相同内存)
{
    float msg_get[MAX_DATA_LENGTH];
    uint8_t buff_msg[MAX_DATA_LENGTH * 4];
}data;

void mapCallback(const rc_msgs::serialConstPtr& msg)
{
    ROS_INFO("Data array size: %zu", msg->data.size());
    //float msgs[msg->data.size()] = {0};
    for (size_t i = 0; i < msg->data.size(); ++i) 
    {
        data.msg_get[i] = msg->data[i];
    }
    std::cout<<"-------------------------------"<<std::endl;
    // ros::NodeHandle nh;
    // ros::Publisher status_pub = nh.advertise<rc_msgs::serial>("/serial/status", 10);
    // rc_msgs::serial serial_pub;
    // serial_pub.stamp = ros::Time::now();
    // serial_pub.data.push_back(2);
    // status_pub.publish(serial_pub);
    std::cout<<"ok"<<std::endl;
    for(int i = 0; i<msg->data.size()*4; i++)
    {
        std::cout<<(int)data.buff_msg[i]<<std::endl;
        if(i%2 == 1)
        {
            std::cout<<"-------"<<std::endl;
        }
    }
    std::cout<<"-------------------------------"<<std::endl;
}
    // uint8_t frame_id;
    // float received_data[5];
    // uint8_t data_length;
    // while(ros::ok())
    // {
    //     if (global.serialComm->serial_read(&frame_id, received_data, &data_length)) {
    //         rc_msgs::serial serial_pub;
    //         serial_pub.stamp = ros::Time::now();
    //         serial_pub.data.push_back(received_data[0]);
    //         status_pub.publish(serial_pub);
    //         std::cout<<"ok"<<std::endl;
    //     }
    //     //std:cout<<"status_pub.publish(status_msg);"<<std::endl;
    //     //ros::spinOnce();
    //     rate.sleep();
    // }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sub_serial_node");
    ros::NodeHandle nh;
    ros::Subscriber map_sub = nh.subscribe("/serial/map", 10, mapCallback);
    ros::spin();
    return 0;
}