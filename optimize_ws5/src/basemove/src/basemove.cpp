//44 28
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <thread>
#include "PID.cpp"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <vision_msgs/BoundingBox2DArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <geometry_msgs/PoseWithCovariance.h> 
#include <geometry_msgs/TwistWithCovariance.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h> 
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <cstring>  // 或者 #include <string.h>
#include <iostream>
#include "rc_msgs/yolo_detector.h"
#include "rc_msgs/yolo_detectorArray.h"

#include <chrono> 

struct
{
    float _lidar_x;
    float _lidar_y;
    float _lidar_yaw;
    float _camera_target_y;
    float _camera_measure_y;
    std::mutex mtx_lidar;
    std::mutex mtx_camera;
}global;

// 回调函数：处理接收到的 TF 消息
void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
    for (const auto& transform : msg->transforms)
    {
        if(transform.child_frame_id != "body")
        {
            continue;
        }
        // 获取四元数
        geometry_msgs::Quaternion quat = transform.transform.rotation;

        // 使用 tf2 库将四元数转换为欧拉角
        tf2::Quaternion tf_quat;
        tf_quat.setX(quat.x);
        tf_quat.setY(quat.y);
        tf_quat.setZ(quat.z);
        tf_quat.setW(quat.w);

        // 转换为旋转矩阵，然后提取欧拉角
        tf2::Matrix3x3 mat(tf_quat);
        double roll, pitch, yaw;
        mat.getRPY(roll, pitch, yaw);  // 得到的是弧度值

        // ROS_INFO_STREAM("Transform from " << transform.header.frame_id 
        //               << " to " << transform.child_frame_id 
        //               << "\nRoll: " << roll 
        //               << " [rad], Pitch: " << pitch 
        //               << " [rad], Yaw: " << yaw 
        //               << " [rad]");
        
        double yaw_deg = yaw * 180.0 / M_PI;
        float _x  = transform.transform.translation.x - 0.28*cos(yaw) + 0.28;
        float _y = transform.transform.translation.y - 0.28*sin(yaw);

        std::cout << "base x = "<< _x << std::endl;
        std::cout << "base y = "<< _y << std::endl;
        std::cout << "base yaw = "<< yaw << std::endl;

        //mtx.lock();
        std::lock_guard<std::mutex> lock(global.mtx_lidar);
        global._lidar_x = _x;
        global._lidar_y = _y;
        global._lidar_yaw = yaw;
        //mtx.unlock();
    }
}


void yolodetector(const rc_msgs::yolo_detectorArray::ConstPtr msg)
{

    
    for (size_t i = 0; i < msg->boxes.size(); ++i) 
    {
        const auto& bbox = msg->boxes[i];
        if(bbox.cls == 1)
        {
            std::cout<<"bbox.cls"<<bbox.cls<<std::endl;
            std::lock_guard<std::mutex> lock(global.mtx_camera);
            global._camera_target_y = bbox.center.x;
        }
        else if(bbox.cls == 4)
        {
            std::cout<<"bbox.cls"<<bbox.cls<<std::endl;
            std::lock_guard<std::mutex> lock(global.mtx_camera);
            global._camera_measure_y = bbox.center.x;
        }
    }
    
} 


void worker_task1(ros::NodeHandle nh)
{
    ros::Rate sl(1000);
    ros::Subscriber tf_sub = nh.subscribe("/tf", 10, tfCallback);
    while (ros::ok())
    {
        /* code */
        
        ros::spinOnce();
        sl.sleep();
    }
    
}



void worker_task2(ros::NodeHandle nh)
{
    ros::Rate sl(300);
    ros::Subscriber tf_sub = nh.subscribe("/yolo/detections", 10, yolodetector);
    while (ros::ok())
    {
        /* code */
        
        ros::spinOnce();
        sl.sleep();
    }
    
}



int main(int argc, char** argv)
{
    // 初始化节点
    ros::init(argc, argv, "basemove_node");

    // 创建节点句柄
    ros::NodeHandle nh;

    std::vector<std::thread> workers;
    workers.emplace_back(worker_task1, nh);
    workers.emplace_back(worker_task2, nh);

    // 创建 Publisher，发布 /cmd_vel 话题，消息类型 geometry_msgs::Twist，队列长度 10
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 设置循环频率
    ros::Rate loop_rate(30); // 10Hz

    int flag = 0;
    PIDcontroler pid_x;
    PIDcontroler pid_y(0.1, 0.3, 0.02);
    PIDcontroler pid_yaw(0.3, 0.3, 0.001);
    auto start = std::chrono::steady_clock::now();
    while (ros::ok())
    {
        geometry_msgs::Twist vel_msg;
        auto end = std::chrono::steady_clock::now();
        auto duration = end - start;
        double seconds_double = static_cast<double>(duration.count()) / 1e9;
        std::cout<< "seconds_double" << seconds_double << std::endl;
        if(flag == 0)
        {
            std::lock_guard<std::mutex> lock(global.mtx_lidar);
            std::cout<<"A"<<std::endl;
            vel_msg.linear.x = pid_x.PIDcalculate(8.527, global._lidar_x, seconds_double);
            vel_msg.linear.y = pid_y.PIDcalculate(0, global._lidar_y, seconds_double);
            vel_msg.angular.z = pid_yaw.PIDcalculate(0, global._lidar_yaw, seconds_double);
            if(global._lidar_x >= 7.8)
            {
                pid_x.PIDreset(0.1);
                pid_y.PIDreset(0.15);
                flag = 1;
            }
        }
        else
        {
            {
                std::lock_guard<std::mutex> lock(global.mtx_lidar);
                vel_msg.linear.x = pid_x.PIDcalculate(8.527, global._lidar_x, seconds_double);
                vel_msg.angular.z = pid_yaw.PIDcalculate(0, global._lidar_yaw, seconds_double);
            }

            {
                std::lock_guard<std::mutex> lock(global.mtx_camera);
                vel_msg.linear.y = -pid_y.PIDcalculate(global._camera_target_y*0.01,global._camera_measure_y*0.01, seconds_double);
            }
        }        
        start = std::chrono::steady_clock::now();
        // // 线性速度 (m/s)
        // vel_msg.linear.x = 0.0;  // 前进
        // vel_msg.linear.y = 0.0;
        // vel_msg.linear.z = 0.0;

        // // 角速度 (rad/s)
        // vel_msg.angular.x = 0.0;
        // vel_msg.angular.y = 0.0;
        // vel_msg.angular.z = 0.0; 

        // 发布消息
        cmd_vel_pub.publish(vel_msg);

        ROS_INFO("Publishing /cmd_vel: linear.x=%f, linear.y=%f, angular.z=%f",
                 vel_msg.linear.x, vel_msg.linear.y, vel_msg.angular.z);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}






















