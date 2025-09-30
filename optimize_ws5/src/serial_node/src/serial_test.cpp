#include <ros/ros.h>
#include "../include/slampkg/ros_mcu0.h"
#include "../include/slampkg/serial_test.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <vision_msgs/BoundingBox2DArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <nav_msgs/Odometry.h>  // 核心头文件
#include <geometry_msgs/PoseWithCovariance.h> 
#include <geometry_msgs/TwistWithCovariance.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h> 
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Geometry>
#include <cstring>  // 或者 #include <string.h>
#include <iostream>
#include <thread>
#include "rc_msgs/serial.h"
//M_PI;

//---------------------------
#include <stdexcept>
#include <bitset>

/**
 * 计算数组的奇偶校验位
 * @param data 要校验的数组
 * @param length 数组长度
 * @param parityType 校验类型: "odd"(奇校验), "even"(偶校验), "mark"(标记校验), "space"(空格校验)
 * @return 校验位值(0或1)
 */

//异或校验
int calculateArrayParity(const uint8_t* data, size_t length) {

    uint8_t checksum = 0;
    if (data == NULL || length == 0) {
        return checksum; // 空指针或空数组返回0
    }
    for (size_t i = 0; i < length; i++) {
        checksum ^= data[i]; // 逐字节异或累加
    }
    return checksum;
}

//--------------------------

namespace mcu0_serial
{
    serial_mcu::serial_mcu(const std::string& port)
    {
        serial_port_ = port;
        ros::NodeHandle private_nh("~");
        private_nh.param("baud", serial_baud_, 9600);
        // 持续尝试打开串口直到成功
        while (!serial_.isOpen())
        {
            ROS_WARN("try to open port %s", serial_port_.c_str());
            serial_.setPort(serial_port_);
            serial_.setBaudrate(serial_baud_);
            serial_.setFlowcontrol(serial::flowcontrol_none);
            serial_.setParity(serial::parity_none); // default is parity_none
            serial_.setStopbits(serial::stopbits_one);
            serial_.setBytesize(serial::eightbits);
            serial::Timeout time_out = serial::Timeout::simpleTimeout(serial_timeout_);
            serial_.setTimeout(time_out);
            serial_.open();
        }
        ROS_INFO("port open success");
    }
    // 析构函数：关闭串口
    serial_mcu::~serial_mcu()
    {
        if (serial_.isOpen())
            serial_.close();
    }
    // 检查串口是否打开
    bool serial_mcu::isOpen() const{
        return serial_.isOpen();
    }
    // 串口数据发送函数
    size_t serial_mcu::serial_send(uint8_t frame_id, float msgs[], uint8_t length)
    {
        // 构建数据帧：帧头+ID+数据长度+数据+CRC+帧尾
        uint8_t buff_msg[length * 4 + 6] = {0};
        //uint8_t buff_msg[length + 8] = {0};
        frameout_.frame_head[0] = FRAME_HEAD_0;
        frameout_.frame_head[1] = FRAME_HEAD_1;
        frameout_.frame_id = frame_id;
        frameout_.data_length = length * 4;
        // 填充浮点数据（转换为字节）//
        for (int i = 0; i < length; i++)
        {
            frameout_.data.msg_get[i] = msgs[i];
        }

        buff_msg[0] = FRAME_HEAD_0;
        buff_msg[1] = FRAME_HEAD_1;
        buff_msg[2] = frame_id;
        //buff_msg[3] = length * 4;   
        buff_msg[3] = length * 4;


        for (int q = 0; q < length * 4; q++)
        {
            buff_msg[4 + q] = frameout_.data.buff_msg[q];
        }

        // frameout_.check_code.crc_code = CRC16_Table(frameout_.data.buff_msg, length * 4);
        //buff_msg[4 + length * 4] = frameout_.check_code.crc_buff[0];
        // buff_msg[5 + length * 4] = frameout_.check_code.crc_buff[1];
        // buff_msg[6 + length * 4] = FRAME_END_0;
        // buff_msg[7 + length * 4] = FRAME_Erc3_wsND_1;
       
        //------------------------------------
        //奇校验
        buff_msg[4 + length * 4] = calculateArrayParity(&buff_msg[4], length * 4);
        //------------------------------------

        buff_msg[5 + length * 4] = FRAME_END_0;
        //buff_msg[5 + length * 4] = FRAME_END_1;

        //size_t write_num = serial_.write(buff_msg, length * 4 + 8);
        size_t write_num = serial_.write(buff_msg, length * 4 + 6);
        ROS_INFO("write %d", write_num);
        return write_num;
    }
    // 串口数据接收函数
    bool serial_mcu::serial_read(uint8_t* received_frame_id, float msgs[], uint8_t* received_length) {
        if (!serial_.isOpen()) {
            ROS_ERROR("Serial port is not open.");
            return false;
        }

        uint8_t byte;//声明临时变量存储当前读取的字节
        //循环处理串口缓冲区中的所有可用数据
        while (serial_.available() > 0) {
            /*
            从串口读取1字节数据
            读取失败则跳过后续处理继续循环
            */
            ros::Rate rate(1000);

            if (serial_.read(&byte, 1) != 1) 
            {
                
                continue;
            }

            if (byte == FRAME_HEAD_0) {
                //std::cout<<0<<std::endl;
                // Check FRAME_HEAD_1
                if (serial_.read(&byte, 1) != 1 || byte != FRAME_HEAD_1) continue;
                //std::cout<<1<<std::endl;
                // Read frame ID
                if (serial_.read(&framein_.frame_id, 1) != 1) continue;
                *received_frame_id = framein_.frame_id;
                // std::cout<<2<<std::endl;
                // Read data length
                uint8_t data_length;
                if (serial_.read(&data_length, 1) != 1) 
                {
                    //std::cout<<(int)data_length<<std::endl;
                    continue;
                }
                
                size_t expected_data_length = data_length;
                //std::cout<<expected_data_length<<std::endl;
                // Validate data length检查数据长度是否超出缓冲区容量
                if (expected_data_length > sizeof(framein_.data.buff_msg)) {
                    ROS_WARN("Data too long: %d", expected_data_length);
                    continue;
                }

                // Read data payload读取数据域内容到缓冲区
                //serial_.read(framein_.data.buff_msg, expected_data_length);
                if (serial_.read(framein_.data.buff_msg, expected_data_length) != expected_data_length) continue;

                // Skip CRC and check frame end
                uint8_t end_bytes[2];
                //if (serial_.read(framein_.check_code.crc_buff, 2) != 2) continue;
                //if (serial_.read(framein_.check_code.crc_buff, 1) != 1) continue;
                //-----------------------------------------
                if (serial_.read(&end_bytes[0], 1) != 1 || calculateArrayParity(framein_.data.buff_msg, expected_data_length) != end_bytes[0]) continue;


                //--------------------------------------
                //if (serial_.read(end_bytes, 2) != 2 || end_bytes[0] != FRAME_END_0 || end_bytes[1] != FRAME_END_1) continue;
                if (serial_.read(&end_bytes[1], 1) != 1 || end_bytes[0] != FRAME_END_0) continue;


                // Extract float data
                *received_length = expected_data_length / 4;
                for (size_t i = 0; i < *received_length; ++i) {
                    msgs[i] = framein_.data.msg_get[i];
                }
                //std::cout<<3<<std::endl;
                rate.sleep(); // 休眠至满足100Hz频率
                return true;
            }
        }
        
        return false;
    }
    uint16_t CRC16_Table(uint8_t *p, uint8_t counter)
    {
        static const uint16_t CRC16Table[256] =
        {
            0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
            0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
            0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
            0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
            0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
            0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
            0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
            0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
            0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
            0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
            0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
            0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
            0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
            0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
            0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
            0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
            0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
            0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
            0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
            0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
            0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
            0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
            0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
            0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
            0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
            0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
            0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
            0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
            0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
            0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
            0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
            0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
        };

        uint16_t crc16 = 0;
        for (int i = 0; i < counter; i++)
        {
            uint8_t value = p[i];
            crc16 = CRC16Table[((crc16 >> 8) ^ value) & 0xff] ^ (crc16 << 8);
        }
        return crc16;
    }
}




using namespace mcu0_serial;



struct
{
    serial_mcu* serialComm;
    std::mutex mtx;
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

        ROS_INFO_STREAM("Transform from " << transform.header.frame_id 
                      << " to " << transform.child_frame_id 
                      << "\nRoll: " << roll 
                      << " [rad], Pitch: " << pitch 
                      << " [rad], Yaw: " << yaw 
                      << " [rad]");
        double yaw_deg = yaw * 180.0 / M_PI;
        float _x  = transform.transform.translation.x - 0.28*cos(yaw) + 0.28;
        float _y = transform.transform.translation.y - 0.28*sin(yaw);
        float msgs[4] = {0};
        msgs[0] = _x;
        msgs[1] = _y;
        msgs[2] = transform.transform.translation.z;
        msgs[3] = yaw_deg;
        //mtx.lock();
        std::lock_guard<std::mutex> lock(global.mtx);
        global.serialComm->serial_send(1, msgs, 4);
        //mtx.unlock();
    }
}

// 回调函数，用于处理接收到的Odometry消息
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 提取位置信息
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double z = msg->pose.pose.position.z;
    
    // 提取姿态信息（四元数）
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    
    // 将四元数转换为欧拉角（roll, pitch, yaw）
    // 单位为弧度
    double roll, pitch, yaw;
    tf2::Quaternion quat;
    // 从消息中获取四元数
    quat.setX(qx);
    quat.setY(qy);
    quat.setZ(qz);
    quat.setW(qw);
    // 转换为欧拉角（滚转、俯仰、偏航）
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    // 可选：转换为角度（弧度 * (180/PI)）
    double roll_deg = roll * 180.0 / M_PI;
    double pitch_deg = pitch * 180.0 / M_PI;
    double yaw_deg = yaw * 180.0 / M_PI;

    // 提取线速度信息
    double linear_x = msg->twist.twist.linear.x;
    double linear_y = msg->twist.twist.linear.y;
    double linear_z = msg->twist.twist.linear.z;
    
    // 提取角速度信息
    double angular_x = msg->twist.twist.angular.x;
    double angular_y = msg->twist.twist.angular.y;
    double angular_z = msg->twist.twist.angular.z;
    
    // 打印接收到的信息
    ROS_INFO("Received Odometry Data:");
    ROS_INFO("Position: x=%.2f, y=%.2f, z=%.2f", x, y, z);
    // ROS_INFO("Orientation: x=%.2f, y=%.2f, z=%.2f, w=%.2f", qx, qy, qz, qw);
    // ROS_INFO("Linear Velocity: x=%.2f, y=%.2f, z=%.2f", linear_x, linear_y, linear_z);
    // ROS_INFO("Angular Velocity: x=%.2f, y=%.2f, z=%.2f", angular_x, angular_y, angular_z);
    // ROS_INFO("----------------------------------------");

    float _x  = x - 0.28*cos(yaw) + 0.28;
    float _y = y - 0.28*sin(yaw);

    float msgs[4] = {0};
    msgs[0] = _x;
    msgs[1] = _y;
    msgs[2] = z;
    msgs[3] = yaw_deg;
    std::lock_guard<std::mutex> lock(global.mtx);
    global.serialComm->serial_send(1, msgs, 4);
}

void mapCallback(const rc_msgs::serialConstPtr& msg)
{
    ROS_INFO("Data array size: %zu", msg->data.size());
    float msgs[msg->data.size()] = {0};
    for (size_t i = 0; i < msg->data.size(); ++i) 
    {
        msgs[i] = msg->data[i];
    }
    std::lock_guard<std::mutex> lock(global.mtx);
    global.serialComm->serial_send(2, msgs, msg->data.size());
}

// 回调函数：处理接收到的 Imu 消息
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // 打印消息中的关键数据（根据需求调整）
    ROS_INFO_STREAM("接收到 Imu 数据:");
    
    // 方向四元数（x, y, z, w）
    ROS_INFO_STREAM("Orientation: x=" << msg->orientation.x 
              << ", y=" << msg->orientation.y 
              << ", z=" << msg->orientation.z 
              << ", w=" << msg->orientation.w);
    
    // 角速度（rad/s，x, y, z 轴）
    ROS_INFO_STREAM("Angular Velocity: x=" << msg->angular_velocity.x 
              << ", y=" << msg->angular_velocity.y 
              << ", z=" << msg->angular_velocity.z);
    
    // 线加速度（m/s²，x, y, z 轴）
    ROS_INFO_STREAM("Linear Acceleration: x=" << msg->linear_acceleration.x 
              << ", y=" << msg->linear_acceleration.y 
              << ", z=" << msg->linear_acceleration.z);

    float msgs[5] = {0};
    msgs[0] = msg->angular_velocity.x;
    msgs[1] = msg->angular_velocity.y;
    msgs[2] = msg->angular_velocity.z;
    msgs[3] = 0;
    msgs[4] = 0;
    ros::Rate _sleep(11);
    _sleep.sleep();
    //mtx.lock();
    std::lock_guard<std::mutex> lock(global.mtx);
    global.serialComm->serial_send(3, msgs, 3);
    //mtx.unlock();
    
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
    //ros::Subscriber tf2_sub = global.nh.subscribe("/target", 2, tf2Callback);
    
    ros::Rate sl(1000);
    ros::Subscriber map_sub = nh.subscribe("/serial/map", 10, mapCallback);
    while (ros::ok())
    {
        /* code */
        
        ros::spinOnce();
        sl.sleep();
    }
    
}
void worker_task3(ros::NodeHandle nh)
{
    
    ros::Rate sl(1000);
    //ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 2, imuCallback);
    while (ros::ok())
    {
        /* code */
        
        ros::spinOnce();
        sl.sleep();
    }  
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;
    
    // nh.param<int>("image_width", image_width, 640);
    // nh.param<int>("image_height", image_height, 480);
    std::string port;
    //nh.param<std::string>("serial_port", port, "/dev/ttyUSB0");
    nh.param<std::string>("serial_port", port, "/dev/ttyACM0");
    try {
        global.serialComm = new serial_mcu(port);
        ROS_INFO("Serial port initialized successfully");
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to initialize serial port: %s", e.what());
        return -1;
    }

    std::vector<std::thread> workers;
    workers.emplace_back(worker_task1, nh);
    workers.emplace_back(worker_task2, nh);
    workers.emplace_back(worker_task3, nh);
    ros::Publisher status_pub = nh.advertise<rc_msgs::serial>("/serial/status", 10);
    
    //ros::AsyncSpinner spinner(4);
    //spinner.start();

    ros::Rate rate(1000);
    //std_msgs::Bool status_msg;
    
    //status_msg.data = false;

    uint8_t frame_id;
    float received_data[5];
    uint8_t data_length;
    while(ros::ok())
    {
        if (global.serialComm->serial_read(&frame_id, received_data, &data_length)) {
            rc_msgs::serial serial_pub;
            serial_pub.stamp = ros::Time::now();
            serial_pub.data.push_back(received_data[0]);
            status_pub.publish(serial_pub);
            std::cout<<"ok"<<std::endl;
        }
        //std:cout<<"status_pub.publish(status_msg);"<<std::endl;
        //ros::spinOnce();
        rate.sleep();
    }


    delete global.serialComm;
    //ros::waitForShutdown();
    return 0;
}