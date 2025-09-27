/**
 * @file robot_communicate.cpp
 * @author py <997276894@qq.com>
 * @brief  两车通讯源文件
 * @version 0.1
 * @date  2025-05-27 
 * 
 * @copyright Copyright (c)  2025 
 * 
 * @attention  
 * @note  
 */

 #include "robot_communication/robot_communicate.h"

namespace robot_communicate
{
bool RobotCommunicate::init(ros::NodeHandle& nh)
{

    nh_ = nh;
    if(!serial_.isOpen())
    {
        
        serial_.setPort("/dev/c6");
        serial_.setBaudrate(115200);
        serial::Timeout time_out = serial::Timeout::simpleTimeout(100);
        serial_.setTimeout(time_out);
        serial_.open();
        serial_.flushInput();

        // 初始化状态机变量
        fsm_flag_ = 0;
        index_ = 0;
        memset(buffer_rx, 0, 24);
        
        // 创建数据发布者
        partner_position_publisher_ = 
            std::make_shared<any_node::ThreadedPublisher<ros_ecat_msgs::PositionSensorMsg>>(
                nh.advertise<ros_ecat_msgs::PositionSensorMsg>("/partner_position", 10),
                50, true);
        
        // 创建工作线程
        read_worker_ = std::make_shared<any_worker::Worker>(
            "readWorker", 0.01, // 10ms周期
            std::bind(&RobotCommunicate::readWorkerCb, this, std::placeholders::_1));
        read_worker_->start(45); // 启动线程

        // position_sub_ = nh_.subscribe("position_sensor/position_sensor_data", 1, &RobotCommunicate::PostionCallback, this);
        radar_sub_ = nh_.subscribe("/odom",1,&RobotCommunicate::RadarCallback,this);

        return true;
    }
    else return false;
}

void RobotCommunicate::write()
{
    PosData.PosData = static_cast<float>(position_msg_.pose_x);
    memcpy(buffer_, PosData.data, 4);
    PosData.PosData =  static_cast<float>(position_msg_.pose_y);
    memcpy(buffer_ + 4, PosData.data, 4);
    PosData.PosData =  static_cast<float>(position_msg_.linear_x);
    memcpy(buffer_ + 8, PosData.data, 4);
    PosData.PosData =  static_cast<float>(position_msg_.linear_y);
    memcpy(buffer_ + 12, PosData.data, 4);

    std::vector<uint8_t> send_buffer(24, 0);
    send_buffer[0] = 0xFC; // header1
    send_buffer[1] = 0xFB; // header2
    send_buffer[2] = 0x01; // ID
    send_buffer[3] = 16; // Data length
    memcpy(send_buffer.data() + 4, buffer_, 16); // Data
    send_buffer[20] = 0x00; // CRC placeholder
    send_buffer[21] = 0x00; // CRC placeholder
    send_buffer[22] = 0xFD; // footer1
    send_buffer[23] = 0xFE; // footer2
    try
    {
        serial_.flushOutput();
        serial_.write(send_buffer.data(), send_buffer.size());
        ROS_INFO("SEND OK");
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Failed to write to serial port: %s", e.what());
    }
}

void RobotCommunicate::write_2()
{
    PosData.PosData = static_cast<float>(radar_msg_.pose.pose.position.y);
    memcpy(buffer_, PosData.data, 4);
    PosData.PosData =  static_cast<float>(radar_msg_.pose.pose.position.x);
    memcpy(buffer_ + 4, PosData.data, 4);
    PosData.PosData =  0.0f;
    memcpy(buffer_ + 8, PosData.data, 4);
    PosData.PosData =  0.0f;
    memcpy(buffer_ + 12, PosData.data, 4);

    std::vector<uint8_t> send_buffer(24, 0);
    send_buffer[0] = 0xFC; // header1
    send_buffer[1] = 0xFB; // header2
    send_buffer[2] = 0x01; // ID
    send_buffer[3] = 16; // Data length
    memcpy(send_buffer.data() + 4, buffer_, 16); // Data
    send_buffer[20] = 0x00; // CRC placeholder
    send_buffer[21] = 0x00; // CRC placeholder
    send_buffer[22] = 0xFD; // footer1
    send_buffer[23] = 0xFE; // footer2
    try
    {
        serial_.flushOutput();
        serial_.write(send_buffer.data(), send_buffer.size());
        ROS_INFO("SEND OK");
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Failed to write to serial port: %s", e.what());
    }
}

void RobotCommunicate::PostionCallback(const ros_ecat_msgs::PositionSensorMsg& msg)
{
    {
        std::lock_guard<std::mutex> lock(position_mutex_);
        position_msg_ = msg;
        write();
    }
}

void RobotCommunicate::RadarCallback(const nav_msgs::Odometry& msg)
{
    {
        std::lock_guard<std::mutex> lock(radar_mutex_);
        radar_msg_ = msg;
        write_2();
    }
}

// 读取工作线程回调
bool RobotCommunicate::readWorkerCb(const any_worker::WorkerEvent& event)
{
    read(); // 读取串口数据
    return true;
}

// 串口数据读取函数
void RobotCommunicate::read()
{
    size_t n = serial_.available();
    uint8_t serial_buffer;
    
    if (!n) return; // 无数据直接返回

    while (n--) {
        serial_.read(&serial_buffer, 1); // 每次读取一个字节
        
        // 状态机处理
        switch (fsm_flag_) {
            case 0: // 等待帧头1
                if (serial_buffer == 0xFC) fsm_flag_++;
                break;
                
            case 1: // 等待帧头2
                if (serial_buffer == 0xFB) fsm_flag_++;
                else fsm_flag_ = 0;
                break;
                
            case 2: // 等待设备ID
                if (serial_buffer == 0x01) fsm_flag_++;
                else fsm_flag_ = 0;
                break;
                
            case 3: // 等待数据长度
                if (serial_buffer == 0x18) // 固定24字节数据
                {
                    fsm_flag_++;
                    index_ = 0;
                }
                else fsm_flag_ = 0;
                break;
                
            case 4: // 接收数据
                buffer_rx[index_] = serial_buffer;
                index_++;
                if (index_ >= 24) // 数据接收完成
                {
                    fsm_flag_++;
                    index_ = 0;
                }
                break;
                
            case 5: // 跳过CRC字节2
                index_++;
                if (index_ >= 2) fsm_flag_++; // 只需跳过2字节
                break;
                
            case 6: // 等待帧尾1
                if (serial_buffer == 0xFD) fsm_flag_++;
                else
                {
                    fsm_flag_ = 0;
                    memset(buffer_rx, 0, 24);
                }
                break;
                
            case 7: // 等待帧尾2
                if (serial_buffer == 0xFE) 
                {
                    unpack(); // 解析有效数据
                }
                fsm_flag_ = 0; // 状态机复位
                serial_.flushInput(); // 清空缓冲区
                break;
                
            default:
                fsm_flag_ = 0;
                break;
        }
    }
}

// 解析接收到的数据
void RobotCommunicate::unpack()
{
    ros_ecat_msgs::PositionSensorMsg partner_position_msg_;
    
    // 解析6个float数据 (yaw_angle pose_x pose_y omega linear_x linear_y)
    memcpy(&partner_position_msg_.yaw_angle, buffer_rx, 4);
    memcpy(&partner_position_msg_.pose_x, buffer_rx + 4, 4);
    memcpy(&partner_position_msg_.pose_y, buffer_rx + 8, 4);
    memcpy(&partner_position_msg_.omega, buffer_rx + 12, 4);
    memcpy(&partner_position_msg_.linear_x, buffer_rx + 16, 4);
    memcpy(&partner_position_msg_.linear_y, buffer_rx + 20, 4);
    
    partner_position_msg_.header.stamp = ros::Time::now();
    partner_position_msg_.header.frame_id = "partner_pose";
    
    // 发布伙伴位置信息
    ROS_INFO_STREAM_THROTTLE(1.0, "Received partner position: "
        << partner_position_msg_.yaw_angle << partner_position_msg_.pose_x << ", " << partner_position_msg_.pose_y
        << " | Velocity: " << partner_position_msg_.omega << partner_position_msg_.linear_x << ", " << partner_position_msg_.linear_y);
    
    partner_position_publisher_->publish(partner_position_msg_);
}

}