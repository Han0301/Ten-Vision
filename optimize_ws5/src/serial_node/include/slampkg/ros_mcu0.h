#ifndef __SERIAL_MCU0_H_
#define __SERIAL_MCU0_H_
#include <ros/ros.h>
#include <iostream>
#include <serial/serial.h>
#include <string>
#include <ros/package.h>

using namespace std;

namespace mcu0_serial
{
#define FRAME_HEAD_0 0xAA //帧头字节1
#define FRAME_HEAD_1 0x55  // 帧头字节2
#define FRAME_END_0 0xEE // 帧尾字节1
#define FRAME_END_1 0x0A // 帧尾字节2
//#define MAX_DATA_LENGTH 36 // 最大数据长度(浮点数个数)
#define MAX_DATA_LENGTH 10 // 最大数据长度(浮点数个数)

uint16_t CRC16_Table(uint8_t *p, uint8_t counter);

typedef struct serial_frame
{
    uint8_t data_length;  // 有效数据长度(浮点数个数)
    uint8_t frame_head[2]; // 帧头(2字节)
    uint8_t frame_id; // 帧ID
    uint16_t crc_calculated; // 计算出的CRC值
    union data // 数据域联合体(允许以不同方式访问相同内存)
    {
        float msg_get[MAX_DATA_LENGTH];
        uint8_t buff_msg[MAX_DATA_LENGTH * 4];
    } data;
    union check_code // CRC校验码联合体
    {
        uint16_t crc_code;
        //uint8_t crc_buff[2];
        uint8_t crc_buff[1];
    } check_code;
    uint8_t frame_end[2]; // 帧尾(2字节):0xFD 0xFE
} msg_frame;

class serial_mcu
{
public:
    serial_mcu(const std::string& port); // 构造函数 (参数:串口设备路径)
    ~serial_mcu();
    // 串口发送函数
    // 参数: 帧ID, 浮点数组, 数据长度(浮点数个数)
    // 返回: 实际发送的字节数
    size_t serial_send(uint8_t frame_id, float msgs[], uint8_t length);
    // 串口读取函数
    // 参数: 接收的帧ID(输出), 浮点数组(输出), 接收数据长度(输出)
    // 返回: 是否成功读取有效帧
    bool serial_read(uint8_t* received_frame_id, float msgs[], uint8_t* received_length);
    bool isOpen() const;
    ros::NodeHandle nh_;  // ROS节点句柄
    std::string serial_port_; // 串口设备路径

private:
    msg_frame framein_, frameout_;
    
    //serial
    serial::Serial serial_; 
    
    int serial_baud_ = 9600; // 波特率(默认115200)
    int serial_timeout_ = 100; // 超时时间(ms, 默认100ms)
    double dt = 0, speed_dt = 0; // 当前时间差 ，速度计算时间差
    double last_update_time = 0, last_speed_update_time = 0;// 上一次更新时间， // 上一次速度更新时间
    double update_time = 0.02;// 预设更新周期(20ms)
    ros::Time time_now;
    int test = 0;
    
    uint8_t send_length = 0;// 已发送数据长度
    double last_x = 0;// 上一坐标X值
    double last_y = 0; // 上一坐标Y值
};

}

#endif
