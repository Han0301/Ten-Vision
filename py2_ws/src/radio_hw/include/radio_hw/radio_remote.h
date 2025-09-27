/**
 * @file ： xbox_remote.h
 * @author: py <997276894@qq.com>
 * @brief ： xbox接收头文件
 * @version 0.1
 * @date: 2025-05-06 20:16:06
 *
 * @copyright Copyright (c)  2025
 *
 * @attention :
 * @note :
 */
#pragma once

#include <XmlRpc.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <ros/ros.h>
#include <ros_ecat_msgs/Ops9Command.h>
#include <ros_ecat_msgs/UpperBoardCmd.h>
#include <ros_ecat_msgs/RemoteState.h>
#include <ros_ecat_msgs/InitState.h>
#include <ros_ecat_msgs/PositionSensorCmd.h>
#include <serial/serial.h>
#include <unistd.h>
#include <ros_ecat_msgs/PboxMsg.h>
#include <realtime_tools/realtime_buffer.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float32.h>

#include <any_node/ThreadedPublisher.hpp>
#include <any_worker/Worker.hpp>
#include <signal_handler/SignalHandler.hpp>
#include "boost/asio.hpp"
#include "stdint.h"
#include "uart.hpp"
#include "radio_hw/crc.h"
#include "vision_msgs/BoundingBox2DArray.h"

namespace radio_hw {

#define BTN_OFF 191
#define BTN_ON 1792

#define BTN_1 191
#define BTN_2 1004
#define BTN_3 1792


//填写最原始的通道数据
// MAX稍微小点 MIN稍微大点，保证区域与真实值有重合
#define JOY_LVERT_MAX 1726
#define JOY_LVERT_MIN 174

#define JOY_LHORI_MAX 1811
#define JOY_LHORI_MIN 174

#define JOY_RVERT_MAX 1811
#define JOY_RVERT_MIN 174

#define JOY_RHORI_MAX 1811
#define JOY_RHORI_MIN 183

#define JOY_Death_Zone 40

struct XBOX_DATA {
  bool btnY;
  bool btnX;
  bool btnA;
  bool btnB;
  bool btnLB;
  bool btnRB;
  bool btnLS;
  bool btnRS;
  bool btnDirUp;
  bool btnDirLeft;
  bool btnDirDown;
  bool btnDirRight;
  bool btnShare;
  bool btnStart;
  bool btnBack;
  bool btnXbox;
  bool SWA;
  bool SWB;
  
  uint16_t joyLHori;
  uint16_t joyLVert;
  uint16_t joyRHori;
  uint16_t joyRVert;
  uint16_t trigLT;
  uint16_t trigRT;
  float UIPosX;
  float UIPosY;
  float ShooterFactor;
};

class RadioRemote {
 public:
  RadioRemote();
  ~RadioRemote() = default;

  bool init(ros::NodeHandle& nh);
  void read_data(const ros::Time& time);
  // void write(const std::vector<uint8_t>& data);

  void send_state(uint8_t mode,double posx,double posy,float max_speed,float radar_yaw,float radar_shoot_cmd,float camera_shoot_cmd,float camera_catch);
  void get_state(const ros::Time& time);
  /* rosrun relative */
  bool updateWorkerCb(const any_worker::WorkerEvent& event);

  bool publishWorkerCb(const any_worker::WorkerEvent& event);

  void handleSignal(int /* signum */);

  void requestControllerLoad();

  void requestControllerUnload();

  void requestControllerStop();

  void requestControllerStart();

  void map_value_compute();
  void PboxDataCallback(const ros_ecat_msgs::PboxMsg::ConstPtr& msg);
  void InitStateCallback(const ros_ecat_msgs::InitState::ConstPtr& msg);
  void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void CameraCallback(const vision_msgs::BoundingBox2DArrayConstPtr &msg);
  void imuCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg);
  void radarCmdCallback(const std_msgs::Float32::ConstPtr &msg);
  void cameraCmdCallback(const std_msgs::Float32::ConstPtr &msg);

private:
void sendGps(double latitude, double longitude, uint16_t groundspeed,uint16_t heading, uint16_t altitude, uint8_t satellites);
void sendAttitude(float pitch, float roll, float yaw);
void unpack();
#pragma pack(push, 1)
typedef struct
{
    unsigned ch0 : 11;
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
}crsf_channels_t;
#pragma pack(pop)

  typedef struct
  {
      uint8_t device_addr;
      uint8_t frame_size; // size after this byte, so it's type + payload + crc
      uint8_t type;
      crsf_channels_t channels; // 16 cnav_msgshannels, 22 bytes
      uint8_t crc;
  }CrsfRcChannelsFrame_t;


  enum CrsfRxState
  {
    CRSF_WAITING_FOR_ADDRESS,
    CRSF_WAITING_FOR_FRAMESIZE,
    CRSF_WAITING_FOR_TYPE,
    CRSF_WAITING_FOR_PAYLOAD_CHANNELS,
    CRSF_WAITING_FOR_CRC_BYTE,
    CRSF_PACKET_COMPLETE
  }rx_state_;

typedef enum
{
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
} crsf_addr_e;

enum
{
    CRSF_FRAME_GPS_PAYLOAD_SIZE = 15,
    CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE = 8,
    CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE = 10,
    CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE = 22, // 11 bits per channel * 16 channels = 22 bytes.
    CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE = 6,
};
enum
{
    CRSF_FRAME_LENGTH_ADDRESS = 1,
    CRSF_FRAME_LENGTH_FRAMELENGTH = 1,
    CRSF_FRAME_LENGTH_TYPE = 1,
    CRSF_FRAME_LENGTH_CRC = 1,
    CRSF_FRAME_LENGTH_TYPE_CRC = 2,
    CRSF_FRAME_LENGTH_EXT_TYPE_CRC = 4,
    CRSF_FRAME_LENGTH_NON_PAYLOAD = 4,
};

typedef enum
{
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
    CRSF_FRAMETYPE_RADIO_ID = 0x3A,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,
    CRSF_FRAMETYPE_COMMAND = 0x32,
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C,
} crsf_frame_type_e;

struct RADIO_T
{
  double left_H_map;
  double left_V_map;
  double right_H_map;
  double right_V_map;
  double roll_map;

  bool btnSA;
  bool btnSD;
  bool btnSE;
  uint8_t btnSB;
  uint8_t btnSC;

};

  ros::NodeHandle nh_;
  ros::Subscriber pbox_data_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber camera_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber camera_cmd_sub_;
  ros::Subscriber radar_cmd_sub_;
  ros::Subscriber init_state_sub_;

  RADIO_T radio_data_{};
  Uart uart;

  uint8_t buffer_[42]{};
  uint8_t fsm_flag_;
  double deadzone;
  double max_vel_;
  int CRSF_NUM_CHANNELS_;
  int CRSF_CHANNEL_VALUE_MIN_;
  int CRSF_CHANNEL_VALUE_MID_;
  int CRSF_CHANNEL_VALUE_MAX_;
  int CRSF_MAX_PACKET_SIZE_;
  int CRSF_MAX_PAYLOAD_LEN_;
  int CRSF_CRC_POLY_;
  CrsfRcChannelsFrame_t current_rc_frame_;
    uint8_t *payload_ptr_;
    uint8_t packet_byte_index_;
  int channels_[16];
  int channels_init_[16];
  bool channels_init_flag_={false};
  int fd;
  // CRC 对象
  GENERIC_CRC8 crc_; // 使用多项式 0xD5 初始化
  uint8_t tx_buffer_[64];
  float c = -0.76f;

  ros_ecat_msgs::RemoteState remote_state_msg;
  /* rosrun relative */
  std::shared_ptr<any_worker::Worker> update_worker_;
  std::shared_ptr<any_worker::Worker> publish_worker_;
  any_node::ThreadedPublisherPtr<geometry_msgs::Twist> cmd_vel_publisher_;
  any_node::ThreadedPublisherPtr<ros_ecat_msgs::UpperBoardCmd>
      upperboard_cmd_publisher_;
  any_node::ThreadedPublisherPtr<ros_ecat_msgs::Ops9Command>
      ops9_cmd_publisher_;
  
  any_node::ThreadedPublisherPtr<ros_ecat_msgs::RemoteState>
      remote_state_publisher_;
  
  any_node::ThreadedPublisherPtr<std_msgs::String>
      swerve_init_publisher_;

  any_node::ThreadedPublisherPtr<ros_ecat_msgs::PositionSensorCmd>
      position_reset_pub_;

      

  /* controller manager service */
  ros::ServiceClient controller_load_client_;
  ros::ServiceClient controller_unload_client_;
  ros::ServiceClient controller_switch_client_;
  std::vector<std::string> controller_name_to_load_{};
  std::vector<std::string> controller_name_to_start_{};
  std::vector<std::string> controller_name_to_stop_{};
  geometry_msgs::Twist cmd_vel;

  realtime_tools::RealtimeBuffer<ros_ecat_msgs::PboxMsg> pbox_data_buffer_;
  realtime_tools::RealtimeBuffer<ros_ecat_msgs::InitState> init_state_buffer_;
  realtime_tools::RealtimeBuffer<nav_msgs::Odometry> odom_buffer_;
  realtime_tools::RealtimeBuffer<vision_msgs::BoundingBox2DArray> camera_buffer_;
  realtime_tools::RealtimeBuffer<geometry_msgs::Vector3Stamped> imu_buffer_;
  realtime_tools::RealtimeBuffer<std_msgs::Float32> radar_cmd_buffer_;
  realtime_tools::RealtimeBuffer<std_msgs::Float32> camera_cmd_buffer_;

  bool camera_init_ = {false};
  bool init_camera_ = {false};
  bool init_basket_ = {false};




  
};
}  // namespace xbox_hw