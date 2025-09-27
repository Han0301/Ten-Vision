/**
 * @file pbox_remote.h
 * @author py <997276894@qq.com>
 * @brief  
 * @version 0.1
 * @date  2025-06-08
 * 
 * @copyright Copyright (c)  2025 
 * 
 * @attention  
 * @note  
 */


#pragma once

#include <XmlRpc.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/UnloadController.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <realtime_tools/realtime_buffer.h>
#include <ros/ros.h>
#include <ros_ecat_msgs/Ops9Command.h>
#include <ros_ecat_msgs/UpperBoardCmd.h>
#include <ros_ecat_msgs/RemoteState.h>
#include <serial/serial.h>
#include <ros_ecat_msgs/PboxMsg.h>
#include <std_msgs/Float32MultiArray.h>

#include <any_node/ThreadedPublisher.hpp>
#include <any_worker/Worker.hpp>
#include <signal_handler/SignalHandler.hpp>

namespace pbox_hw {

#define JOY_CENTER 32768.0

struct PBOX_DATA {
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
    bool SWC;
    
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

class PboxRemote {
    public:
    PboxRemote() = default;
    ~PboxRemote() = default;

    bool init(ros::NodeHandle& nh);
    void read(const ros::Time& time);
    void write(const std::vector<uint8_t>& data);

    /* rosrun relative */
    bool updateWorkerCb(const any_worker::WorkerEvent& event);

    bool publishWorkerCb(const any_worker::WorkerEvent& event);

    bool sendWorkerCb(const any_worker::WorkerEvent& event);

    void handleSignal(int /* signum */);

    void requestControllerLoad();

    void requestControllerUnload();

    void requestControllerStop();

    void requestControllerStart();

    void PboxDataCallback(const ros_ecat_msgs::PboxMsg::ConstPtr& msg);
    void YoloDataCallback(const std_msgs::Float32MultiArrayConstPtr &msg);

    private:
    void unpack();

    union Pbox_Data {
    uint8_t data[4];
    float UIData;
    } PboxData_;

    union Pbox_Buffer {
    uint8_t data[4];
    float UIData;
    } PboxBuffer_;

    ros::NodeHandle nh_;
    PBOX_DATA xbox_data_{};
    serial::Serial serial_{};
    uint8_t buffer_[43]{};
    uint8_t fsm_flag_;
    double deadzone;
    double max_vel_;

    bool last_swa_{false};
    bool last_swb_{false};

    /* rosrun relative */
    std::shared_ptr<any_worker::Worker> update_worker_;
    std::shared_ptr<any_worker::Worker> publish_worker_;
    std::shared_ptr<any_worker::Worker> send_worker_;
    any_node::ThreadedPublisherPtr<geometry_msgs::Twist> cmd_vel_publisher_;
    any_node::ThreadedPublisherPtr<ros_ecat_msgs::UpperBoardCmd>
        upperboard_cmd_publisher_;
    any_node::ThreadedPublisherPtr<ros_ecat_msgs::Ops9Command>
        ops9_cmd_publisher_;
    
    any_node::ThreadedPublisherPtr<ros_ecat_msgs::RemoteState>
        remote_state_publisher_;
    
    any_node::ThreadedPublisherPtr<std_msgs::String>
        swerve_init_publisher_;

    /* controller manager service */
    ros::ServiceClient controller_load_client_;
    ros::ServiceClient controller_unload_client_;
    ros::ServiceClient controller_switch_client_;
    std::vector<std::string> controller_name_to_load_{};
    std::vector<std::string> controller_name_to_start_{};
    std::vector<std::string> controller_name_to_stop_{};

    ros::Subscriber pbox_data_sub_;
    realtime_tools::RealtimeBuffer<ros_ecat_msgs::PboxMsg> pbox_data_buffer_;
    std::mutex pbox_data_mutex_;

    ros::Subscriber yolo_data_sub_;
    realtime_tools::RealtimeBuffer<std_msgs::Float32MultiArray> yolo_data_buffer_;
    bool basket_lock_ = {false};

    ros_ecat_msgs::RemoteState remote_state_msg_;

    
};
}


