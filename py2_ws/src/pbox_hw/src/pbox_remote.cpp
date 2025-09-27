/**
 * @file 
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

#include <pbox_hw/pbox_remote.h>

namespace pbox_hw {

bool PboxRemote::init(ros::NodeHandle& nh) {
    XmlRpc::XmlRpcValue config;
    if (!nh.getParam("pbox_settings", config)) {
      ROS_ERROR("Failed to get param: xbox_settings");
      return false;
    }
    if (!config.hasMember("usb_port")) {
      ROS_ERROR("usb_port not found in xbox_settings");
      return false;
    }
    std::string usb_port = static_cast<std::string>(config["usb_port"]);
    if (!config.hasMember("baudrate")) {
      ROS_ERROR("baudrate not found in xbox_settings");
      return false;
    }
    int baudrate = static_cast<int>(config["baudrate"]);
    if (!config.hasMember("communication_cycle")) {
      ROS_ERROR("communication_cycle not found in xbox_settings");
      return false;
    }
    double communication_cycle =
        static_cast<double>(config["communication_cycle"]);
  
    if (!config.hasMember("pub_cycle")) {
      ROS_ERROR("pub_cycle not found in xbox_settings");
      return false;
    }
    double pub_cycle = static_cast<double>(config["pub_cycle"]);
  
    if (!config.hasMember("joy_deadzone")) {
      ROS_ERROR("joy_deadzone not found in xbox_settings");
      return false;
    }
    deadzone = static_cast<double>(config["joy_deadzone"]);
    if (!config.hasMember("max_vel")) {
      ROS_ERROR("max_vel not found in xbox_settings");
      return false;
    }
    max_vel_ = static_cast<double>(config["max_vel"]);
  
    /* finish the controller manager config */
    if (!nh.getParam("controller_manager_settings", config)) {
      ROS_ERROR("failed to get param: controller_manager_settings");
      return false;
    }
    // 要加载的控制器，这里的列表同样会是在程序退出时的unload掉的，程序运行时，会先将他们都switch启动
    if (!config.hasMember("controller_to_load")) {
      ROS_ERROR("controller_to_load not found in controller_manager_settings");
      return false;
    } else {
      if (config["controller_to_load"].getType() ==
          XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < config["controller_to_load"].size(); ++i) {
          std::string controller_name =
              static_cast<std::string>(config["controller_to_load"][i]);
          controller_name_to_load_.push_back(controller_name);
        }
      } else {
        ROS_ERROR("controller_to_load is not an array");
        return false;
      }
    }
    // 程序运行中要启动的控制器列表
    if (!config.hasMember("controller_to_start")) {
      ROS_ERROR("controller_to_start not found in controller_manager_settings");
      return false;
    } else {
      if (config["controller_to_start"].getType() ==
          XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < config["controller_to_start"].size(); ++i) {
          std::string controller_name =
              static_cast<std::string>(config["controller_to_start"][i]);
          controller_name_to_start_.push_back(controller_name);
        }
      } else {
        ROS_ERROR("controller_to_start is not an array");
        return false;
      }
    }
    // 程序运行中要停止的控制器列表
    if (!config.hasMember("controller_to_stop")) {
      ROS_ERROR("controller_to_stop not found in controller_manager_settings");
      return false;
    } else {
      if (config["controller_to_stop"].getType() ==
          XmlRpc::XmlRpcValue::TypeArray) {
        for (int i = 0; i < config["controller_to_stop"].size(); ++i) {
          std::string controller_name =
              static_cast<std::string>(config["controller_to_stop"][i]);
          controller_name_to_stop_.push_back(controller_name);
        }
      } else {
        ROS_ERROR("controller_to_stop is not an array");
        return false;
      }
    }
  
    nh_ = nh;
    if (!serial_.isOpen()) {
      serial_.setPort(usb_port);
      serial_.setBaudrate(baudrate);
      serial::Timeout time_out = serial::Timeout::simpleTimeout(100);
      serial_.setTimeout(time_out);
      serial_.open();
      serial_.flushInput();
  
    } else
      return false;
    /* 初始化遥杆值 */
    xbox_data_.joyLHori = JOY_CENTER;
    xbox_data_.joyLVert = JOY_CENTER;
    xbox_data_.joyRHori = JOY_CENTER;
    xbox_data_.joyRVert = JOY_CENTER;
  
    /* worker relative */
    update_worker_ = std::make_shared<any_worker::Worker>(
        "updateWorker", communication_cycle,
        std::bind(&PboxRemote::updateWorkerCb, this, std::placeholders::_1));

    publish_worker_ = std::make_shared<any_worker::Worker>(
        "publishWorker", pub_cycle,
        std::bind(&PboxRemote::publishWorkerCb, this, std::placeholders::_1));

    send_worker_ = std::make_shared<any_worker::Worker>(
        "sendWorker", communication_cycle,
        std::bind(&PboxRemote::sendWorkerCb, this, std::placeholders::_1));

    /* publisher relative */
    cmd_vel_publisher_ =
        std::make_shared<any_node::ThreadedPublisher<geometry_msgs::Twist>>(
            nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10), 50, true);
    upperboard_cmd_publisher_ = std::make_shared<
        any_node::ThreadedPublisher<ros_ecat_msgs::UpperBoardCmd>>(
        nh.advertise<ros_ecat_msgs::UpperBoardCmd>("/upper_board/upperboard_cmd",
                                                   10),
        50, true);
    ops9_cmd_publisher_ =
        std::make_shared<any_node::ThreadedPublisher<ros_ecat_msgs::Ops9Command>>(
            nh.advertise<ros_ecat_msgs::Ops9Command>(
                "/controllers/Ops9Controller/action_cmd", 10),
            50, true);
  
    remote_state_publisher_ = 
        std::make_shared<any_node::ThreadedPublisher<ros_ecat_msgs::RemoteState>>(
            nh.advertise<ros_ecat_msgs::RemoteState>(
                "/remote_state", 10),50, true);
  
    swerve_init_publisher_ = 
        std::make_shared<any_node::ThreadedPublisher<std_msgs::String>>(
            nh.advertise<std_msgs::String>(
                "/controllers/chassis_controller/init_steer", 10),50, true);
  
  
    /* signal relative */
    signal_handler::SignalHandler::bindAll(&PboxRemote::handleSignal, this);
  
    /* controller manager relative */
    controller_load_client_ =
        nh.serviceClient<controller_manager_msgs::LoadController>(
            "/controller_manager/load_controller");
  
    controller_unload_client_ =
        nh.serviceClient<controller_manager_msgs::UnloadController>(
            "/controller_manager/unload_controller");
    controller_switch_client_ =
        nh.serviceClient<controller_manager_msgs::SwitchController>(
            "/controller_manager/switch_controller");


    pbox_data_sub_ = nh.subscribe<ros_ecat_msgs::PboxMsg>(
        "/pbox_data", 1, &PboxRemote::PboxDataCallback, this);

    yolo_data_sub_ = nh.subscribe<std_msgs::Float32MultiArray>(
        "/yolo/offset", 1, &PboxRemote::YoloDataCallback, this);

    // controller_load_client_.waitForExistence();
    // controller_unload_client_.waitForExistence();
    // controller_switch_client_.waitForExistence();
  
    /* 启动服务先： */
    // requestControllerLoad();
  
    /* worker start */
    update_worker_->start(45);
    publish_worker_->start(45);
    send_worker_->start(20);
  
    return true;
  }
  
  void PboxRemote::read(const ros::Time& time) {
    size_t n = serial_.available();
    uint8_t serial_buffer;
    static uint8_t i;
    // ROS_INFO_STREAM("N: "<<n);
  
    //缓存是空的话直接飞走
    if (!n) return;
  
    serial_.read(&serial_buffer, 1);
    // ROS_INFO_STREAM("flag:"<<(int)fsm_flag_);
    //接收状态坤
    switch (fsm_flag_) {
      case 0:
        if (serial_buffer == 0xFC)
          fsm_flag_++;
        else
          fsm_flag_ = 0;
        break;
      case 1:
        if (serial_buffer == 0xFB)
          fsm_flag_++;
        else
          fsm_flag_ = 0;
        break;
      case 2:
        if (serial_buffer == 0x01)
          fsm_flag_++;
        else
          fsm_flag_ = 0;
        break;
      case 3:
        if (serial_buffer == 43)
          fsm_flag_++;
        else
          fsm_flag_ = 0;
        break;
      case 4:
        buffer_[i] = serial_buffer;
        i++;
        if (i == 43) {
          fsm_flag_++;
          i = 0;
        }
        break;
      case 5:
        i++;
        if (i == 2) fsm_flag_++;  //跳过两位crc
        break;
      case 6:
        if (serial_buffer == 0xFD)
          fsm_flag_++;
        else {
          fsm_flag_ = 0;
          i = 0;
          memset(buffer_, 0, 43);
        }
        break;
      case 7:
        if (serial_buffer == 0xFE) unpack();
        fsm_flag_ = 0;
        i = 0;
        serial_.flushInput();  //每处理一次数据就清空缓存区，保证数据都是最新的
        break;
      default:
        fsm_flag_ = 0;
        break;
    }
  }
  
  void PboxRemote::unpack() {
    xbox_data_.btnY = buffer_[0];
    xbox_data_.btnB = buffer_[1];
    xbox_data_.btnA = buffer_[2];
    xbox_data_.btnX = buffer_[3];
    xbox_data_.btnShare = buffer_[4];
    xbox_data_.btnStart = buffer_[5];
    xbox_data_.btnBack = buffer_[6];
    xbox_data_.btnXbox = buffer_[7];
    xbox_data_.btnLB = buffer_[8];
    xbox_data_.btnRB = buffer_[9];
    xbox_data_.btnLS = buffer_[10];
    xbox_data_.btnRS = buffer_[11];
    xbox_data_.btnDirUp = buffer_[12];
    xbox_data_.btnDirLeft = buffer_[13];
    xbox_data_.btnDirRight = buffer_[14];
    xbox_data_.btnDirDown = buffer_[15];

    xbox_data_.SWA = buffer_[16];
    xbox_data_.SWB = buffer_[17];
    xbox_data_.SWC = buffer_[18];

    xbox_data_.joyLHori = (uint16_t)((buffer_[19] << 8) | buffer_[20]);
    xbox_data_.joyLVert = (uint16_t)((buffer_[21] << 8) | buffer_[22]);
    xbox_data_.joyRHori = (uint16_t)((buffer_[23] << 8) | buffer_[24]);
    xbox_data_.joyRVert = (uint16_t)((buffer_[25] << 8) | buffer_[26]);
    xbox_data_.trigLT = (uint16_t)((buffer_[27] << 8) | buffer_[28]);
    xbox_data_.trigRT = (uint16_t)((buffer_[29] << 8) | buffer_[30]);
    memcpy(PboxData_.data, buffer_ + 31, 4);
    xbox_data_.UIPosX = PboxData_.UIData;
    memcpy(PboxData_.data + 4, buffer_ + 35, 4);
    xbox_data_.UIPosY = PboxData_.UIData;
    memcpy(PboxData_.data + 8, buffer_ + 39, 4);
    xbox_data_.ShooterFactor = PboxData_.UIData;
    ROS_INFO_STREAM("RHori: "<<xbox_data_.joyRHori);
    
  }
  
  bool PboxRemote::updateWorkerCb(const any_worker::WorkerEvent& event) {
    read(ros::Time::now());
    return true;
  }
  
  bool PboxRemote::publishWorkerCb(const any_worker::WorkerEvent& event) {
    // 计算原始差值
    double diff_x = xbox_data_.joyLVert - JOY_CENTER;
    double diff_y = xbox_data_.joyLHori - JOY_CENTER;
    double diff_ang = xbox_data_.joyRHori - JOY_CENTER;
  
    /*如果使用pbox，可以考虑不设置死区*/

    // 对原始差值先检测死区
    // 对原始差值先检测死区
  if (fabs(diff_x) < deadzone) diff_x = 0.0;
  else 
  {
    if(diff_x<0)diff_x += deadzone;
    else diff_x -= deadzone;
  }

  if (fabs(diff_y) < deadzone) diff_y = 0.0;
  else 
  {
    if(diff_y<0)diff_y += deadzone;
    else diff_y -= deadzone;
  }

  if (fabs(diff_ang) < deadzone) diff_ang = 0.0;
  else 
  {
    if(diff_ang<0)diff_ang += deadzone;
    else diff_ang -= deadzone;
  }

    // if (fabs(diff_x) < deadzone) diff_x = 0.0;
    // if (fabs(diff_y) < deadzone) diff_y = 0.0;
    // if (fabs(diff_ang) < deadzone) diff_ang = 0.0;
  
    // 将差值归一化后乘上最大速度
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = diff_x / JOY_CENTER * max_vel_;
    cmd_vel.linear.y = diff_y / JOY_CENTER * max_vel_;
    cmd_vel.angular.z = -diff_ang / JOY_CENTER * max_vel_;
  
    ROS_INFO_STREAM_THROTTLE(0.5, "cmd_vel: " << cmd_vel.linear.x << " "
                                              << cmd_vel.linear.y << " "
                                              << cmd_vel.angular.z);
  
    ros_ecat_msgs::UpperBoardCmd upperboard_cmd;
    upperboard_cmd.stamp = ros::Time::now();
    upperboard_cmd.shooter_factor = xbox_data_.ShooterFactor;
    upperboard_cmd.shoot = xbox_data_.btnLB;
    upperboard_cmd.raiseUp = xbox_data_.btnDirUp;
    upperboard_cmd.putDown = xbox_data_.btnDirDown;
    upperboard_cmd.go1PoseUp = xbox_data_.btnDirRight;
    upperboard_cmd.go1PoseDown = xbox_data_.btnDirLeft;
    upperboard_cmd.sendShootMsg = xbox_data_.btnRB;
    if(xbox_data_.btnX)
    {
      upperboard_cmd.cylinder_test = 0;
    }
    else if(xbox_data_.btnY)
    {
      upperboard_cmd.cylinder_test = 1;
    }
  
    ros_ecat_msgs::Ops9Command ops9_cmd;
    ops9_cmd.stamp = ros::Time::now();
    ops9_cmd.name = "action1";
    ops9_cmd.cmd = "RESET";
    // if (xbox_data_.btnShare) ops9_cmd_publisher_->publish(ops9_cmd);
    
    // if(xbox_data_.SWA && last_swa_ == false)
    // {
    //   // requestControllerStart();
    //   remote_state_msg_.yaw_lock=2;
    //   remote_state_publisher_->publish(remote_state_msg_);
    //   last_swa_ = xbox_data_.SWA;
    // }
    // else if(!xbox_data_.SWA && last_swa_ == true)
    // {
    //   // requestControllerStop();
    //   remote_state_msg_.yaw_lock=0;
    //   last_swa_ = xbox_data_.SWA;
    // }
  
    // if(xbox_data_.SWB && !last_swb_)
    // {
    //   remote_state_msg_.yaw_lock=1;
    //   basket_lock_ = true;
    //   remote_state_publisher_->publish(remote_state_msg_);
    //   last_swb_ = xbox_data_.SWB;
    // }
    // else if(!xbox_data_.SWB && last_swb_)
    // {
    //   if(xbox_data_.SWA && last_swa_ == false)
    //   {
    //     remote_state_msg_.yaw_lock=2;
    //     remote_state_publisher_->publish(remote_state_msg_);
    //     last_swa_ = xbox_data_.SWA;
    //   }
    //   else if(!xbox_data_.SWA && last_swa_ == true)
    //   {
    //     remote_state_msg_.yaw_lock=0;
    //     remote_state_publisher_->publish(remote_state_msg_);
    //     last_swa_ = xbox_data_.SWA;
    //   }
    //   last_swb_ = xbox_data_.SWB;
    // }


    if(xbox_data_.trigLT>900.0)remote_state_msg_.brake_flag = true;
    else remote_state_msg_.brake_flag = false;

    if(xbox_data_.btnRB)
    {
      remote_state_msg_.shoot_flag = true;
      remote_state_msg_.yaw_lock=0;
    }
    else remote_state_msg_.shoot_flag = false;

    if(xbox_data_.btnLB)
    {
      remote_state_msg_.dribble_flag = true;
      remote_state_msg_.yaw_lock=0;
    }
    else remote_state_msg_.dribble_flag = false;

    if(xbox_data_.btnDirUp)
    {
      remote_state_msg_.net_rise = true;
    }
    else if(xbox_data_.btnDirDown)
    {
      remote_state_msg_.net_rise = false; 
    }

    if(xbox_data_.btnB)
    {
      remote_state_msg_.yaw_lock=1;
      remote_state_msg_.shoot_flag = false;       
    }
    else if(xbox_data_.btnY)
    {
      remote_state_msg_.yaw_lock=2;
      remote_state_msg_.shoot_flag = false;       

    }
    else if(xbox_data_.btnX)
    {
      remote_state_msg_.yaw_lock=3;
      remote_state_msg_.shoot_flag = false;       
    }
    else if(xbox_data_.btnA)
    {
      remote_state_msg_.yaw_lock=0;
      remote_state_msg_.dribble_flag = false;
    }
 
    remote_state_publisher_->publish(remote_state_msg_);
    
    std_msgs::String init_msg;
    init_msg.data = "START";
    if(xbox_data_.btnXbox)
    {
      swerve_init_publisher_->publish(init_msg);
    }
  
  
  
    cmd_vel_publisher_->publish(cmd_vel);
    // upperboard_cmd_publisher_->publish(upperboard_cmd);
  
    return true;
  }
  
  bool PboxRemote::sendWorkerCb(const any_worker::WorkerEvent& event) {

    uint8_t buffer_[15] = {0};  // 发送数据的缓冲区
    {
      std::lock_guard<std::mutex> lock(pbox_data_mutex_);
      ros_ecat_msgs::PboxMsg *pbox_data = pbox_data_buffer_.readFromRT();
      std_msgs::Float32MultiArray *yolo_data = yolo_data_buffer_.readFromRT();

      buffer_[0] = 0xFC;  // Start byte
      buffer_[1] = 0xFB;  // Second byte
      buffer_[2] = 0x01;
      buffer_[3] = 7;
      buffer_[4] = (uint8_t)pbox_data->swerve_mode;
      buffer_[5] = (uint8_t)pbox_data->lock_backet_flag;
      // PboxBuffer_.UIData = yolo_data->data[0];  // Copy first YOLO data
      // memcpy(buffer_ + 6, PboxBuffer_.data, 4);  // Copy Pbox data
      buffer_[10] = (uint8_t)pbox_data->camera_flag;
      buffer_[13] = 0xFD;  // End byte
      buffer_[14] = 0xFE;  // Checksum byte
    }
    
    // 发送数据到pbox
    serial_.write(buffer_, 15);
    return true;
  }

  void PboxRemote::handleSignal(int /* signum */) {
    update_worker_->stop();
    publish_worker_->stop();
    ros::shutdown();
    //   serial_.close();
  }
  
  void PboxRemote::requestControllerLoad() {
    for (const auto& controller_name : controller_name_to_load_) {
      // 加载控制器
      controller_manager_msgs::LoadController load_srv;
      load_srv.request.name = controller_name;
      if (controller_load_client_.call(load_srv)) {
        if (load_srv.response.ok) {
          ROS_INFO_STREAM("Loaded controller: " << controller_name);
        } else {
          ROS_ERROR_STREAM("Failed to load controller: " << controller_name);
          continue;
        }
      } else {
        ROS_ERROR_STREAM("Service call to load controller " << controller_name
                                                            << " failed.");
      }
      ros::Duration(0.5).sleep();  // 操作延时
  
      // 启动控制器：调用switch 服务
      controller_manager_msgs::SwitchController switch_srv;
      switch_srv.request.start_controllers.push_back(controller_name);
      switch_srv.request.strictness =
          controller_manager_msgs::SwitchController::Request::STRICT;
      switch_srv.request.start_asap = false;
      switch_srv.request.timeout = 5.0;  // 超时时间 5 秒
      if (controller_switch_client_.call(switch_srv)) {
        if (switch_srv.response.ok) {
          ROS_INFO_STREAM("Started controller: " << controller_name);
        } else {
          ROS_ERROR_STREAM("Failed to start controller: " << controller_name);
          continue;
        }
      } else {
        ROS_ERROR_STREAM("Service call to start controller " << controller_name
                                                             << " failed.");
      }
      ros::Duration(0.5).sleep();  // 操作延时
    }
  }
  void PboxRemote::requestControllerUnload() {
    // 依次停止程序启动时加载的所有控制器
    for (const auto& controller_name : controller_name_to_load_) {
      controller_manager_msgs::UnloadController unload_srv;
      unload_srv.request.name = controller_name;
      if (controller_unload_client_.call(unload_srv)) {
        if (unload_srv.response.ok) {
          ROS_INFO_STREAM("Unloaded controller: " << controller_name);
        } else {
          ROS_ERROR_STREAM("Failed to unload controller: " << controller_name);
          continue;
        }
      } else {
        ROS_ERROR_STREAM("Service call to unload controller " << controller_name
                                                              << " failed.");
      }
    }
  }
  
  void PboxRemote::requestControllerStop() {
    // 依次停止设定的在程序运行期间允许关闭的控制器
    for (const auto& controller_name : controller_name_to_stop_) {
      controller_manager_msgs::SwitchController switch_srv;
      switch_srv.request.stop_controllers.push_back(controller_name);
      switch_srv.request.strictness =
          controller_manager_msgs::SwitchController::Request::STRICT;
      switch_srv.request.start_asap = false;
      switch_srv.request.timeout = 5.0;  // 超时时间 5 秒
      if (controller_switch_client_.call(switch_srv)) {
        if (switch_srv.response.ok) {
          ROS_INFO_STREAM("Stopped controller: " << controller_name);
        } else {
          ROS_ERROR_STREAM("Failed to stop controller: " << controller_name);
          continue;
        }
      } else {
        ROS_ERROR_STREAM("Service call to stop controller " << controller_name
                                                            << " failed.");
      }
    }
  }
  
  void PboxRemote::requestControllerStart() {
    for (const auto& controller_name : controller_name_to_start_) {
      controller_manager_msgs::SwitchController switch_srv;
      switch_srv.request.start_controllers.push_back(controller_name);
      switch_srv.request.strictness =
          controller_manager_msgs::SwitchController::Request::STRICT;
      switch_srv.request.start_asap = false;
      switch_srv.request.timeout = 5.0;  // 超时时间 5 秒
      if (controller_switch_client_.call(switch_srv)) {
        if (switch_srv.response.ok) {
          ROS_INFO_STREAM("Started controller: " << controller_name);
        } else {
          ROS_ERROR_STREAM("Failed to start controller: " << controller_name);
          continue;
        }
      } else {
        ROS_ERROR_STREAM("Service call to start controller " << controller_name
                                                             << " failed.");
      }
    }
  }


  void PboxRemote::PboxDataCallback(const ros_ecat_msgs::PboxMsg::ConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(pbox_data_mutex_);
    pbox_data_buffer_.writeFromNonRT(*msg);
  }

  void PboxRemote::YoloDataCallback(const std_msgs::Float32MultiArrayConstPtr &msg)
  {
    yolo_data_buffer_.writeFromNonRT(*msg);
  }

}