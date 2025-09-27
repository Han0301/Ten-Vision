/**
 * @file radio_remote.cpp
 * @author py <997276894@qq.com>
 * @brief  航模遥控器接收
 * @version 0.1
 * @date  2025-07-31  
 * 
 * @copyright Copyright (c)  2025 
 * 
 * @attention  使用termios库修改自定义波特率
 * @note  
 */
 #include <radio_hw/radio_remote.h>

 namespace radio_hw {

RadioRemote::RadioRemote() : crc_(0xD5)
{
} // 初始化CRC，CRSF协议使用0xD5

 
bool RadioRemote::init(ros::NodeHandle& nh) {
  XmlRpc::XmlRpcValue config;
  if (!nh.getParam("radio_settings", config)) {
    ROS_ERROR("Failed to get param: radio_settings");
    return false;
  }
  if (!config.hasMember("usb_port")) {
    ROS_ERROR("usb_port not found in radio_settings");
    return false;
  }
  std::string usb_port = static_cast<std::string>(config["usb_port"]);
  if (!config.hasMember("baudrate")) {
    ROS_ERROR("baudrate not found in radio_settings");
    return false;
  }
  int baudrate = static_cast<int>(config["baudrate"]);
  ROS_INFO_STREAM("baund: "<<baudrate);
  if (!config.hasMember("communication_cycle")) {
    ROS_ERROR("communication_cycle not found in radio_settings");
    return false;
  }
  double communication_cycle = static_cast<double>(config["communication_cycle"]);

  if (!config.hasMember("pub_cycle")) {
    ROS_ERROR("pub_cycle not found in radio_settings");
    return false;
  }
  double pub_cycle = static_cast<double>(config["pub_cycle"]);

  if (!config.hasMember("joy_deadzone")) {
    ROS_ERROR("joy_deadzone not fobufferund in radio_settings");
    return false;
  }
  deadzone = static_cast<double>(config["joy_deadzone"]);
  if (!config.hasMember("max_vel")) {
    ROS_ERROR("max_vel not found in radio_settings");
    return false;
  }
  max_vel_ = static_cast<double>(config["max_vel"]);

  if (!config.hasMember("CRSF_NUM_CHANNELS")) {
  ROS_ERROR("CRSF_NUM_CHANNELS not found in radio_settings");
  return false;
}
CRSF_NUM_CHANNELS_ = static_cast<int>(config["CRSF_NUM_CHANNELS"]);

if (!config.hasMember("CRSF_CHANNEL_VALUE_MIN")) {
  ROS_ERROR("CRSF_CHANNEL_VALUE_MIN not found in radio_settings");
  return false;
}
CRSF_CHANNEL_VALUE_MIN_ = static_cast<int>(config["CRSF_CHANNEL_VALUE_MIN"]);

if (!config.hasMember("CRSF_CHANNEL_VALUE_MID")) {
  ROS_ERROR("CRSF_CHANNEL_VALUE_MID not found in radio_settings");
  return false;
}
CRSF_CHANNEL_VALUE_MID_ = static_cast<int>(config["CRSF_CHANNEL_VALUE_MID"]);

if (!config.hasMember("CRSF_CHANNEL_VALUE_MAX")) {
  ROS_ERROR("CRSF_CHANNEL_VALUE_MAX not found in radio_settings");
  return false;
}
CRSF_CHANNEL_VALUE_MAX_ = static_cast<int>(config["CRSF_CHANNEL_VALUE_MAX"]);
if (!config.hasMember("CRSF_MAX_PACKET_SIZE")) {
  ROS_ERROR("CRSF_MAX_PACKET_SIZE not found in radio_settings");
  return false;
}
CRSF_MAX_PACKET_SIZE_ = static_cast<int>(config["CRSF_MAX_PACKET_SIZE"]);
CRSF_MAX_PAYLOAD_LEN_ = CRSF_MAX_PACKET_SIZE_ - 4;

if (!config.hasMember("CRSF_CRC_POLY")) {
  ROS_ERROR("CRSF_CRC_POLY not found in radio_settings");
  return false;
}
CRSF_CRC_POLY_ = static_cast<int>(config["CRSF_CRC_POLY"]);


//  /* finish the controller manager config */使用termios2配置波特率420000的
//  if (!nh.getParam("controller_manager_settings", config)) {
//    ROS_ERROR("failed to get param: controller_manager_settings");
//    return false;
//  }
//  // 要加载的控制器，这里的列表同样会是在程序退出时的unload掉的，程序运行时，会先将他们都switch启动
//  if (!config.hasMember("controller_to_load")) {
//    ROS_ERROR("controller_to_load not found in controller_manager_settings");
//    return false;
//  } else {
//    if (config["controller_to_load"].getType() ==
//        XmlRpc::XmlRpcValue::TypeArray) {
//      for (int i = 0; i < config["controller_to_load"].size(); ++i) {
//        std::string controller_name =
//            static_cast<std::string>(config["controller_to_load"][i]);
//        controller_name_to_load_.push_back(controller_name);
//      }
//    } else {
//      ROS_ERROR("controller_to_load is not an array");
//      return false;
//    }
//  }
//  // 程序运行中要启动的控制器列表
//  if (!config.hasMember("controller_to_start")) {
//    ROS_ERROR("controller_to_start not found in controller_manager_settings");
//    return false;
//  } else {
//    if (config["controller_to_start"].getType() ==
//        XmlRpc::XmlRpcValue::TypeArray) {
//      for (int i = 0; i < config["controller_to_start"].size(); ++i) {
//        std::string controller_name =
//            static_cast<std::string>(config["controller_to_start"][i]);
//        controller_name_to_start_.push_back(controller_name);
//      }
//    } else {
//      ROS_ERROR("controller_to_start is not an array");
//      return false;
//    }
//  }
  // 程序运行中要停止的控制器列表
//  if (!config.hasMember("controller_to_stop")) {
//    ROS_ERROR("controller_to_stop not found in controller_manager_settings");
//    return false;
//  } else {
//    if (config["controller_to_stop"].getType() ==
//        XmlRpc::XmlRpcValue::TypeArray) {
//      for (int i = 0; i < config["controller_to_stop"].size(); ++i) {
//        std::string controller_name =
//            static_cast<std::string>(config["controller_to_stop"][i]);
//        controller_name_to_stop_.push_back(controller_name);
//      }
//    } else {
//      ROS_ERROR("controller_to_stop is not an array");
//      return false;
//    }
//  }

nh_ = nh;

  /* worker relative */
  update_worker_ = std::make_shared<any_worker::Worker>(
      "updateWorker", communication_cycle,
      std::bind(&RadioRemote::updateWorkerCb, this, std::placeholders::_1));
  publish_worker_ = std::make_shared<any_worker::Worker>(
      "publishWorker", pub_cycle,
      std::bind(&RadioRemote::publishWorkerCb, this, std::placeholders::_1));
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

  position_reset_pub_ = std::make_shared<any_node::ThreadedPublisher<ros_ecat_msgs::PositionSensorCmd>>(
    nh.advertise<ros_ecat_msgs::PositionSensorCmd>("/imu_cmd", 1), 15, true);
              
  camera_sub_ = nh.subscribe<vision_msgs::BoundingBox2DArray>("/yolo/detections", 1, &RadioRemote::CameraCallback, this);

  pbox_data_sub_ = nh.subscribe<ros_ecat_msgs::PboxMsg>("/pbox_data", 1, &RadioRemote::PboxDataCallback, this);

  odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", 1, &RadioRemote::OdomCallback, this);

  imu_sub_ = nh.subscribe<geometry_msgs::Vector3Stamped>("/mahony/euler", 1, &RadioRemote::imuCallback, this);

  radar_cmd_sub_ = nh.subscribe<std_msgs::Float32>("/radar_shoot_cmd", 1, &RadioRemote::radarCmdCallback, this);
  camera_cmd_sub_ = nh.subscribe<std_msgs::Float32>("/camera_shoot_cmd", 1, &RadioRemote::cameraCmdCallback, this);
  init_state_sub_ = nh.subscribe<ros_ecat_msgs::InitState>("/init_state", 1, &RadioRemote::InitStateCallback, this);

  


  /* signal relative */
  signal_handler::SignalHandler::bindAll(&RadioRemote::handleSignal, this);

//  /* controller manager relative */
//  controller_load_client_ =
//      nh.serviceClient<controller_manager_msgs::LoadController>(
//          "/controller_manager/load_controller");

//  controller_unload_client_ =
//      nh.serviceClient<controller_manager_msgs::UnloadController>(
//          "/controller_manager/unload_controller");
//  controller_switch_client_ =
//      nh.serviceClient<controller_manager_msgs::SwitchController>(
//          "/controller_manager/switch_controller");
//  controller_load_client_.waitForExistence();
//  controller_unload_client_.waitForExistence();
//  controller_switch_client_.waitForExistence();

  /* 启动服务先： */
  // requestControllerLoad();

  int check=uart.SerialInit(fd,usb_port.c_str(),420000);
  ROS_INFO_STREAM("check: "<<check);
  /* worker start */
  update_worker_->start(45);
  publish_worker_->start(45);

  return true;
}

void RadioRemote::read_data(const ros::Time& time) {

uint8_t serial_buffer[256];
static uint8_t i = 0;
int bytesRead = read(fd, serial_buffer, sizeof(serial_buffer));
if (bytesRead <= 0)
{
  // ROS_INFO("no data");
}
else
{
    
  for ( i = 0; i < bytesRead; i++)
  {
    if (serial_buffer[i] == CRSF_ADDRESS_FLIGHT_CONTROLLER)
    {
      current_rc_frame_.device_addr = serial_buffer[i];
      rx_state_ = CRSF_WAITING_FOR_FRAMESIZE;
      break;
    }
  }

  if(rx_state_==CRSF_WAITING_FOR_FRAMESIZE && i+3+CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE<=bytesRead)
  {
    current_rc_frame_.frame_size = serial_buffer[i+1];
    if(current_rc_frame_.frame_size == (CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC))
    {
      current_rc_frame_.type = serial_buffer[i+2];
      if(current_rc_frame_.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
      {
        payload_ptr_ = (uint8_t *)&current_rc_frame_.channels;
        for(packet_byte_index_ = 0; packet_byte_index_ < CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE; packet_byte_index_++)
        {
          payload_ptr_[packet_byte_index_] = serial_buffer[i+3+packet_byte_index_];
        }
        unpack();
      }
    }
  }
  rx_state_=CRSF_WAITING_FOR_ADDRESS;
  return;
}

// 打印原始数据（调试用）
// std::stringstream ss;
// for (uint8_t byte : serial_buffer) {
//   ss << "0x" << std::hex << static_cast<int>(byte) << " ";
// }
// ROS_INFO_STREAM("Raw data received: " << ss.str());
}
 
void RadioRemote::unpack() {

  channels_[0] = current_rc_frame_.channels.ch0;
  channels_[1] = current_rc_frame_.channels.ch1;
  channels_[2] = current_rc_frame_.channels.ch2;
  channels_[3] = current_rc_frame_.channels.ch3;
  channels_[4] = current_rc_frame_.channels.ch4;
  channels_[5] = current_rc_frame_.channels.ch5;
  channels_[6] = current_rc_frame_.channels.ch6;
  channels_[7] = current_rc_frame_.channels.ch7;
  channels_[8] = current_rc_frame_.channels.ch8;
  channels_[9] = current_rc_frame_.channels.ch9;
  channels_[10] = current_rc_frame_.channels.ch10;
  channels_[11] = current_rc_frame_.channels.ch11;
  channels_[12] = current_rc_frame_.channels.ch12;
  channels_[13] = current_rc_frame_.channels.ch13;
  channels_[14] = current_rc_frame_.channels.ch14;
  channels_[15] = current_rc_frame_.channels.ch15;

  map_value_compute();

  if(!channels_init_flag_)
  {
    channels_init_flag_ = true;
    memcpy(channels_init_,channels_,16); //初始化通道值检测
  }
}



 
bool RadioRemote::updateWorkerCb(const any_worker::WorkerEvent& event) {
// read_data(ros::Time::now());
  get_state(ros::Time::now());
  return true;
}

bool RadioRemote::publishWorkerCb(const any_worker::WorkerEvent& event) {

  // 将差值归一化后乘上最大速度

std_msgs::String init_msg;
init_msg.data = "START";
double max_set_vel = max_vel_ * radio_data_.roll_map;

//**************************右手摇杆操作**************************************
if(radio_data_.right_V_map>0.8)
{
  if(radio_data_.right_H_map<-0.8)
  {
    if(init_basket_)
    {
      remote_state_msg.dribble_flag = true;
      remote_state_msg.yaw_lock=0;
    }
    else
    {
      remote_state_msg.set_finish = true;
    }

  }
  else if(radio_data_.right_H_map>0.8)
  {
    remote_state_msg.dribble_flag = false;
    swerve_init_publisher_->publish(init_msg);
    remote_state_msg.set_finish = false;
  }
  else
  {
    remote_state_msg.dribble_flag = false;
    remote_state_msg.set_finish = false;
  }
  remote_state_msg.brake_flag = false;
}
else if(radio_data_.right_V_map<-0.8)
{
  remote_state_msg.brake_flag = true;
  if(radio_data_.right_H_map>0.8)
  {
    ros_ecat_msgs::PositionSensorCmd imu_cmd;
    imu_cmd.cmd="RESET";
    position_reset_pub_->publish(imu_cmd);
  }
}
else 
{
  remote_state_msg.set_finish = false;
  remote_state_msg.dribble_flag = false;
  remote_state_msg.brake_flag = false;
}



if(radio_data_.right_V_map==0)cmd_vel.angular.z = -radio_data_.right_H_map* max_set_vel;


if(radio_data_.btnSC==0)
{
  if(radio_data_.btnSB==0)
  {
    remote_state_msg.yaw_lock=0;
  }
  else if(radio_data_.btnSB==1)remote_state_msg.yaw_lock=1;
  else if(radio_data_.btnSB==2)remote_state_msg.yaw_lock=3;
  remote_state_msg.net_rise = false;
}
else if(radio_data_.btnSC==1)
{
  remote_state_msg.net_rise = true;
  if(radio_data_.btnSB==0)
  {
    remote_state_msg.yaw_lock=0;
  }
  else if(radio_data_.btnSB==1)remote_state_msg.yaw_lock=1;
  else if(radio_data_.btnSB==2)remote_state_msg.yaw_lock=3;
  
}
else if(radio_data_.btnSC==2)remote_state_msg.yaw_lock=2;//传球锁车

if(radio_data_.btnSE)
{
  if(init_basket_)remote_state_msg.shoot_flag = true;
  else remote_state_msg.set_offset = true;
  // remote_state_msg.shoot_flag = true;
}
else 
{
  remote_state_msg.shoot_flag = false;
  remote_state_msg.set_offset = false;
}


if(radio_data_.btnSA)remote_state_msg.world_flag = true;
else remote_state_msg.world_flag = false;


if(radio_data_.btnSD)remote_state_msg.camera_shoot = true;
else remote_state_msg.camera_shoot = false;

remote_state_publisher_->publish(remote_state_msg);



cmd_vel.linear.x = radio_data_.left_V_map * max_set_vel;
cmd_vel.linear.y = -radio_data_.left_H_map * max_set_vel;


ROS_INFO_STREAM_THROTTLE(0.5, "cmd_vel: " << cmd_vel.linear.x << " "
                                          << cmd_vel.linear.y << " "
                                          << cmd_vel.angular.z);
cmd_vel_publisher_->publish(cmd_vel);


// send_state(1.0,2.0,3.0,100.0,5.0,6.0,3.14);
return true;
}

void RadioRemote::handleSignal(int /* signum */) {
  update_worker_->stop();
  publish_worker_->stop();
  ros::shutdown();
}
 

void RadioRemote::map_value_compute()
{
  // ROS_INFO_STREAM("channel0: "<<channels_[0]<<"channel1: "<<channels_[1]<<"channel2: "<<channels_[2]<<"channel3: "<<channels_[3]<<"channel4: "<<channels_[4]); 
  // ROS_INFO_STREAM("channel5: "<<channels_[5]<<"channel6: "<<channels_[6]<<"channel7: "<<channels_[7]<<"channel8: "<<channels_[8]<<"channel9: "<<channels_[9]); 
  // ROS_INFO_STREAM("channel10: "<<channels_[10]<<"channel11: "<<channels_[11]<<"channel12: "<<channels_[12]<<"channel13: "<<channels_[13]<<"channel14: "<<channels_[14]); 
  double joy;
  
  //右水平
  if (channels_[0] - channels_init_[0] > JOY_Death_Zone || channels_[0] - channels_init_[0] < -JOY_Death_Zone) 
  {
    joy = (double)(65535.0 / (JOY_RHORI_MAX - JOY_RHORI_MIN) * (channels_[0] - channels_init_[0]))/32767.0;
    if (joy > 1.0)
      joy = 1.0;
    else if (joy < -1.0)
      joy = -1.0;
    radio_data_.right_H_map = joy;
  }
  else radio_data_.right_H_map = 0.0;

  //右竖直
  if (channels_[1] - channels_init_[1] > JOY_Death_Zone || channels_[1] - channels_init_[1] < -JOY_Death_Zone) 
  {
    joy = (double)(65535.0 / (JOY_RVERT_MAX - JOY_RVERT_MIN) * (channels_[1] - channels_init_[1]))/32767.0;
    if (joy > 1.0)
      joy = 1.0;
    else if (joy < -1.0)
      joy = -1.0;
    radio_data_.right_V_map = joy;
  }
  else radio_data_.right_V_map = 0.0;

  //左竖直
  if (channels_[2] - channels_init_[2] > JOY_Death_Zone || channels_[2] - channels_init_[2] < -JOY_Death_Zone) 
  {
    joy = (double)(65535.0 / (JOY_LVERT_MAX - JOY_LVERT_MIN) * (channels_[2] - channels_init_[2]))/32767.0;
    if (joy > 1.0)
      joy = 1.0;
    else if (joy < -1.0)
      joy = -1.0;
    radio_data_.left_V_map = joy;
  }
  else radio_data_.left_V_map = 0.0;

  //左水平
  if (channels_[3] - channels_init_[3] > JOY_Death_Zone || channels_[3] - channels_init_[3] < -JOY_Death_Zone) 
  {
    joy = (double)(65535.0 / (JOY_LHORI_MAX - JOY_LHORI_MIN) * (channels_[3] - channels_init_[3]))/32767.0;
    if (joy > 1.0)
      joy = 1.0;
    else if (joy < -1.0)
      joy = -1.0;
    radio_data_.left_H_map = joy;
  }
  else radio_data_.left_H_map = 0.0;
  if (channels_[9] < 191)
  {
    radio_data_.roll_map = 0.0f;
  }
  if (channels_[9] > 1792)
  {
    radio_data_.roll_map = 1.0f;
  }
  if (channels_[9] >= 191 && channels_[9] <= 1792)
  {
    radio_data_.roll_map = (float)(channels_[9] -  191) / 1601.0f;
    if(radio_data_.roll_map<0.0)radio_data_.roll_map=0.0;
    else if(radio_data_.roll_map>1.0)radio_data_.roll_map = 1.0;
  }

  if (channels_[4] == BTN_ON)
  {
    radio_data_.btnSA = 1;
  }
  if (channels_[4] == BTN_OFF)
  {
    radio_data_.btnSA = 0;
  }

  if (channels_[8] == BTN_ON)
  {
    radio_data_.btnSD = 1;
  }
  if (channels_[8] == BTN_OFF)
  {
    radio_data_.btnSD = 0;
  }

  if (channels_[6] == BTN_ON)
  {
    radio_data_.btnSE = 1;
  }
  if (channels_[6] == BTN_OFF)
  {
    radio_data_.btnSE = 0;
  }

  if (channels_[5] == BTN_1)
  {
    radio_data_.btnSB = 0;
  }
  if (channels_[5] == BTN_2)
  {
    radio_data_.btnSB = 1;
  }
  if (channels_[5] == BTN_3)
  {
    radio_data_.btnSB = 2;
  }

  if (channels_[7] == BTN_1)
  {
    radio_data_.btnSC = 0;
  }
  if (channels_[7] == BTN_2)
  {
    radio_data_.btnSC = 1;
  }
  if (channels_[7] == BTN_3)
  {
    radio_data_.btnSC = 2;
  }
}

void RadioRemote::send_state(uint8_t mode,double posx,double posy,float max_speed,float radar_yaw,float radar_shoot_cmd,float camera_shoot_cmd,float camera_catch)
{
  float rsc=round(radar_shoot_cmd*100.0); 
  uint16_t rshoot1 = static_cast<uint16_t>(round(rsc));
  uint16_t rshoot2 = static_cast<uint16_t>((rsc-rshoot1)*1000);
  uint16_t cshoot_cmd = static_cast<uint16_t>(round(camera_shoot_cmd*1000));
  float ryaw = radar_yaw/57.3;
  float cc = camera_catch;
  float Mspeed = max_speed/57.3;
  try{
    sendGps(posx,posy,rshoot2,rshoot1,cshoot_cmd,mode);
    sendAttitude(Mspeed,ryaw,cc);
  }catch(const std::runtime_error& e){
    ROS_ERROR("Send data error!");
    update_worker_->stop();
    publish_worker_->stop();
    ros::shutdown();
  }
  return;
}

void RadioRemote::get_state(const ros::Time& time)
{
  nav_msgs::Odometry *odom_msg = odom_buffer_.readFromRT();
  geometry_msgs::Vector3Stamped *imu_msg = imu_buffer_.readFromRT();
  std_msgs::Float32 *radar_cmd_msg = radar_cmd_buffer_.readFromRT();
  std_msgs::Float32 *camera_cmd_msg = camera_cmd_buffer_.readFromRT();
  ros_ecat_msgs::PboxMsg *pbox_msg = pbox_data_buffer_.readFromRT();
  ros_ecat_msgs::InitState *init_msg = init_state_buffer_.readFromRT();

  uint8_t mode = pbox_msg->swerve_mode;
  double posx;
  double posy;
  float radar_yaw = imu_msg->vector.z;
  float radar_shoot_cmd = radar_cmd_msg->data;
  float camera_shoot_cmd = camera_cmd_msg->data;
  float camera_catch;
  float m_s = max_vel_ * radio_data_.roll_map;
  
  // init_camera_ = init_msg->init_camera_flag;
  init_basket_ = init_msg->init_basket_flag;
  // if(!init_msg->init_camera_flag)
  // {
  //   posx = init_msg->camera_center;
  // }
  // else if(!init_msg->init_basket_flag)
  // {
  //   posx = init_msg->basket_x;
  //   posy = init_msg->basket_y;
  // }
  // else
  // {
    posx = odom_msg->pose.pose.position.x;
    posy = odom_msg->pose.pose.position.y;
    // mode = pbox_msg->swerve_mode;
  // }

  if(camera_init_)
  {
    vision_msgs::BoundingBox2DArray *camera_msg = camera_buffer_.readFromRT();
    if(camera_msg->boxes[0].center.x==-1 && camera_msg->boxes[0].center.y==0)camera_catch = 0;
    else camera_catch = 1;
  }

  send_state(mode,posx,posy,m_s,radar_yaw,radar_shoot_cmd,camera_shoot_cmd,camera_catch);
}

void RadioRemote::PboxDataCallback(const ros_ecat_msgs::PboxMsg::ConstPtr& msg)
{
  pbox_data_buffer_.writeFromNonRT(*msg);
}

void RadioRemote::InitStateCallback(const ros_ecat_msgs::InitState::ConstPtr& msg)
{
  init_state_buffer_.writeFromNonRT(*msg);
}

void RadioRemote::OdomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_buffer_.writeFromNonRT(*msg);
}

void RadioRemote::CameraCallback(const vision_msgs::BoundingBox2DArrayConstPtr& msg) 
{
  camera_buffer_.writeFromNonRT(*msg);
}

void RadioRemote::imuCallback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) 
{
  imu_buffer_.writeFromNonRT(*msg);
}

void RadioRemote::radarCmdCallback(const std_msgs::Float32::ConstPtr &msg)
{
  radar_cmd_buffer_.writeFromNonRT(*msg);
}

void RadioRemote::cameraCmdCallback(const std_msgs::Float32::ConstPtr &msg)
{
  camera_cmd_buffer_.writeFromNonRT(*msg);
  camera_init_ = true;
}




/**
 * @brief 发送GPS数据
 *
 * 根据CRSF协议组装GPS数据帧，并通过串口发送给飞控设备。
 *
 * @param latitude 卫星定位的纬度，单位为度，内部乘以1e7转换为定点数。
 * @param longitude 卫星定位的经度，单位为度，内部乘以1e7转换为定点数。
 * @param groundspeed 地面速度，单位由协议定义（通常为 m/s），占2个字节。
 * @param heading 航向角，单位为度，占2个字节，表示飞控朝向。
 * @param altitude 海拔高度，单位为米，在发送前会加上1000的偏移。
 * @param satellites 卫星数量，占1个字节，表示当前接收到的卫星信号数量。
 */
void RadioRemote::sendGps(double latitude, double longitude, uint16_t groundspeed,uint16_t heading, uint16_t altitude, uint8_t satellites)
{
  // 根据 CRSF 协议，GPS Payload 大小为 15 字节
  // (int32+int32+uint16+uint16+uint16+uint8 = 4+4+2+2+2+1)
  constexpr uint8_t PAYLOAD_SIZE = 15;
  // Frame Size 字段的值 = Payload Size + 2 (1 for Type, 1 for CRC)
  constexpr uint8_t FRAME_LENGTH_FIELD = PAYLOAD_SIZE + 2; // 15 + 2 = 17
  // 整个数据帧的总长度（用于DMA发送）
  constexpr uint8_t TOTAL_FRAME_LENGTH = FRAME_LENGTH_FIELD + 2; // 17 + 2 = 19

  // 1. 将浮点数和整数转换为协议要求的定点格式
  const int32_t lat_scaled = static_cast<int32_t>(latitude * 1e7);  // 协议单位: 度 * 10,000,000
  const int32_t lon_scaled = static_cast<int32_t>(longitude * 1e7); // 协议单位: 度 * 10,000,000
  const uint16_t alt_scaled = altitude + 1000;                      // 协议单位: 米，带 1000米 偏移

  // 2. 直接在 DMA 缓冲区中组装数据帧
  tx_buffer_[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER; // [Byte 0] 目标设备地址
  tx_buffer_[1] = FRAME_LENGTH_FIELD;             // [Byte 1] 帧长度 (类型+载荷+CRC) = 17
  tx_buffer_[2] = CRSF_FRAMETYPE_GPS;             // [Byte 2] 帧类型 (0x02)

  // --- Payload (15 bytes) ---
  // 同样，所有多字节数据都使用大端字节序 (Big-Endian)
  tx_buffer_[3] = (lat_scaled >> 24) & 0xFF; // [Byte 3] Latitude (MSB)
  tx_buffer_[4] = (lat_scaled >> 16) & 0xFF; // [Byte 4]
  tx_buffer_[5] = (lat_scaled >> 8) & 0xFF;  // [Byte 5]
  tx_buffer_[6] = lat_scaled & 0xFF;         // [Byte 6] Latitude (LSB)

  tx_buffer_[7] = (lon_scaled >> 24) & 0xFF; // [Byte 7] Longitude (MSB)
  tx_buffer_[8] = (lon_scaled >> 16) & 0xFF; // [Byte 8]
  tx_buffer_[9] = (lon_scaled >> 8) & 0xFF;  // [Byte 9]
  tx_buffer_[10] = lon_scaled & 0xFF;        // [Byte 10] Longitude (LSB)

  tx_buffer_[11] = (groundspeed >> 8) & 0xFF; // [Byte 11] Groundspeed (MSB)
  tx_buffer_[12] = groundspeed & 0xFF;        // [Byte 12] Groundspeed (LSB)

  tx_buffer_[13] = (heading >> 8) & 0xFF; // [Byte 13] Heading (MSB)
  tx_buffer_[14] = heading & 0xFF;        // [Byte 14] Heading (LSB)

  tx_buffer_[15] = (alt_scaled >> 8) & 0xFF; // [Byte 15] Altitude (MSB)
  tx_buffer_[16] = alt_scaled & 0xFF;        // [Byte 16] Altitude (LSB)

  tx_buffer_[17] = satellites; // [Byte 17] Satellites count

  // 3. 计算 CRC 校验码
  //    计算范围：从“帧类型”到 Payload 末尾
  const uint8_t crc = crc_.calc(&tx_buffer_[2], 1 + PAYLOAD_SIZE);
  tx_buffer_[18] = crc; // [Byte 18] CRC

  // 4. 通过 DMA 发送整个数据帧

  ssize_t bytesSent = write(fd, tx_buffer_, TOTAL_FRAME_LENGTH);
  if (bytesSent == -1) {
     ROS_INFO("write error");
     throw std::runtime_error("write GPS error");
  }
}

/**
 * @brief 发送姿态数据
 *
 * 根据 CRSF 协议，将姿态数据（俯仰角、横滚角和偏航角）组装成数据帧，并通过串口发送给飞控设备。
 *
 * 姿态数据以浮点数格式传入，函数内部会将其转换成定点数格式（乘以 10000），转换为 16 位有符号整数，
 * 然后使用大端字节序填入数据帧中。
 *
 * @param pitch  俯仰角，单位为度；内部乘以 10000 转换为定点数。
 * @param roll   横滚角，单位为度；内部乘以 10000 转换为定点数。
 * @param yaw    偏航角，单位为度；内部乘以 10000 转换为定点数。
 */
void RadioRemote::sendAttitude(float pitch, float roll, float yaw)
{
    // 根据 CRSF 协议，Payload 大小为 6 字节 (3 * int16_t)
    constexpr uint8_t PAYLOAD_SIZE = 6;
    // Frame Size 字段的值 = Payload Size + 2 (1 for Type, 1 for CRC)
    constexpr uint8_t FRAME_LENGTH_FIELD = PAYLOAD_SIZE + 2;
    // 整个数据帧的总长度（用于DMA发送）
    constexpr uint8_t TOTAL_FRAME_LENGTH = FRAME_LENGTH_FIELD + 2; // +2 for Address and Frame Size fields

    // 1. 将浮点数转换为协议要求的定点整数
    const int16_t pitch_scaled = static_cast<int16_t>(pitch * 10000.0f);
    const int16_t roll_scaled = static_cast<int16_t>(roll * 10000.0f);
    const int16_t yaw_scaled = static_cast<int16_t>(yaw * 10000.0f);

    // 2. 直接在 DMA 缓冲区中组装数据帧
    //    这里的索引和内容严格按照 CRSF 协议
    tx_buffer_[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER; // [Byte 0] 目标设备地址
    tx_buffer_[1] = FRAME_LENGTH_FIELD;             // [Byte 1] 帧长度 (类型+载荷+CRC) = 8
    tx_buffer_[2] = CRSF_FRAMETYPE_ATTITUDE;        // [Byte 2] 帧类型 (0x1E)

    // --- Payload (6 bytes) ---
    // 注意：CRSF 协议要求多字节数据为大端字节序 (Big-Endian)
    // STM32 是小端 (Little-Endian)，所以必须手动转换。
    tx_buffer_[3] = (pitch_scaled >> 8) & 0xFF; // [Byte 3] Pitch (MSB)
    tx_buffer_[4] = pitch_scaled & 0xFF;        // [Byte 4] Pitch (LSB)

    tx_buffer_[5] = (roll_scaled >> 8) & 0xFF; // [Byte 5] Roll (MSB)
    tx_buffer_[6] = roll_scaled & 0xFF;        // [Byte 6] Roll (LSB)

    tx_buffer_[7] = (yaw_scaled >> 8) & 0xFF; // [Byte 7] Yaw (MSB)
    tx_buffer_[8] = yaw_scaled & 0xFF;        // [Byte 8] Yaw (LSB)

    // 3. 计算 CRC 校验码
    //    CRC 的计算范围是从“帧类型”到 Payload 的末尾
    const uint8_t crc = crc_.calc(&tx_buffer_[2], 1 + PAYLOAD_SIZE);
    tx_buffer_[9] = crc; // [Byte 9] CRC

    // 4. 通过 DMA 发送整个数据帧
    ssize_t bytesSent = write(fd, tx_buffer_, TOTAL_FRAME_LENGTH);
    if (bytesSent == -1) {
       ROS_INFO("write error");
      throw std::runtime_error("write attitude error");
    }
}
 
 }  // namespace xbox_hw