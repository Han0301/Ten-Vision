/**
 * @file RcEcatHardwareInterface.cpp
 * @author Keten (2863861004@qq.com)
 * @brief
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "rc_ecat_master/RcEcatHardwareInterface.h"
#include <thread>
namespace rc_ecat_master {

// 提供ethercat时间戳转 ros时间戳的内联函数
ros::Time createRosTime(
    const std::chrono::time_point<std::chrono::high_resolution_clock>&
        timePoint) {
  return {
      static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::seconds>(
                                timePoint.time_since_epoch())
                                .count()),
      static_cast<uint32_t>(
          std::chrono::duration_cast<std::chrono::nanoseconds>(
              timePoint.time_since_epoch())
              .count() %
          1000000000)};
}

bool RcEcatHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  /* 先是parse一波 */
  XmlRpc::XmlRpcValue xml_rpc_value;
  std::string pkg_path = ros::package::getPath("rc_ecat_master");
  if (!root_nh.getParam("ConfigPath", xml_rpc_value)) {
    ROS_ERROR("Failed to get param: ConfigPath");
    return false;
  }
  if (xml_rpc_value.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      xml_rpc_value.size() < 1) {
    ROS_ERROR("Param ConfigPath:: is not a non-empty array!");
    return false;
  }
  // 取第一个元素里面的 ActuatorsConfig 字段
  std::string actuatorsConfig, ecatConfig, ecatHWConfig;
  if (!xml_rpc_value[0].hasMember("ActuatorsConfig")) {
    ROS_ERROR("ActuatorsConfig not found in ConfigPath::");
    return false;
  } else {
    actuatorsConfig =
        static_cast<std::string>(xml_rpc_value[0]["ActuatorsConfig"]);
    ROS_INFO("ActuatorsConfig path: %s", actuatorsConfig.c_str());
  }
  if (xml_rpc_value[1].hasMember("EcatConfig")) {
    ecatConfig = static_cast<std::string>(xml_rpc_value[1]["EcatConfig"]);
    ROS_INFO("EcatConfig path: %s", ecatConfig.c_str());
  } else {
    ROS_ERROR("EcatConfig not found in second element of ConfigPath");
    return false;
  }
  if (xml_rpc_value[2].hasMember("EcatSlaveHWConfig")) {
    ecatHWConfig =
        static_cast<std::string>(xml_rpc_value[2]["EcatSlaveHWConfig"]);
    ROS_INFO("EcatSlaveHWConfig path: %s", ecatHWConfig.c_str());
  } else {
    ROS_ERROR("EcatSlaveHWConfig not found in third element of ConfigPath");
    return false;
  }

  // 如果配置了ops9，那就解析ops9，并且配置ops9
  if (root_nh.getParam("Ops9", xml_rpc_value)) {
    if (xml_rpc_value.getType() == XmlRpc::XmlRpcValue::TypeArray &&
        xml_rpc_value.size() > 0) {
      if (!parseOps9Config(xml_rpc_value)) {
        ROS_ERROR("parseOps9Config failed");
        return false;
      }
    }
    if (setupOps9(root_nh) == false) {
      ROS_ERROR("setupOps9 failed");
      return false;
    }
    ops9_init_ = true;
  } else
    ROS_WARN("Ops9 is not a non-empty array!");
  /* 注意：由于其他硬件都是从从站进行读取，所以其他硬件的初始化要晚于ecat初始化
   */

  /* ecat relative */
  ecatConfig = pkg_path + "/config/" + ecatConfig;
  if (!parseEcatConfig(ecatConfig)) {
    ROS_ERROR("parseEcatConfig failed");
    return false;
  }
  // start the ecat slave and bus
  if (!setupEcat()) {
    ROS_ERROR("setupEcat failed");
    return false;
  }
  ecatHWConfig = pkg_path + "/config/" + ecatHWConfig;
  if (!parseEcatSlaveHWConfig(ecatHWConfig)) {
    ROS_ERROR("parseEcatSlaveHWConfig failed");
    return false;
  }

  actuatorsConfig = pkg_path + "/config/" + actuatorsConfig;
  ROS_INFO("ActuatorsConfig path: %s", actuatorsConfig.c_str());
  // 调用 parseActuatorConfig()，传入 actuatorsConfig
  if (!parseActuatorConfig(root_nh, actuatorsConfig)) {
    ROS_ERROR("parseActuatorConfig failed");
    return false;
  }

  /* 然后是hw配置 */
  if (!root_nh.getParam("Actuators", xml_rpc_value)) {
    ROS_ERROR("Actuators not found in hw.yaml");
    return false;
  }
  if (xml_rpc_value.getType() != XmlRpc::XmlRpcValue::TypeArray ||
      xml_rpc_value.size() < 1) {
    ROS_ERROR("Actuators is not a non-empty array!");
    return false;
  }
  parseActuators(xml_rpc_value);

  /* setup the gpio hw */

  // 如果配置了gpio，那就解析gpio,并且配置gpio
  if (root_nh.getParam("Gpio", xml_rpc_value)) {
    if (xml_rpc_value.getType() == XmlRpc::XmlRpcValue::TypeArray &&
        xml_rpc_value.size() > 0) {
      if (!parseGpioConfig(xml_rpc_value)) {
        ROS_ERROR("parseGpioConfig failed");
        return false;
      }
    }
    if (setupGpio() == false) {
      ROS_ERROR("setupGpio failed");
      return false;
    }
  } else
    ROS_WARN("Gpio is not a non-empty array!");

  /* setup the serial_to_32 hw */
  if (root_nh.getParam("Serial_To_32", xml_rpc_value)) {
    // 如果配置了serial_to_32，那就解析serial_to_32,并且配置serial_to_32
    if (!parseSerialTo32Config(xml_rpc_value)) {
      ROS_ERROR("parseSerialTo32Config failed");
      return false;
    }
    ros::NodeHandle serial_nh("upper_board");
    if (setupSerialTo32(serial_nh) == false) {
      ROS_ERROR("setupSerialTo32 failed");
      return false;
    }
    if_setup_serial_to_32_ = true;
  } else {
    if_setup_serial_to_32_ = false;
  }

  /* setup the handle */
  if (setupHandle() == false) {
    ROS_ERROR("setupHandle failed");
    return false;
  }

  /* setup the transmission */
  if (setupTransmission() == false) {
    ROS_ERROR("setupTransmission failed");
    return false;
  }

  /* setup the jointlimits */
  if (setupJointLimits(root_nh) == false) {
    ROS_ERROR("setupJointLimits failed");
    return false;
  }
  /* start the controller manager and the loop */
  controller_manager_ =
      std::make_shared<controller_manager::ControllerManager>(this, root_nh);

  // setup the update worker
  update_worker_ = std::make_shared<any_worker::Worker>(
      "updateWorker", 0.001,
      std::bind(&RcEcatHW::updateWorkerCb, this, std::placeholders::_1));
  publish_worker_ = std::make_shared<any_worker::Worker>(
      "publishWorker", 0.005,
      std::bind(&RcEcatHW::updatePublishCb, this, std::placeholders::_1));
  // setup the publisher
  ecatmsgPublisher_ = std::make_shared<
      any_node::ThreadedPublisher<ros_ecat_msgs::DJIEcatRosMsg>>(
      root_nh.advertise<ros_ecat_msgs::DJIEcatRosMsg>("ecat_msg", 10), 50,
      true);
  ecatcommandsPublisher_ = std::make_shared<
      any_node::ThreadedPublisher<ros_ecat_msgs::DJIEcatRosCommands>>(
      root_nh.advertise<ros_ecat_msgs::DJIEcatRosCommands>("ecat_cmd", 10), 50,
      true);
  vescmsgPublisher_ = std::make_shared<
      any_node::ThreadedPublisher<ros_ecat_msgs::VescEcatRosMsg>>(
      root_nh.advertise<ros_ecat_msgs::VescEcatRosMsg>("vesc_msg", 10), 50,
      true);
  vesccommandsPublisher_ = std::make_shared<
      any_node::ThreadedPublisher<ros_ecat_msgs::VescEcatRosCommands>>(
      root_nh.advertise<ros_ecat_msgs::VescEcatRosCommands>("vesc_cmds", 10),
      50, true);

  signal_handler::SignalHandler::bindAll(&RcEcatHW::handleSignal, this);

  update_worker_->start(95);
  publish_worker_->start(45);
  ecatCheckWorker_->start(90);
  if (ops9_init_) ops9UpdateWorker_->start(90);
  return true;
}

void RcEcatHW::read(const ros::Time& time, const ros::Duration& period) {
  /* read from ecat bus */

  ecat_bus_->updateRead();
  analysisActuatorData();
  /* get the time-stamp */
  act_to_jnt_state_->propagate();  // 将执行器状态转化为关节状态
  // ROS_WARN_STREAM("something wrong before here");

  // update the gpio data
  gpio_manager_->readGpioData();
}

void RcEcatHW::write(const ros::Time& time, const ros::Duration& period) {
  /* converse */
  jnt_to_act_effort_->propagate();  // 将关节力矩转化为执行器力矩

  // for (auto effort_joint_handle : joint_handles_) {
  //   ROS_INFO_STREAM_THROTTLE(0.5,
  //                            "joint cmd :" <<
  //                            effort_joint_handle.getCommand());
  // }
  // save the commanded effort before enforceLimits
  saveActuatorCommands();

  // enforceLimits will limit cmd_effort into suitable value
  // effort_joint_saturation_interface_.enforceLimits(
  //     period);  // enforceLimits会限制cmd_effort到合适的值
  for (auto& it : actuator_data_map_) {
    for (auto& it2 : it.second) {
      ROS_INFO_STREAM_THROTTLE(0.5, "actuator cmd :" << it2.second.eff_exe_cmd);
    }
  }

  // converse again 关节指令到执行器指令（做减速比换算）
  jnt_to_act_effort_->propagate();
  // setup the commands and save the raw commmands
  setActuatorCommands();

  // setup the gpio commands
  gpio_manager_->writeGpioData();
  ecat_bus_->updateWrite();  // 更新写入

  // write to the ops9
  if (ops9_init_) ops9_manager_.writeOps9(time, period);

  // write to the serial_to_32
  if (if_setup_serial_to_32_) {
    serial_to_32_manager_->write();
  }
}

bool RcEcatHW::parseEcatSlaveHWConfig(const std::string& yaml_path) {
  if (!std::filesystem::exists(yaml_path)) {
    throw std::runtime_error("File not found: " + yaml_path);
  }
  YAML::Node config = YAML::LoadFile(yaml_path);
  /* check the root */
  if (config["gpio"]) {
    for (const auto& node : config["gpio"]) {
      std::string name = node["name"].as<std::string>();
      input_gpio_num_ = node["num"].as<int>();
      ROS_INFO_STREAM("GPIO: " << name << ", num: " << input_gpio_num_);
    }
  } else {
    ROS_ERROR("EcatSlaveHWConfig: gpio configuration not found!");
    return false;
  }
  if (config["can"]) {
    if (config["can"]["num"]) {
      has_can_num_ = config["can"]["num"].as<int>();
      ROS_INFO_STREAM("CAN: num: " << has_can_num_);
    }
  } else {
    ROS_ERROR("EcatSlaveHWConfig: can configuration not found!");
    return false;
  }
  return true;
}

bool RcEcatHW::parseActuatorConfig(ros::NodeHandle& root_nh,
                                   const std::string& yaml_path) {
  if (!std::filesystem::exists(yaml_path))
    throw std::runtime_error("File not found: " + yaml_path);
  YAML::Node config = YAML::LoadFile(yaml_path);
  /* check the root */
  if (!config["actuator_coefficients"])
    throw YAML::ParserException(config.Mark(),
                                "the root: [actuator_coefficients] not found");
  actuator_coefficients_map_.clear();
  const auto masterNode = config["actuator_coefficients"];

  /* for all the sub */
  for (const auto& node : masterNode) {
    rc_ecat_master::ActuatorCoefficients coeff;
    coeff.if_vesc = node["if_vesc"].as<bool>();
    coeff.act2pos = node["act2pos"].as<double>();
    coeff.act2vel = node["act2vel"].as<double>();
    coeff.act2effort = node["act2effort"].as<double>();
    coeff.effort2act = node["effort2act"].as<double>();
    coeff.max_out = node["max_out"].as<double>();
    actuator_coefficients_map_[node["name"].as<std::string>()] = coeff;
  }

  for (const auto& pair : actuator_coefficients_map_) {
    const std::string& type = pair.first;
    const auto& coeff = pair.second;
    ROS_INFO_STREAM("Coeff for type [" << type << "]: if_vesc=" << coeff.if_vesc
                                       << ", act2pos=" << coeff.act2pos
                                       << ", act2vel=" << coeff.act2vel
                                       << ", act2effort=" << coeff.act2effort
                                       << ", effort2act=" << coeff.effort2act
                                       << ", max_out=" << coeff.max_out);
  }
  // ROS_INFO_STREAM("");
  return true;
}

bool RcEcatHW::parseActuators(const XmlRpc::XmlRpcValue& xml_rpc_value) {
  // get the sum of the actuators that will be loaded
  int numActuators = xml_rpc_value.size();
  ROS_INFO_STREAM("Found [" << numActuators << "] actuators");

  actuator_data_map_.clear();
  for (size_t i = 0; i < numActuators; i++) {
    int id;
    std::string name, type, canbusStr;
    double reducer;
    try {
      name = static_cast<std::string>(xml_rpc_value[i]["name"]);
      type = static_cast<std::string>(xml_rpc_value[i]["type"]);
      id = static_cast<int>(xml_rpc_value[i]["id"]);
      reducer = static_cast<double>(xml_rpc_value[i]["reducer"]);
      canbusStr = static_cast<std::string>(xml_rpc_value[i]["canbus"]);
    } catch (const XmlRpc::XmlRpcException& exc) {
      ROS_ERROR("Error parsing actuator[%d]: %s", i, exc.getMessage().c_str());
      return false;
    }
    // 将 canbus 字符串转换为枚举，并记录具体是哪一条CAN总线
    RcEcatSlave::CanBus canbus =
        (canbusStr == "CAN0" ? RcEcatSlave::CanBus::CAN0
                             : RcEcatSlave::CanBus::CAN1);
    std::string canbusEnumStr =
        (canbus == RcEcatSlave::CanBus::CAN0 ? "CAN0" : "CAN1");

    ROS_INFO_STREAM("Actuator[" << i << "]: name=" << name << ", type=" << type
                                << ", id=" << id << ", reducer=" << reducer
                                << ", canbus_str=" << canbusStr
                                << ", assigned CANBUS=" << canbusEnumStr);
    // 如果该类型还未创建，则先创建对应的 map
    if (actuator_data_map_.find(type) == actuator_data_map_.end()) {
      actuator_data_map_[type] =
          std::unordered_map<std::string, rc_ecat_master::ActuatorData>();
    }
    rc_ecat_master::ActuatorData new_actuator_data;
    new_actuator_data.name = name;
    new_actuator_data.type = type;
    new_actuator_data.id = id;
    new_actuator_data.reducer = reducer;

    new_actuator_data.canbus =
        (canbusStr == "CAN0" ? RcEcatSlave::CanBus::CAN0
                             : RcEcatSlave::CanBus::CAN1);
    // 直接插入默认构造的 ActuatorData
    actuator_data_map_[type][name] = new_actuator_data;
  }
  return true;
}

bool RcEcatHW::setupHandle() { /* register the handle */
  // 在这里根据关节数量注册关节句柄
  for (auto& type_pair : actuator_data_map_) {
    const std::string& type = type_pair.first;
    auto& actuator_map = type_pair.second;
    for (auto& actuator_pair : actuator_map) {
      const std::string& actuator_name = actuator_pair.first;
      rc_ecat_master::ActuatorData& actuator_data = actuator_pair.second;

      /* set the joint handle and joint state interface */
      hardware_interface::JointStateHandle joint_state_handle(
          actuator_name, &actuator_data.pos, &actuator_data.vel,
          &actuator_data.effort);
      joint_state_interface_.registerHandle(joint_state_handle);

      /* set the effort handle and effort interface */
      hardware_interface::JointHandle effort_joint_handle(
          joint_state_interface_.getHandle(actuator_name),
          &actuator_data.eff_exe_cmd);  // 注意将实际输出命令传入进行构造
      effort_joint_interface_.registerHandle(effort_joint_handle);
      joint_handles_.push_back(effort_joint_handle);
      ROS_INFO_STREAM("jointhandle: " << actuator_name << " is registered to "
                                      << "effort_joint_interface");

      /* set the actuator handle and actuator interface */
      // set state handle and state interface
      hardware_interface::ActuatorStateHandle actuator_state_handle(
          actuator_name, &actuator_data.act_pos, &actuator_data.act_vel,
          &actuator_data.act_effort);
      act_state_interface_.registerHandle(actuator_state_handle);
      // set effort handle and effort interface
      hardware_interface::ActuatorHandle effort_actuator_handle(
          act_state_interface_.getHandle(actuator_name),
          &actuator_data.act_eff_cmd);
      effort_actuator_interface_.registerHandle(effort_actuator_handle);
    }
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&effort_joint_interface_);
  registerInterface(&act_state_interface_);
  registerInterface(&effort_actuator_interface_);
  return true;
}

bool RcEcatHW::setupJointLimits(ros::NodeHandle& root_nh) {
  XmlRpc::XmlRpcValue joint_limits_settings;
  if (!root_nh.getParam("joint_limits", joint_limits_settings)) {
    ROS_WARN(
        "Parameter [joint_limits] not set. Using default limits for all "
        "joints.");
  }
  // 遍历所有电机数据（一般每个电机对应一个关节）
  for (auto& type_pair : actuator_data_map_) {
    auto& actuator_map = type_pair.second;
    for (auto& actuator_pair : actuator_map) {
      const std::string& joint_name = actuator_pair.first;
      double min_position, max_position, max_velocity, max_acceleration,
          max_effort;
      bool has_position_limits, has_velocity_limits, has_acceleration_limits,
          has_effort_limits, has_jerk_limits, angle_wraparound;
      // 如果 joint_limits 参数中包含该关节，就用参数配置；否则使用默认值
      if (joint_limits_settings.getType() == XmlRpc::XmlRpcValue::TypeStruct &&
          joint_limits_settings.hasMember(joint_name)) {
        XmlRpc::XmlRpcValue limits = joint_limits_settings[joint_name];
        // 提取具体的限制参数
        if (static_cast<bool>(limits["has_position_limits"]) == true) {
          has_position_limits = true;
          min_position = static_cast<double>(limits["min_position"]);
          max_position = static_cast<double>(limits["max_position"]);
        } else {
          has_position_limits = false;
        }
        if (static_cast<bool>(limits["has_velocity_limits"]) == true) {
          max_velocity = static_cast<double>(limits["max_velocity"]);
          has_velocity_limits = true;
        } else {
          has_velocity_limits = false;
        }
        if (static_cast<bool>(limits["has_acceleration_limits"]) == true) {
          max_acceleration = static_cast<double>(limits["max_acceleration"]);
          has_acceleration_limits = true;
        } else {
          has_acceleration_limits = false;
        }
        if (static_cast<bool>(limits["has_effort_limits"]) == true) {
          max_effort = static_cast<double>(limits["max_effort"]);
          has_effort_limits = true;
        } else {
          has_effort_limits = false;
        }
        if (static_cast<bool>(limits["has_jerk_limits"]) == true) {
          has_jerk_limits = static_cast<bool>(limits["has_jerk_limits"]);
        } else {
          has_jerk_limits = false;
        }
        if (static_cast<bool>(limits["angle_wraparound"]) == true) {
          angle_wraparound = true;
        } else {
          angle_wraparound = false;
        }
      } else {
        ROS_WARN("Joint [%s] has no specified limits. Using default limits.",
                 joint_name.c_str());
        // 定义默认参数，例如：
        min_position = -1e9;
        max_position = 1e9;
        max_velocity = 50.0;  // 根据实际情况调整
        max_acceleration = 1e9;
        max_effort = 50.0;
      }
      // then prepare the joint limits
      try {
        // register joint limits interface
        joint_limits_interface::JointLimits limit;
        limit.min_position = min_position;
        limit.max_position = max_position;
        limit.max_velocity = max_velocity;
        limit.max_acceleration = max_acceleration;
        limit.max_effort = max_effort;
        limit.has_position_limits = has_position_limits;
        limit.has_velocity_limits = has_velocity_limits;
        limit.has_acceleration_limits = has_acceleration_limits;
        limit.has_effort_limits = has_effort_limits;
        limit.angle_wraparound = angle_wraparound;  // 根据实际情况调整
        /* TODO: can add other limits like soft limits and saturation limits
         */

        hardware_interface::JointHandle joint_handle =
            effort_joint_interface_.getHandle(joint_name);
        joint_limits_interface::EffortJointSaturationHandle eff_sat_handle(
            joint_handle, limit);
        // effort_joint_soft_limits_interface_.registerHandle(soft_handle);
        effort_joint_saturation_interface_.registerHandle(eff_sat_handle);
      } catch (const std::exception& ex) {
        ROS_ERROR("Failed to register limits for joint [%s]: %s",
                  joint_name.c_str(), ex.what());
      }
    }
  }

  // 如有需要，此处还可以注册或更新 joint_limits_interface
  return true;
}

bool RcEcatHW::setupTransmission() {
  // 清空之前的 transmission
  transmission_reducer_.clear();
  act_to_jnt_state_ =
      std::make_unique<transmission_interface::ActuatorToJointStateInterface>();
  jnt_to_act_effort_ = std::make_unique<
      transmission_interface::JointToActuatorEffortInterface>();
  transmission_group_data_.clear();
  size_t total_actuators = 0;
  for (auto& type_pair : actuator_data_map_) {
    total_actuators += type_pair.second.size();
  }
  transmission_reducer_.reserve(total_actuators);

  // 对于每个电机都创建一组 transmission
  for (auto& type_pair : actuator_data_map_) {
    // type_pair.first 是具体的类型，如 "m3508"
    for (auto& actuator_pair : type_pair.second) {
      // actuator_pair.first 是电机名称
      rc_ecat_master::ActuatorData& actuator_data = actuator_pair.second;

      TransmissionGroupData tgroup;
      // 每个 transmission 仅包含单个电机数据，依次把各指针放入 vector
      tgroup.act_data.position.push_back(&actuator_data.act_pos);
      tgroup.jnt_data.position.push_back(&actuator_data.pos);
      tgroup.act_data.velocity.push_back(&actuator_data.act_vel);
      tgroup.jnt_data.velocity.push_back(&actuator_data.vel);
      tgroup.act_data.effort.push_back(&actuator_data.act_effort);
      tgroup.jnt_data.effort.push_back(&actuator_data.effort);

      tgroup.act_cmd.effort.push_back(&actuator_data.act_eff_cmd);
      tgroup.jnt_cmd.effort.push_back(&actuator_data.eff_exe_cmd);
      tgroup.act_cmd.velocity.push_back(&actuator_data.act_vel_cmd);
      tgroup.jnt_cmd.velocity.push_back(&actuator_data.vel_exe_cmd);
      tgroup.act_cmd.position.push_back(&actuator_data.act_pos_cmd);
      tgroup.jnt_cmd.position.push_back(&actuator_data.pos_exe_cmd);

      // 使用该电机的 reducer 创建 transmission
      double reducer = actuator_data.reducer;
      transmission_interface::SimpleTransmission transmission(reducer);
      transmission_reducer_.push_back(transmission);
      tgroup.trans_ptr = &transmission_reducer_.back();

      // 以电机名称组合句柄名称注册
      std::string handle_name = actuator_pair.first;
      act_to_jnt_state_->registerHandle(
          transmission_interface::ActuatorToJointStateHandle(
              handle_name + "_act_to_jnt", tgroup.trans_ptr, tgroup.act_data,
              tgroup.jnt_data));
      jnt_to_act_effort_->registerHandle(
          transmission_interface::JointToActuatorEffortHandle(
              handle_name + "_jnt_to_act_effort", tgroup.trans_ptr,
              tgroup.act_cmd, tgroup.jnt_cmd));
      transmission_group_data_.push_back(tgroup);
    }
  }
  // 注册传输接口
  registerInterface(act_to_jnt_state_.get());
  registerInterface(jnt_to_act_effort_.get());
  return true;
}

bool RcEcatHW::parseEcatConfig(const std::string& yaml_path) {
  if (!std::filesystem::exists(yaml_path)) {
    ROS_ERROR_STREAM("EcatConfig file not found: " << yaml_path);
    return false;
  }
  YAML::Node config = YAML::LoadFile(yaml_path);
  if (!config["ecat_config"]) {
    ROS_ERROR_STREAM("ecat_config node not found in " << yaml_path);
    return false;
  }
  YAML::Node ecatnode = config["ecat_config"];
  if (!ecatnode["master"] || !ecatnode["master"]["name"]) {
    ROS_ERROR_STREAM("ecat_config master config not found in " << yaml_path);
    return false;
  }
  if (!ecatnode["slave"] || !ecatnode["slave"]["name"]) {
    ROS_ERROR_STREAM("ecat_config slave config not found in " << yaml_path);
    return false;
  }

  slave_name_ = ecatnode["slave"]["name"].as<std::string>();
  slave_address_ = ecatnode["slave"]["address"].as<uint32_t>();
  bus_name_ = ecatnode["master"]["name"].as<std::string>();

  return true;
}


// add some retry times in the setupEcat
bool RcEcatHW::setupEcat() {
  int retry = 0;
  const int max_retries = 1;
  int reopen_times = 0;
  const int max_reopen_times = 10;
  bool operational = false;
  while (reopen_times < max_reopen_times) {
    retry = 0;
    while (retry < max_retries) {
      ecat_bus_ =
          std::make_unique<soem_interface_rsl::EthercatBusBase>(bus_name_);
      ecat_slave_ = std::make_shared<rc_ecat_master::RcEcatSlave>(
          slave_name_, ecat_bus_.get(), slave_address_);

      ecat_bus_->addSlave(ecat_slave_);
      ros::Duration(0.05).sleep();
      ecat_bus_->startup(true);
      ecat_bus_->setState(EC_STATE_PRE_OP);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      
      if(!ecat_bus_->waitForState(EC_STATE_PRE_OP)){
        ROS_ERROR_STREAM("");
        return false;
      }
      ecat_bus_->setState(EC_STATE_SAFE_OP);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      if(!ecat_bus_->waitForState(EC_STATE_SAFE_OP)){
        return false;
      }

      ecat_bus_->setState(EC_STATE_OPERATIONAL);
      // 强制发送写操作，使从站有机会接收到有效的过程数据
      ecat_bus_->updateRead();
      ecat_bus_->updateWrite();
      if (ecat_bus_->waitForState(EC_STATE_OPERATIONAL, slave_address_)) {
        operational = true;
        break;
      }
      ROS_WARN_STREAM("Slave state is not OPERATIONAL, retrying... ("
                      << retry + 1 << "/" << max_retries << ")");
      ros::Duration(0.05).sleep();  // 等待0.5秒后重试
      retry++;
    }

    if (operational) {
      inOP_ = true;
      // 设置掉线检测线程
      ecatCheckWorker_ = std::make_shared<any_worker::Worker>(
          "ecatCheckWorker", 0.1,
          std::bind(&RcEcatHW::ecatCheckWorkerCb, this, std::placeholders::_1));

      // put the bus ptr to the slaves   
      ecat_slave_->setEthercatBusBasePointer(ecat_bus_.get());
      return true;
    } else {
      ROS_ERROR_STREAM("Failed to set slave state OPERATONAL after "
                       << max_retries
                       << " retries. Restarting EtherCAT bus...");
      // 实际动作：关闭当前总线和从站，然后重新启动
      ecat_bus_->shutdown();
      ecat_slave_->shutdown();
      retry = 0;  // 重置重试计数器
      reopen_times++;
      ros::Duration(0.05).sleep();  // 等待一段时间再重启
    }
  }
  ROS_ERROR_STREAM("Failed to set slave state OPERATONAL after "
                   << max_reopen_times
                   << " reopen attempts. Please check the EtherCAT setup.");
  return false;
}


void RcEcatHW::analysisActuatorData() {
  if (!ecat_slave_) {
    ROS_ERROR("ecat_slave_ is null");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(actuator_data_map_lock_);
    for (auto& it : actuator_data_map_) {
      const std::string& actuatorType = it.first;
      auto& actuator = it.second;
      for (auto& itit : actuator) {
        const std::string& actuatorName = itit.first;
        auto& actuator_data = itit.second;
        // according to the coefficients map, convert the data
        // 注意这里是做原始数据向执行器的转换（理解执行器 和
        // 关节，对于3508来说，执行器就是转子端，关节就是加了减速箱之后的轴）
        auto coeff = actuator_coefficients_map_[actuatorType];
        if (coeff.if_vesc == false) {
          // call the RcEcatSlave function to get the data
          actuator_data.vel_raw = ecat_slave_->getActuatorVelocityRaw(
              actuator_data.canbus, actuator_data.id);
          // actuator_data.effort_raw = ecat_slave_->getActuatorEffortRaw(
          //     actuator_data.canbus, actuator_data.id);
          actuator_data.effort_raw = 0.0f;
          actuator_data.temp = ecat_slave_->getActuatorTemperatureRaw(
              actuator_data.canbus, actuator_data.id);
          actuator_data.temp = 25;
          actuator_data.act_vel = coeff.act2vel * actuator_data.vel_raw;
          actuator_data.act_effort =
              coeff.act2effort *
              actuator_data.effort_raw;  // 注意转换得到的是act_
          unsigned char bytes[4];
          bytes[1] = static_cast<uint8_t>(
              ecat_slave_->getActuatorPositionRaw(actuator_data.canbus,
                                                  actuator_data.id) >>
              8);
          bytes[0] =
              static_cast<uint8_t>(ecat_slave_->getActuatorPositionRaw(
                                       actuator_data.canbus, actuator_data.id) &
                                   0xFF);
          bytes[3] = static_cast<uint8_t>(
              ecat_slave_->getActuatorEffortRaw(actuator_data.canbus,
                                                actuator_data.id) >>
              8);
          bytes[2] =
              static_cast<uint8_t>(ecat_slave_->getActuatorEffortRaw(
                                       actuator_data.canbus, actuator_data.id) &
                                   0xFF);
          float f_val;
          memcpy(&f_val, bytes, sizeof(float));
          actuator_data.act_pos = f_val;

          // if (actuator_data.pos_raw - actuator_data.last_pos_raw > 4096) {
          //   actuator_data.actualCircle--;
          // } else if (actuator_data.pos_raw - actuator_data.last_pos_raw <
          //            -4096) {
          //   actuator_data.actualCircle++;
          // }
          // if (actuator_data.firstReceived == true) {
          //   actuator_data.firstReceived = false;
          //   actuator_data.actualCircle = 0;
          //   actuator_data.last_pos_raw = 0;
          // }
          // actuator_data.act_pos =
          // actuator_data.act_pos =
          //     static_cast<double>(actuator_data.actualCircle) * 2. * M_PI +
          //     actuator_data.pos_raw * coeff.act2pos;
          // actuator_data.last_pos_raw = actuator_data.pos_raw;
        } else {
          // 使用 VESC 驱动器 使用特殊的获取电机数据的方法
          actuator_data.vel_raw = ecat_slave_->getVescActuatorVelocityRaw(
              actuator_data.canbus, actuator_data.id);
          actuator_data.effort_raw = ecat_slave_->getVescActuatorCurrentRaw(
              actuator_data.canbus, actuator_data.id);
          // ROS_INFO_STREAM("VESC [" << actuator_data.name << "]["
          //                          << static_cast<int>(actuator_data.id)
          //                          << "] before computed velocity: "
          //                          << actuator_data.vel_raw
          //                          << ", before computed effort: "
          //                          << actuator_data.effort_raw);
          /* vesc就只能算这些了，其他直接设成0了 */
          actuator_data.act_vel = coeff.act2vel * actuator_data.vel_raw;
          actuator_data.act_effort =
              coeff.act2effort * actuator_data.effort_raw;
          // ROS_INFO_STREAM("VESC ["
          //                 << actuator_data.name
          //                 << "] computed velocity: " << actuator_data.act_vel
          //                 << ", computed effort: " <<
          //                 actuator_data.act_effort);
        }
      }
    }
  }
}


void RcEcatHW::setActuatorCommands() {
  // 先打印 actuator_data_map_ 的详细信息，查看各执行器的 id 和 eff_exe_cmd
  // 等数据
  {
    std::lock_guard<std::mutex> lock(actuator_data_map_lock_);
    for (const auto& type_pair : actuator_data_map_) {
      const std::string& actuatorType = type_pair.first;
      const auto& actuator_map = type_pair.second;
      for (const auto& actuator_pair : actuator_map) {
        const std::string& actuatorName = actuator_pair.first;
        if (actuatorName == "drive_front_joint") {
          const auto& actuator_data = actuator_pair.second;
          // ROS_INFO_STREAM_THROTTLE(
          //     0.5, "Debug: Actuator ["
          //              << actuatorName << "] of type [" << actuatorType
          //              << "] has id: " << static_cast<int>(actuator_data.id)
          //              << ", canbus: "
          //              << (actuator_data.canbus == RcEcatSlave::CanBus::CAN0
          //                      ? "CAN0"
          //                      : "CAN1")
          //              << ", eff_exe_cmd: " << actuator_data.eff_exe_cmd);
        }
      }
    }
  }

  std::unordered_map<rc_ecat_master::RcEcatSlave::CanBus,
                     std::unordered_map<int, int16_t>>
      commands{};
  std::unordered_map<rc_ecat_master::RcEcatSlave::CanBus,
                     std::unordered_map<int, int32_t>>
      vesc_commands{};
  for (auto bus : {RcEcatSlave::CanBus::CAN0, RcEcatSlave::CanBus::CAN1}) {
    for (int i = 0; i < 8; i++) {
      commands[bus][i] = 0;
    }
  }
  for (auto bus : {RcEcatSlave::CanBus::CAN0, RcEcatSlave::CanBus::CAN1}) {
    for (int i = 0; i < 6; i++) {
      vesc_commands[bus][i] = 0;  // 初始化所有命令为0
    }
  }

  for (auto& it : actuator_data_map_) {
    const std::string& actuatorType = it.first;
    auto& actuator = it.second;
    for (auto& itit : actuator) {
      auto& actuator_data = itit.second;
      auto coeff = actuator_coefficients_map_[actuatorType];
      if (coeff.if_vesc == false) {
        double cmd =
            minAbs(actuator_data.eff_exe_cmd * coeff.effort2act, coeff.max_out);
        commands[actuator_data.canbus][actuator_data.id] =
            static_cast<int16_t>(cmd);
        actuator_data.actual_cmd = static_cast<int16_t>(cmd);
      } else {
        // 如果是 VESC 驱动器，使用特殊的命令设置
        double cmd =
            minAbs(actuator_data.eff_exe_cmd * coeff.effort2act, coeff.max_out);
        vesc_commands[actuator_data.canbus][actuator_data.id] =
            static_cast<int32_t>(cmd);
        actuator_data.actual_cmd_32 = static_cast<int32_t>(cmd);
      }
    }
  }
  // 添加打印 vesc 指令信息
  for (const auto& [bus, cmd_map] : vesc_commands) {
    std::string bus_str = (bus == RcEcatSlave::CanBus::CAN0) ? "CAN0" : "CAN1";
    for (const auto& [id, cmd_value] : cmd_map) {
      // ROS_INFO_STREAM_THROTTLE(0.5, "Set VESC command on "
      //                                   << bus_str << " for actuator id " <<
      //                                   id
      //                                   << " : " << cmd_value);
    }
  }
  // call the RcEcatSlave function to set the data
  ecat_slave_->setActuatorCommands(commands);
  ecat_slave_->setVescActuatorCommands(vesc_commands);
  // 将计算结果快速拷贝到成员变量 latest_commands_ 中（加锁保护）
  {
    std::lock_guard<std::mutex> lock(commandsMutex_);
    latest_commands_ = std::move(commands);
    latest_vesc_commands_ = std::move(vesc_commands);
  }
}

void RcEcatHW::saveActuatorCommands() {
  for (auto& it : actuator_data_map_) {
    const std::string& actuatorType = it.first;
    auto& actuator = it.second;
    for (auto& itit : actuator) {
      const std::string& actuatorName = itit.first;
      auto& actuator_data = itit.second;
      auto coeff = actuator_coefficients_map_[actuatorType];
      actuator_data.eff_cmd =
          minAbs(actuator_data.eff_exe_cmd * coeff.effort2act, coeff.max_out);
    }
  }
}

bool RcEcatHW::updateWorkerCb(const any_worker::WorkerEvent& event) {
  if (inOP_) {
    auto tp = createRosTime(ecat_bus_->getUpdateReadStamp());
    ros::Duration period(event.timeStep);
    read_stamp_ = tp;
    read(tp, period);
    controller_manager_->update(tp, period);
    tp = createRosTime(ecat_bus_->getUpdateWriteStamp());
    write_stamp_ = tp;
    write(tp, period);
  } else {
    ROS_WARN_STREAM_THROTTLE(0.5,
                             "ecat bus is not in OP, please check the state");
  }
  return true;
}

bool RcEcatHW::updatePublishCb(const any_worker::WorkerEvent& event) {
  if (inOP_) {
    ros_ecat_msgs::DJIEcatRosMsg msg{};
    ros_ecat_msgs::DJIEcatRosCommands cmd{};
    ros_ecat_msgs::VescEcatRosMsg vesc_msg{};
    ros_ecat_msgs::VescEcatRosCommands vesc_cmds{};
    // 预置各数组前8个元素为0.0（或0）
    msg.names.assign(8, "");
    msg.position.assign(8, 0.0);
    msg.velocity.assign(8, 0.0);
    msg.torque.assign(8, 0.0);
    msg.temperature.assign(8, 0.0);
    // 如果需要，也可以预置命令数组（这里假设命令为 int64 类型）
    msg.input_io.resize(input_gpio_num_);
    msg.output_io.resize(input_gpio_num_);
    cmd.output_io.resize(input_gpio_num_);
    cmd.can0_motor_commands.assign(8, 0);
    cmd.can1_motor_commands.assign(8, 0);

    vesc_msg.names.clear();
    vesc_msg.velocity.clear();
    vesc_msg.torque.clear();
    vesc_cmds.vesc_can0_motor_commands.clear();
    vesc_cmds.vesc_can1_motor_commands.clear();

    msg.stamp = getReadStamp();
    cmd.stamp = getWriteStamp();
    vesc_msg.stamp = getReadStamp();
    vesc_cmds.stamp = getWriteStamp();
    // 先拷贝最新的命令数据
    std::unordered_map<RcEcatSlave::CanBus, std::unordered_map<int, int16_t>>
        local_commands;
    std::unordered_map<RcEcatSlave::CanBus, std::unordered_map<int, int32_t>>
        local_vesc_commands;
    {
      std::lock_guard<std::mutex> lock(commandsMutex_);
      local_commands = latest_commands_;
      local_vesc_commands = latest_vesc_commands_;  // 获取 VESC 的命令数据
    }
    int vesc_index = 0;
    // 遍历 actuator_data_map_ 中所有电机数据
    {
      std::lock_guard<std::mutex> lock(actuator_data_map_lock_);
      for (auto& type_pair : actuator_data_map_) {
        bool if_vesc = actuator_coefficients_map_[type_pair.first].if_vesc;
        auto& actuator_map = type_pair.second;
        for (auto& actuator_pair : actuator_map) {
          // actuator_pair.first 为电机名称
          // actuator_pair.second 存放电机数据
          if (if_vesc == false) {
            // 1.pub the ecat msgs (TxPdo)
            msg.names[actuator_pair.second.id] = actuator_pair.first;

            // 存放关节/电机的状态数据
            msg.position[actuator_pair.second.id] = actuator_pair.second.pos;
            msg.velocity[actuator_pair.second.id] = actuator_pair.second.vel;
            msg.torque[actuator_pair.second.id] = actuator_pair.second.effort;
            msg.temperature[actuator_pair.second.id] =
                actuator_pair.second.temp;
          } else {
            // 先从数据结构中获取计算后的值，并打印调试日志
            double computed_vel = actuator_pair.second.vel;
            double computed_effort = actuator_pair.second.effort;
            // ROS_INFO_STREAM("DEBUG actuator ["
            //                 << actuator_pair.first << "] computed vel: "
            //                 << computed_vel << ", effort: " <<
            //                 computed_effort);
            vesc_msg.names.push_back(actuator_pair.first);
            vesc_msg.velocity.push_back(computed_vel);
            vesc_msg.torque.push_back(computed_effort);
          }
        }
      }
    }

    // 遍历gpio数据，写入msg
    std::unordered_map<int32_t, bool> gpio_data =
        gpio_manager_->getGpioData(ecat_slave_hw::GpioType::INPUT);
    for (const auto& it : gpio_data) {
      int id = it.first;
      bool value = it.second;
      if (id < 8) {
        msg.input_io[id] = value;
      } else {
        ROS_WARN_STREAM("Gpio id " << id << " is out of range");
      }
    }
    gpio_data = gpio_manager_->getGpioData(ecat_slave_hw::GpioType::OUTPUT);
    for (const auto& it : gpio_data) {
      int id = it.first;
      bool value = it.second;
      if (id < 8) {
        msg.output_io[id] = value;
      } else {
        ROS_WARN_STREAM("Gpio id " << id << " is out of range");
      }
    }

    for (const auto& bus_pair : local_commands) {
      for (const auto& id_cmd : bus_pair.second) {
        int id = id_cmd.first;
        int16_t cmd_val = id_cmd.second;
        if (bus_pair.first == RcEcatSlave::CanBus::CAN0) {
          // 如果预置数组容量不足，可以扩展，例如使用 push_back 或 resize
          // 后再赋值
          if (static_cast<size_t>(id) < cmd.can0_motor_commands.size())
            cmd.can0_motor_commands[id] = cmd_val;
          else
            cmd.can0_motor_commands.push_back(cmd_val);
        } else {  // CAN1
          if (static_cast<size_t>(id) < cmd.can1_motor_commands.size())
            cmd.can1_motor_commands[id] = cmd_val;
          else
            cmd.can1_motor_commands.push_back(cmd_val);
        }
      }
    }
    for (const auto& bus_pair : local_vesc_commands) {
      for (const auto& id_cmd : bus_pair.second) {
        int id = id_cmd.first;
        int16_t cmd_val = id_cmd.second;
        if (bus_pair.first == RcEcatSlave::CanBus::CAN0) {
          vesc_cmds.vesc_can0_motor_commands.push_back(cmd_val);
        } else {  // CAN1
          vesc_cmds.vesc_can1_motor_commands.push_back(cmd_val);
        }
      }
    }
    // if (if_setup_serial_to_32_) {
    //   common::RevData serial_to_32_data;
    //   serial_to_32_data = serial_to_32_manager_->getRevData();
    //   ROS_INFO_STREAM_THROTTLE(
    //       0.5, "serial_to_32_data: " << serial_to_32_data.float_data[0] << "
    //       "
    //                                  << serial_to_32_data.float_data[1] << "
    //                                  "
    //                                  << serial_to_32_data.float_data[2] << "
    //                                  "
    //                                  << serial_to_32_data.float_data[3] << "
    //                                  "
    //                                  << serial_to_32_data.float_data[4] << "
    //                                  "
    //                                  << serial_to_32_data.float_data[5]);
    //   static float senddata[6] = {0};
    //   static uint8_t senddata_int[4] = {0};
    //   for (int i = 0; i < 6; i++) {
    //     senddata[i] += 0.001;
    //   }
    //   for (int i = 0; i < 4; i++) {
    //     senddata_int[i] += 1;
    //   }
    //   serial_to_32_manager_->debugWrite(senddata, senddata_int);
    // }
    // 遍历 actuator_data_map_ 拼接 vesc_msg 后，增加调试日志
    // 调试日志输出 VESC 消息
    // ROS_INFO_STREAM("DEBUG Publishing VESC Msg:");
    // for (size_t i = 0; i < vesc_msg.names.size(); i++) {
    //   ROS_INFO_STREAM("Index " << i << ": name=" << vesc_msg.names[i]
    //                            << ", velocity=" << vesc_msg.velocity[i]
    //                            << ", torque=" << vesc_msg.torque[i]);
    // }
    ecatmsgPublisher_->publish(msg);
    ecatcommandsPublisher_->publish(cmd);
    vescmsgPublisher_->publish(vesc_msg);
    vesccommandsPublisher_->publish(vesc_cmds);
  } else {
  }

  // std::cout << *gpio_manager_.get();
  return true;
}
void RcEcatHW::handleSignal(int /* signum */) {
  update_worker_->stop();
  publish_worker_->stop();
  ecatCheckWorker_->stop();
  ecat_bus_->shutdown();
  ros::shutdown();
  ROS_INFO("ecat bus shutdown");
}

std::ostream& operator<<(std::ostream& os, const RcEcatHW& tmp) {
  os << *(tmp.ecat_slave_);
  return os;
}

// ecat 掉线检测
bool RcEcatHW::ecatCheckWorkerCb(const any_worker::WorkerEvent& event) {
  if (ecat_bus_->checkSlaveStates(0)) {
    inOP_ = false;
  } else {
    inOP_ = true;
  }
  return true;
}

bool RcEcatHW::parseGpioConfig(const XmlRpc::XmlRpcValue& xml_rpc_value) {
  // 解析gpio配置
  gpio_manager_ =
      std::make_shared<ecat_slave_hw::GpioResourceManager>(*this->ecat_slave_);
  int numGpio = xml_rpc_value.size();
  ROS_INFO_STREAM("Found [" << numGpio << "] GPIOs");

  for (size_t i = 0; i < numGpio; ++i) {
    // ecat_slave_hw::GpioData gpio_data_;
    int id;
    std::string name, type;
    bool initial_state;
    try {
      name = static_cast<std::string>(xml_rpc_value[i]["name"]);
      id = static_cast<int>(xml_rpc_value[i]["id"]);
      type = static_cast<std::string>(xml_rpc_value[i]["type"]);
      initial_state = static_cast<bool>(xml_rpc_value[i]["initial_state"]);
    } catch (const XmlRpc::XmlRpcException& exc) {
      ROS_ERROR("Error parsing GPIO[%d]: %s", i, exc.getMessage().c_str());
      return false;
    }

    ecat_slave_hw::GpioData gpio_data;
    gpio_data.type = (type == "input" ? ecat_slave_hw::GpioType::INPUT
                                      : ecat_slave_hw::GpioType::OUTPUT);
    gpio_data.id = id;
    gpio_data.value = initial_state;
    if (!gpio_manager_->addGpioData(name, gpio_data)) {
      ROS_ERROR_STREAM("Failed to add GPIO data: " << name);
      return false;
    }
  }

  return true;
}

bool RcEcatHW::setupGpio() {
  if (!gpio_manager_) {
    ROS_ERROR_STREAM("No GPIO manager create!");
    return false;
  }
  std::vector<ecat_slave_hw::GpioStateHandle> gpio_state_handles;
  gpio_manager_->bindToStateHandle(gpio_state_handles);
  std::vector<ecat_slave_hw::GpioCommandHandle> gpio_command_handles;
  gpio_manager_->bindToCommandHandle(gpio_command_handles);

  // 将句柄注册到接口
  // 将所有句柄注册到接口
  for (const auto& h : gpio_state_handles)
    gpio_state_interface_.registerHandle(h);
  for (const auto& h : gpio_command_handles)
    gpio_command_interface_.registerHandle(h);

  // 注册 GPIO 接口
  registerInterface(&gpio_state_interface_);
  registerInterface(&gpio_command_interface_);

  return true;
}

bool RcEcatHW::parseOps9Config(const XmlRpc::XmlRpcValue& xml_rpc_value) {
  int numOps9 = xml_rpc_value.size();
  for (size_t i = 0; i < numOps9; ++i) {
    std::string name;
    std::string serial_port;
    try {
      name = static_cast<std::string>(xml_rpc_value[i]["name"]);
      serial_port = static_cast<std::string>(xml_rpc_value[i]["serial_port"]);
    } catch (const XmlRpc::XmlRpcException& exc) {
      ROS_ERROR("Error parsing ops9[%d]: %s", i, exc.getMessage().c_str());
      return false;
    }
    serial_hw::serial_settings setting = {
        .serial_fd = 0,
        .serial_port = serial_port,
    };
    common::OpsRevData ops9_data{};
    common::OpsCmdData ops9_cmd{};
    ops9_manager_.addOps9(name, setting, ops9_data, ops9_cmd);
    ROS_INFO_STREAM("add a new ops9: [" << name << "] with serial port: ["
                                        << serial_port << "].");
  }

  ROS_INFO_STREAM("Found [" << numOps9 << "] ops9s");
  return true;
}

bool RcEcatHW::setupOps9(ros::NodeHandle& root_nh) {
  // std::vector<common::Ops9Handle> ops9_handles;
  ops9_manager_.bindToHandle(ops9_handles);
  for (const auto& h : ops9_handles) {
    ops9_interface_.registerHandle(h);
  }
  registerInterface(&ops9_interface_);
  if (ops9_manager_.initOps9(root_nh)) {
    ROS_INFO_STREAM("Ops9 init success");
  } else {
    ROS_ERROR_STREAM("Ops9 init failed");
    return false;
  }
  for (auto it : ops9_manager_.getOps9RevDataMap()) {
    auto ops9_name = it.first;
    auto ops9Publisher = std::make_shared<
        any_node::ThreadedPublisher<ros_ecat_msgs::ActionData>>(
        root_nh.advertise<ros_ecat_msgs::ActionData>(ops9_name + "_msg", 10),
        50, true);
    ops9Publishers_map_[ops9_name] = ops9Publisher;
  }

  ops9UpdateWorker_ = std::make_shared<any_worker::Worker>(
      "ops9UpdateWorker", 0.0005,
      std::bind(&RcEcatHW::ops9UpdateWorkerCb, this, std::placeholders::_1));

  return true;
}

// ops9 更新线程，读取会阻塞，所以更新读取另开线程
bool RcEcatHW::ops9UpdateWorkerCb(const any_worker::WorkerEvent& event) {
  ros::Duration period(event.timeStep);
  ros::Time tp = ros::Time::now();
  // update the ops9 data
  ops9_manager_.readOps9(tp, period);
  // ops9 如果是使用usb直连小电脑，就可以直接发布
  ros_ecat_msgs::ActionData ops9_msg;
  for (auto it : ops9_manager_.getOps9RevDataMap()) {
    auto ops9_name = it.first;
    auto ops9_data = it.second;
    ops9_msg.name = ops9_name;
    ops9_msg.header.frame_id = "action";
    ops9_msg.yaw_angle = ops9_data.yaw_angle;
    ops9_msg.pose_x = ops9_data.pos_x;
    ops9_msg.pose_y = ops9_data.pos_y;
    ops9_msg.omega = ops9_data.omega;
    ops9Publishers_map_[ops9_name]->publish(ops9_msg);
    // 使用动态变换发布器广播新的 "action" 坐标
    // geometry_msgs::TransformStamped transformStamped;
    // tf2::Quaternion q;
    // q.setRPY(0.0, 0.0, ops9_data.yaw_angle);
    // q.normalize();
    // transformStamped.header.stamp = tp;
    // transformStamped.header.frame_id = "map";
    // transformStamped.child_frame_id = "action";
    // transformStamped.transform.translation.x = -ops9_data.pos_y;
    // transformStamped.transform.translation.y = ops9_data.pos_x;
    // transformStamped.transform.translation.z = 0.0;
    // transformStamped.transform.rotation.x = q.x();
    // transformStamped.transform.rotation.y = q.y();
    // transformStamped.transform.rotation.z = q.z();
    // transformStamped.transform.rotation.w = q.w();
    // 发布变换
    // dynamic_broadcaster_.sendTransform(transformStamped);
  }

  return true;
}

bool RcEcatHW::parseSerialTo32Config(const XmlRpc::XmlRpcValue& xml_rpc_value) {
  if (!xml_rpc_value.hasMember("name")) return false;
  serial_to_32_name_ = static_cast<std::string>(xml_rpc_value["name"]);
  ROS_INFO_STREAM("Found serial_to_32: " << serial_to_32_name_);
  return true;
}

bool RcEcatHW::setupSerialTo32(ros::NodeHandle& serial_nh) {
  // 先完成构造
  serial_to_32_manager_ =
      std::make_shared<serial_hw::SerialTo32ResourceManager>(serial_nh);

  serial_to_32_manager_->setName(serial_to_32_name_);
  if (!serial_to_32_manager_->init()) {
    ROS_ERROR_STREAM("SerialTo32 init failed");
    return false;
  }
  /* 注册句柄 */
  serial_to_32_manager_->bindToHandle(serial_to_32_handle_);
  /* 将句柄注册到接口 */
  serial_to_32_interface_.registerHandle(serial_to_32_handle_);
  ROS_INFO_STREAM(
      "SerialTo32Handle registered, name: " << serial_to_32_handle_.getName());
  /* 将接口注册到控制器管理器 */
  registerInterface(&serial_to_32_interface_);
  return true;
}

}  // namespace rc_ecat_master