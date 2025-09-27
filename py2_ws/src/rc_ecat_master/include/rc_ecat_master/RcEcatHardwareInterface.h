/**
 * @file RcEcatHardwareInterface.h
 * @author Keten (2863861004@qq.com)
 * @brief 硬件接口（临时）
 * @version 0.1
 * @date 2025-05-04
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include <XmlRpc.h>
#include <controller_manager/controller_manager.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <rc_ecat_master/ActuatorData.h>
#include <rc_ecat_master/GpioData.h>
#include <rc_ecat_master/common/hardware_interface/Ops9Interface.h>
#include <rc_ecat_master/common/hardware_interface/SerialTo32Interface.h>
#include <rc_ecat_master/ecat_hardware_interface/gpio/GpioInterface.h>
#include <rc_ecat_master/ecat_hardware_interface/gpio/GpioResourceManager.h>
#include <rc_ecat_master/serial_hw/ops9/Ops9ResourceManager.h>
#include <rc_ecat_master/serial_hw/serial_to_32/SerialTo32ResourceManager.h>
#include <ros/ros.h>
#include <ros_ecat_msgs/DJIEcatRosCommands.h>
#include <ros_ecat_msgs/DJIEcatRosMsg.h>
#include <ros_ecat_msgs/VescEcatRosCommands.h>
#include <ros_ecat_msgs/VescEcatRosMsg.h>
#include <soem_rsl/ethercat.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>
#include <urdf/model.h>
#include <yaml-cpp/yaml.h>

#include <any_node/ThreadedPublisher.hpp>
#include <any_worker/Worker.hpp>
#include <chrono>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <rc_ecat_master/RcEcatSlave.hpp>
#include <signal_handler/SignalHandler.hpp>
#include <soem_interface_rsl/EthercatBusBase.hpp>
#include <string>
#include <vector>

namespace rc_ecat_master {

template <typename T>
T minAbs(T a, T b) {
  T sign = (a < 0.0) ? -1.0 : 1.0;
  return sign * fmin(fabs(a), b);
}

class RcEcatHW : public hardware_interface::RobotHW {
 public:
  RcEcatHW() = default;
  // override
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

  bool parseEcatSlaveHWConfig(const std::string& yaml_path);
  /**
   * @brief
   *
   * @param root_nh
   * @param yaml_path
   * @return true
   * @return false
   */
  bool parseActuatorConfig(ros::NodeHandle& root_nh,
                           const std::string& yaml_path);

  /**
   * @brief
   *
   * @param xml_rpc_value
   * @return true
   * @return false
   */
  bool parseActuators(const XmlRpc::XmlRpcValue& xml_rpc_value);

  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool setupHandle();

  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool setupActuator();

  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool setupJointLimits(ros::NodeHandle& root_nh);

  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool setupTransmission();

  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool parseEcatConfig(const std::string& yaml_path);

  /**
   * @brief 启动Ecat
   *
   * @return true
   * @return false
   */
  bool setupEcat();

  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool parseGpioConfig(const XmlRpc::XmlRpcValue& xml_rpc_value);

  /**
   * @brief
   *
   * @return true
   * @return false
   */
  bool setupGpio();

  /**
   * @brief 粘贴Ops9配置
   *
   * @param xml_rpc_value
   * @return true
   * @return false
   */
  bool parseOps9Config(const XmlRpc::XmlRpcValue& xml_rpc_value);

  /**
   * @brief 设置Ops9
   *
   * @return true
   * @return false
   */
  bool setupOps9(ros::NodeHandle& root_nh);

  /**
   * @brief
   *
   * @param xml_rpc_value
   * @return true
   * @return false
   */
  bool parseSerialTo32Config(const XmlRpc::XmlRpcValue& xml_rpc_value);

  /**
   * @brief
   *
   * @param serial_nh
   * @return true
   * @return false
   */
  bool setupSerialTo32(ros::NodeHandle& serial_nh);

  /* after init and official */
  void analysisActuatorData();

  void setActuatorCommands();

  void saveActuatorCommands();

  bool updateWorkerCb(const any_worker::WorkerEvent& event);

  bool updatePublishCb(const any_worker::WorkerEvent& event);

  void handleSignal(int /* signum */);

  ros::Time getReadStamp() const { return read_stamp_; }

  ros::Time getWriteStamp() const { return write_stamp_; }

  friend std::ostream& operator<<(std::ostream& os, const RcEcatHW& tmp);

  bool ecatCheckWorkerCb(const any_worker::WorkerEvent& event);

  bool ops9UpdateWorkerCb(const any_worker::WorkerEvent& event);

 private:
  void setZeroOutputCallback();

 private:
  /* ecat relative */
  std::string slave_name_;
  std::string bus_name_;
  uint32_t slave_address_;
  std::shared_ptr<rc_ecat_master::RcEcatSlave> ecat_slave_;
  std::unique_ptr<soem_interface_rsl::EthercatBusBase> ecat_bus_;
  bool inOP_;
  int wkc_, expected_wkc_;
  std::shared_ptr<any_worker::Worker> ecatCheckWorker_;  // ecat 主站看门狗线程

  /* ros-control relative */
  std::string urdf_string_;
  /* controller_manager relative */
  std::shared_ptr<controller_manager::ControllerManager> controller_manager_;
  /* joint interface relative */
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::EffortJointInterface effort_joint_interface_;
  hardware_interface::ActuatorStateInterface act_state_interface_;
  hardware_interface::EffortActuatorInterface effort_actuator_interface_;

  /* transmission relative */
  struct TransmissionGroupData {
    // 下面的类型为 transmission_interface 里对应的数据结构，
    // 其中每个成员是 vector，保存各个电机数据的指针。
    transmission_interface::ActuatorData act_data;
    transmission_interface::JointData jnt_data;
    transmission_interface::ActuatorData act_cmd;
    transmission_interface::JointData jnt_cmd;
    // 用来保存当前分组的 transmission 指针
    transmission_interface::SimpleTransmission* trans_ptr;
  };
  std::vector<TransmissionGroupData> transmission_group_data_{};

  std::unique_ptr<transmission_interface::ActuatorToJointStateInterface>
      act_to_jnt_state_;
  std::unique_ptr<transmission_interface::JointToActuatorEffortInterface>
      jnt_to_act_effort_;
  transmission_interface::ActuatorData a_data;
  transmission_interface::JointData j_data;
  transmission_interface::ActuatorData a_cmd_;
  transmission_interface::JointData j_cmd_;
  std::vector<transmission_interface::SimpleTransmission>
      transmission_reducer_{};  // 减速比器
  std::unordered_map<std::string, rc_ecat_master::ActuatorCoefficients>
      actuator_coefficients_map_{};  // 存放电机系数
  std::unordered_map<
      std::string,
      std::unordered_map<std::string, rc_ecat_master::ActuatorData>>
      actuator_data_map_{};  // 存放电机数据 <类型 , <名字，数据>>
  std::mutex actuator_data_map_lock_;

  /* joint limits relative */
  joint_limits_interface::EffortJointSaturationInterface
      effort_joint_saturation_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface
      effort_joint_soft_limits_interface_;

  /* Joint handle relative */
  std::unordered_map<std::string, std::string> joint_names_map_{};
  std::vector<hardware_interface::JointHandle> joint_handles_{};

  /* others */
  std::shared_ptr<any_worker::Worker> update_worker_;
  std::shared_ptr<any_worker::Worker> publish_worker_;
  any_node::ThreadedPublisherPtr<ros_ecat_msgs::DJIEcatRosMsg>
      ecatmsgPublisher_;
  any_node::ThreadedPublisherPtr<ros_ecat_msgs::DJIEcatRosCommands>
      ecatcommandsPublisher_;
  any_node::ThreadedPublisherPtr<ros_ecat_msgs::VescEcatRosMsg>
      vescmsgPublisher_;
  any_node::ThreadedPublisherPtr<ros_ecat_msgs::VescEcatRosCommands>
      vesccommandsPublisher_;

  ros::Time read_stamp_, write_stamp_;  // 保存读写时间戳
  std::mutex commandsMutex_;
  std::unordered_map<RcEcatSlave::CanBus, std::unordered_map<int, int16_t>>
      latest_commands_;
  std::unordered_map<RcEcatSlave::CanBus, std::unordered_map<int, int32_t>>
      latest_vesc_commands_;

  /* can relative */
  int has_can_num_ = 0;

  /* gpio relative */
  std::shared_ptr<ecat_slave_hw::GpioResourceManager> gpio_manager_;
  ecat_slave_hw::GpioStateInterface gpio_state_interface_;
  ecat_slave_hw::GpioCommandInterface gpio_command_interface_;
  int input_gpio_num_ = 0;
  int output_gpio_num_ = 0;

  /* Ops9 relative */
  serial_hw::Ops9ResourceManager ops9_manager_;
  common::Ops9Interface ops9_interface_;
  std::unordered_map<std::string,
                     any_node::ThreadedPublisherPtr<ros_ecat_msgs::ActionData>>
      ops9Publishers_map_;  // 调试用话题，使用控制器时需要把这个关闭
  std::vector<common::Ops9Handle>
      ops9_handles;  // 保存这些句柄不是必须，可以用临时变量来代替
  std::shared_ptr<any_worker::Worker> ops9UpdateWorker_;
  //   tf2 变换发布器
  tf2_ros::TransformBroadcaster dynamic_broadcaster_;
  bool ops9_init_{false};

  /* serial_to_32 relative */
  common::SerialTo32Handle serial_to_32_handle_;
  std::shared_ptr<serial_hw::SerialTo32ResourceManager> serial_to_32_manager_;
  std::string serial_to_32_name_;
  common::SerialTo32Interface serial_to_32_interface_;
  bool if_setup_serial_to_32_{false};

  /* sub to the set zero output topic */
  // 当急停断掉电机的强电时，会导致ecat从站一直向主站发送断电前的状态直到从站can总线恢复工作
  // 所以添加订阅一个清零话题，在需要重新加载的地方先执行该函数进行从站数据的刷新
  ros::Subscriber set_zero_sub_;
};

}  // namespace rc_ecat_master