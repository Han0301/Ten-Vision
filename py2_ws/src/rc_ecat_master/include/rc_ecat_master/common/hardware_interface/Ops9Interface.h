/**
 * @file Ops9Interface.h
 * @author Keten (2863861004@qq.com)
 * @brief 东大全场定位的硬件接口
 * @version 0.1
 * @date 2025-05-12
 *
 * @copyright Copyright (c) 2025
 *
 * @attention :
 * @note :
 * @versioninfo :
 */
#pragma once

#include <hardware_interface/internal/hardware_resource_manager.h>

#include <string>

namespace common {

struct OpsRevData {
  double yaw_angle;
  double pos_x;
  double pos_y;
  double omega;
};

struct OpsCmdData {
  bool calibration_state;
  bool reset_state;
  bool update_x_state;
  double update_x;
  bool update_y_state;
  double update_y;
  bool update_x_y_state;
  bool update_yaw_state;
  double update_yaw;
  bool update_yaw_x_y_state;
};

class Ops9Handle {
 public:
  Ops9Handle() = default;
  explicit Ops9Handle(const std::string& name, double* pos_x, double* pos_y,
                      double* yaw_angle, double* omega, bool* calibration_state,
                      bool* reset_state, bool* updatePosX, bool* updatePosY,
                      bool* updateYawAngle, bool* updateYawPosXY,
                      bool* updatePosXY, double* set_pos_x, double* set_pos_y,
                      double* set_yaw_angle)
      : name_(name),
        pos_x_(pos_x),
        pos_y_(pos_y),
        yaw_angle_(yaw_angle),
        omega_(omega),
        calibration_state_(calibration_state),
        reset_state_(reset_state),
        set_pos_x_(set_pos_x),
        set_pos_y_(set_pos_y),
        set_yaw_angle_(set_yaw_angle),
        update_x_state_(updatePosX),
        update_y_state_(updatePosY),
        update_x_y_state_(updatePosXY),
        update_yaw_state_(updateYawAngle),
        update_yaw_x_y_state_(updateYawPosXY) {}
  ~Ops9Handle() = default;

  /* some function to get the sensor data */

  std::string getName() const { return name_; }

  double getYawAngle() const {
    assert(yaw_angle_);  // Ensure raw_data_ is not null
    return *yaw_angle_;
  }
  double getPosX() const {
    assert(pos_x_);  // Ensure raw_data_ is not null
    return *pos_x_;
  }
  double getPosY() const {
    assert(pos_y_);  // Ensure raw_data_ is not null
    return *pos_y_;
  }
  double getOmega() const {
    assert(omega_);  // Ensure raw_data_ is not null
    return *omega_;
  }

  bool setCalibrationState() {
    assert(calibration_state_);  // Ensure raw_data_ is not null
    *calibration_state_ = true;
    return true;
  }

  bool setResetState() {
    assert(reset_state_);  // Ensure raw_data_ is not null
    *reset_state_ = true;
    return true;
  }

  bool updatePosX(double pos_x) {
    assert(set_pos_x_);  // Ensure raw_data_ is not null
    assert(update_x_state_);
    *set_pos_x_ = pos_x;
    *update_x_state_ = true;
    return true;
  }

  bool updatePosY(double pos_y) {
    assert(set_pos_y_);  // Ensure raw_data_ is not null
    assert(update_y_state_);
    *set_pos_y_ = pos_y;
    *update_y_state_ = true;
    return true;
  }

  bool updateYawAngle(double yaw_angle) {
    assert(set_yaw_angle_);  // Ensure raw_data_ is not null
    assert(update_yaw_state_);
    *set_yaw_angle_ = yaw_angle;
    *update_yaw_state_ = true;
    return true;
  }

  bool updatePosXY(double pos_x, double pos_y) {
    assert(set_pos_x_);  // Ensure raw_data_ is not null
    assert(set_pos_y_);  // Ensure raw_data_ is not null
    assert(update_x_y_state_);
    *set_pos_x_ = pos_x;
    *set_pos_y_ = pos_y;
    *update_x_y_state_ = true;
    return true;
  }

  bool updateYawPosXY(double yaw_angle, double pos_x, double pos_y) {
    assert(set_pos_x_);      // Ensure raw_data_ is not null
    assert(set_pos_y_);      // Ensure raw_data_ is not null
    assert(set_yaw_angle_);  // Ensure raw_data_ is not null
    assert(update_yaw_x_y_state_);
    *set_pos_x_ = pos_x;
    *set_pos_y_ = pos_y;
    *set_yaw_angle_ = yaw_angle;
    *update_yaw_x_y_state_ = true;
    return true;
  }

 private:
  std::string name_;
  /* get the data from sensor */
  double* yaw_angle_ = {nullptr};
  double* pos_x_ = {nullptr};
  double* pos_y_ = {nullptr};
  double* omega_ = {nullptr};
  /* send the data to the sensor */
  bool* calibration_state_ = {nullptr};
  bool* reset_state_ = {nullptr};
  bool* update_x_state_ = {nullptr};
  bool* update_y_state_ = {nullptr};
  bool* update_yaw_state_ = {nullptr};
  bool* update_x_y_state_ = {nullptr};
  bool* update_yaw_x_y_state_ = {nullptr};
  double* set_pos_x_ = {nullptr};
  double* set_pos_y_ = {nullptr};
  double* set_yaw_angle_ = {nullptr};
};

class Ops9Interface : public hardware_interface::HardwareResourceManager<
                          Ops9Handle, hardware_interface::ClaimResources> {};
}  // namespace common