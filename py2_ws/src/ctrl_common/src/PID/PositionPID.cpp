#include "ctrl_common/PID/PositionPID.h"

namespace ctrl_common {
void PositionPID::init(double p, double i, double d, double i_max,
                       double output_max, double deadzone) {
  kp_ = p;
  ki_ = i;
  kd_ = d;
  i_max_ = i_max;
  output_max_ = output_max;
  deadzone_ = deadzone;

  Error_ = 0;
  DError_ = 0;
  LastError_ = 0;
  PrevError_ =0;
  SumError_=0;
  first_flag_ = 1;
}

double PositionPID::PIDCalculate(float CurrentPoint, float NextPoint) {
  if (first_flag_ == 1) {
    LastError_ = NextPoint - CurrentPoint;
    PrevError_ = NextPoint - CurrentPoint;
    first_flag_ = 0;
  }

  Error_ = NextPoint - CurrentPoint;
  SumError_ += Error_;
  DError_ = Error_ - LastError_;

  Output_ =
      kp_ * Error_ + PID_abs_limit(ki_ * SumError_, i_max_) + kd_ * DError_;

  if (Output_ > output_max_) Output_ = output_max_;
  if (Output_ < -output_max_) Output_ = -output_max_;
  LastError_ = Error_;

  if (ABS(Error_) < deadzone_) {
    Output_ = 0;
  }
  return Output_;
}

double PositionPID::PIDCalculateByError(float error_) {
  if (first_flag_ == 1) {
    LastError_ = error_;
    PrevError_ = error_;
    first_flag_ = 0;
  }

  Error_ = error_;
  SumError_ += Error_;
  DError_ = Error_ - LastError_;

  Output_ =
      kp_ * Error_ + PID_abs_limit(ki_ * SumError_, i_max_) + kd_ * DError_;

  if (Output_ > output_max_) Output_ = output_max_;
  if (Output_ < -output_max_) Output_ = -output_max_;
  LastError_ = Error_;

  if (ABS(Error_) < deadzone_) {
    Output_ = 0;
  }
  return Output_;
}

}  // namespace ctrl_common