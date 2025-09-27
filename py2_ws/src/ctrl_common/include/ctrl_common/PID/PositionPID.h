#pragma once

#include <stdint.h>
#ifndef ABS
#define ABS(x) ((x) < 0 ? -(x) : (x))
#endif

namespace ctrl_common {
class PositionPID {
 public:
  PositionPID() = default;
  ~PositionPID() = default;

  void init(double p, double i, double d, double i_max, double output_max,
            double deadzone);

  double PIDCalculate(float CurrentPoint, float NextPoint);

  void IntegrationClean()
  {
    SumError_ = 0;
    // DError_ =0.0;
  }

  double PIDCalculateByError(float error);

  float PID_abs_limit(float a, float ABS_MAX) {
    if (a > ABS_MAX) a = ABS_MAX;

    if (a < -ABS_MAX) a = -ABS_MAX;
    return a;
  }

 private:
  double kp_, ki_, kd_, i_max_, output_max_, deadzone_;
  double Error_, DError_, LastError_, PrevError_, SumError_, Output_;
  uint8_t first_flag_ = 1;
};
}  // namespace ctrl_common