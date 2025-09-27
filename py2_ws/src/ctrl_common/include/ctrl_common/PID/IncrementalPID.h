#pragma once

#ifndef ABS
#define ABS(x) ((x) < 0 ? -(x) : (x))
#endif

namespace ctrl_common {
class IncrementalPID {
 public:
  IncrementalPID() = default;
  ~IncrementalPID() = default;

  void init(double p, double i, double d, double i_max, double output_max,
            double deadzone);

  double PIDCalculate(float CurrentPoint, float NextPoint);
  double PIDCalculateByError(float error);

  float PID_abs_limit(float a, float ABS_MAX) {
    if (a > ABS_MAX) a = ABS_MAX;

    if (a < -ABS_MAX) a = -ABS_MAX;
    return a;
  }

 private:
  double kp_, ki_, kd_, i_max_, output_max_, deadzone_;
  double Error_, DError_, LastError_, PrevError_, Output_;
};
}  // namespace ctrl_common
