#include "ctrl_common/MoveBase/YawAdjust.h"

namespace ctrl_common
{
void YawAdjust::init(double p, double i, double d, double i_max, double output_max , double deadzone)
{
    yaw_pid_.init(p, i, d, i_max, output_max, deadzone);
}

double YawAdjust::YawAdjustCalculate(double Current_angle, double Target_angle)
{
    double YawAdjust_error;
 
	 // 计算误差
   if(Current_angle*Target_angle >= 0)
   {
      YawAdjust_error = Target_angle - Current_angle;
   }

   else
   {
		 if(ABS(Current_angle)+ABS(Target_angle) <= 180) YawAdjust_error = Target_angle - Current_angle;
		 else 
		 {
			AngleLimit(&YawAdjust_error);
		 }
   }
   
   // 直接利用PID输出角速度
   Output_=yaw_pid_.PIDCalculateByError(YawAdjust_error);
   
   return Output_;
}

}