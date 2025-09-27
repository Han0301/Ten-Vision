#include "ctrl_common/PID/IncrementalPID.h"

namespace ctrl_common
{
void IncrementalPID::init(double p, double i, double d, double i_max, double output_max,double deadzone)
{
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
}

double IncrementalPID::PIDCalculate(float CurrentPoint, float NextPoint)
{
	if(ABS(Error_) <  deadzone_)Error_ = 0;

    Error_ =  NextPoint - CurrentPoint;                               
	DError_ = Error_ - LastError_;
	
	Output_ +=  kp_ * (DError_)+ PID_abs_limit( ki_ * Error_,  i_max_ ) +  kd_ * ( Error_ +   PrevError_ - 2.0* LastError_);  

	if( Output_ >  output_max_ )   Output_ =  output_max_;
	if( Output_ < -  output_max_ )  Output_ = - output_max_;
	 PrevError_ =  LastError_;  
	 LastError_ = Error_;
	
	// if(ABS(Error_) <  deadzone_)
	// {
	// 	Output_ = 0;
	// }
    return Output_;
}

double IncrementalPID::PIDCalculateByError(float error)
{
	if(ABS(Error_) <  deadzone_)Error_ = 0;

    Error_ =  error;                               
	DError_ = Error_ - LastError_;
	
	Output_ +=  kp_ * (DError_)+ PID_abs_limit( ki_ * Error_,  i_max_ ) +  kd_ * ( Error_ +   PrevError_ - 2.0* LastError_);  

	if( Output_ >  output_max_ )   Output_ =  output_max_;
	if( Output_ < -  output_max_ )  Output_ = - output_max_;
	 PrevError_ =  LastError_;  
	 LastError_ = Error_;
	
	// if(ABS(Error_) <  deadzone_)
	// {
	// 	Output_ = 0;
	// }
    return Output_;
}

}