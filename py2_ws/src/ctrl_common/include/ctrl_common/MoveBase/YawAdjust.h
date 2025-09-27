#pragma once
#ifndef ABS
#define ABS(x) ((x) < 0 ? -(x) : (x))
#endif
#include "ctrl_common/PID/PositionPID.h"

namespace ctrl_common
{
class YawAdjust
{
public:
    YawAdjust()= default;
    ~YawAdjust()= default;

    void init(double p, double i, double d, double i_max, double output_max , double deadzone);

    double YawAdjustCalculate(double Current_angle, double Target_angle);

    void AngleLimit(double *angle)
    {
        static uint8_t recursiveTimes = 0;
        
        recursiveTimes++;
        
        if(recursiveTimes<100)
        {
            if(*angle>180.0f)
            {
                *angle-=360.0f;
                AngleLimit(angle);
            }
            else if(*angle<-180.0f)
            {
                *angle+=360.0f;
                AngleLimit(angle);
            }
        }
        
        recursiveTimes--;
    }

private:
    double Output_;
    PositionPID yaw_pid_;
};

}