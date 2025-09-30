/**
 * @file pid.cpp
 * @author Yang JianYi 
 * @brief PID类的实现文件，该文件封装了基本PID的算法实现，包括微分先行，不完全微分，积分限幅，积分隔离以及增量式和位置式PID的实现。只需要修改PID的配置参数即可使用。
 * 
 * ******************************************************************************************************************************
 *  使用发方法：
 *      配置PID参数(PID_Param_Init(),PID_Mode_Init())。 实时传入current值和target值，然后调用Adjust函数获取结果即可。
 * ******************************************************************************************************************************
 * @version 0.1
 * @date 2024-05-16
 * 
 * @brief 我不知道为什么钟忆要在原本完整的文件里塞他自己那套pid ^-^
 */
#include "pid.h"

SystemTick_Fun PidTimer::get_systemTick = NULL;

uint8_t PidTimer::update_timeStamp(void)
{
    uint32_t now_time=0;

    //Check get sysClock
    if(PidTimer::get_systemTick != NULL)
    {
        if(last_time == 0)
        {
            last_time = PidTimer::get_systemTick();
            return 1;
        }
        now_time = PidTimer::get_systemTick();

        //overflow
        if(now_time < last_time)
            dt = (float)((now_time + 0xFFFFFFFF) - last_time);  //now_time is 32bit, use 0XFFFFFFFF to avoid overflow
        else
            dt = (float)(now_time - last_time);
        
        last_time = now_time;
        
        dt *= 0.000001f;

        return 0;
    }
    else
    {
        dt = 0;
        return 1;
    }
}

uint8_t PidTimer::getMicroTick_regist(uint32_t (*getTick_fun)(void))
{
    if(getTick_fun != NULL)
    {
        PidTimer::get_systemTick = getTick_fun;
        return 1;
    }
    else 
        return 0;
}

float PID::Adjust(void)
{
    //get real time failed
    if(update_timeStamp())
        return 0;
    

    // 防止 dt 过小导致微分爆炸
    if (dt < 1e-6f)
        dt = 1e-6f;
    
    //calculate error and deadzone
    error = target - current;
    if(_tool_Abs(error) < DeadZone)
    {
        Out=0;
        return Out;
    }
    
    //lowpass filter, change the trust value to adjust the filter
    error = LowPass_error.f(error);

    if(Imcreatement_of_Out)     //output increment mode
        P_Term = Kp * error - Kp * pre_error;
    else                        //output position mode
          P_Term = Kp * error;

    //calculate integral term, if use integral term
    if(Ki!=0)
    {
        if(Imcreatement_of_Out)
            integral_e = error;
        else
            integral_e  += error*dt;
        _tool_Constrain(&integral_e, -I_Term_Max/Ki, I_Term_Max/Ki);
    }
    else
    {
        integral_e = 0;
    }

    //integral separate
    if(_tool_Abs(error) < I_SeparThresh)
    {
        I_Term = Ki * integral_e;
        _tool_Constrain(&I_Term, -I_Term_Max, I_Term_Max);
    }
    else
    {
        I_Term = 0;
    }

    float d_err = 0;
    if(D_of_Current)
    {
        if(Imcreatement_of_Out)
            d_err = (current + eriler_Current - 2*pre_Current) / dt;
        else
            d_err = (current - pre_Current) / dt;
    }
    else
    {
        if(Imcreatement_of_Out)
            d_err = (error + eriler_error - 2*pre_error) / dt;
        else
            d_err = (error - pre_error) / dt;
    }

    d_err = LowPass_d_err.f(d_err);     //进行不完全微分
    D_Term = Kd * d_err;

    eriler_error = pre_error;
    pre_error = error;
    eriler_Current = pre_Current;
    pre_Current = current;
    // 计算 PID 输出
    if(Imcreatement_of_Out)
        Out = P_Term + I_Term + D_Term + last_out;
    else
        Out = P_Term + I_Term + D_Term;
    last_out = Out;

    _tool_Constrain(&Out, -Out_Max, Out_Max);

    return Out;
}

/**
 * @brief PID参数初始化
 * 
 * @param _Kp  比例系数
 * @param _Ki  积分系数
 * @param _Kd  微分系数
 * @param _I_Term_Max  I项限幅
 * @param _Out_Max  输出限幅
 * @param DeadZone  死区，fabs(error)小于DeadZone时，输出为0。设为负数不启用死区
 * @param I_SeparThresh  积分分离阈值，fabs(error)大于该阈值取消积分作用。
 */
void PID::PID_Param_Init(float _Kp, float _Ki, float _Kd, float _I_Term_Max, float _Out_Max, float DeadZone)
{
    DeadZone = DeadZone;
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
    I_Term_Max = _I_Term_Max;
    Out_Max = _Out_Max;
}

// void LimitMax(float *input, float max)
// {
//     if (*input > max)
//         *input = max;
//     if (*input < -max)
//         *input = -max;
// }

/**
 * @brief PID参数初始化
 * @param pid
 * @param mode
 * @param maxOutput
 * @param integralLimit
 * @param IntegralSeparate
 * @param deadband
 * @param max_err
 * @param kp
 * @param ki
 * @param kd
 */
void pid_param_init(
    PID_T *pid,
    pidSt mode,
    float maxOutput,
    float integralLimit,
    float IntegralSeparate,
    float deadband,
    float max_err,
    float kp,
    float ki,
    float kd)
{
    pid->mode = mode;
    pid->MaxOutput = maxOutput;
    pid->IntegralLimit = integralLimit;
    pid->IntegralSeparate = IntegralSeparate;
    pid->DeadBand = deadband;
    pid->Max_Err = max_err;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->last_output = 0;
    pid->output = 0;
    pid->first_flag = 1;
}

void pid_fast_init(PID_T *pid, float maxOutput, float kp, float ki, float kd)
{
    pid_param_init(pid, PID_Position, maxOutput, maxOutput / 8, 0, 0, 0, kp, ki, kd);
}

/**
 * @brief 中途修改pid参数
 * @param pid PID结构体
 * @param kp 参数p
 * @param ki 参数i
 * @param kd 参数d
 */
void pid_reset(PID_T *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->first_flag = 1;
}

/**
 * @brief PID计算
 * @param pid PID结构体
 * @param measure 当前值
 * @param target 目标值
 * @return
 */
float pid_calc(PID_T *pid, float target, float measure)
{
    pid->measure[NOW] = measure; // 更新本次最新测量值
    pid->target[NOW] = target;
    pid->err[NOW] = pid->target[NOW] - pid->measure[NOW]; // 更新误差值

    if (pid->first_flag == 1)
    {
        pid->err[LAST] = pid->err[NOW]; // 第一次运行，上次误差等于本次误差
        pid->err[PREV] = pid->err[NOW]; // 第一次运行，上上次误差等于本次误差
        pid->first_flag = 0;
    }

    if (pid->Max_Err != 0 && _tool_Abs(pid->err[NOW]) > pid->Max_Err) // 误差超过限制跳出
    {
        pid->output = 0;
        return 0;
    }

    if (pid->DeadBand != 0 && _tool_Abs(pid->err[NOW]) < pid->DeadBand) // 误差小于死区跳出
    {
        pid->output = 0;
        return 0;
    }

    if (pid->mode == PID_Position)
    {
        pid->pout = pid->kp * pid->err[NOW]; // p输出为Kp*误差

        if (pid->IntegralSeparate == 0)
            pid->iout += pid->ki * pid->err[NOW]; // i输出为i+ki*误差
        else
        { // 积分分离
            if (_tool_Abs(pid->err[NOW]) < pid->IntegralSeparate)
                pid->iout += pid->ki * pid->err[NOW]; // i输出为i+ki*误差
            else
                pid->iout = 0; // i输出为0
        }
        _tool_Constrain(&(pid->iout), -pid->IntegralLimit, pid->IntegralLimit); // 积分是否超出限制

        pid->dout = pid->kd * (pid->err[NOW] - pid->err[LAST]); // d输出为kd*（误差-上次误差）

        pid->fout = pid->kf * (pid->target[NOW] - pid->target[LAST]); // 前馈输出

        pid->output = pid->pout + pid->iout + pid->dout + pid->fout; // pid输出和
        // pid->output = pid->output*0.7f + pid->last_output*0.3f;       //滤波
        _tool_Constrain(&(pid->output), -pid->MaxOutput, pid->MaxOutput);
        pid->last_output = pid->output; // 更新数据
    }
    else if (pid->mode == PID_Incremental)
    {
        pid->pout = pid->kp * (pid->err[NOW] - pid->err[LAST]); // p输出为Kp * 误差增量
        pid->iout = pid->ki * pid->err[NOW];                    // i输出为ki * 误差
        pid->dout = pid->kd * (pid->err[NOW] - (2 * pid->err[LAST]) + pid->err[PREV]);
        // d输出为kd *（误差-2*上次误差+上上次误差）

        // LimitMax(&(pid->iout), pid->IntegralLimit);         //积分是否超出限制
        pid->output = pid->last_output + (pid->pout + pid->iout + pid->dout); // pid输出和
        // pid->output = pid->output*0.7f + pid->last_output*0.3f;   //滤波？
        _tool_Constrain(&(pid->output), -pid->MaxOutput, pid->MaxOutput);
        pid->last_output = pid->output;
    }

    //  数据更新
    pid->err[PREV] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->measure[PREV] = pid->measure[LAST];
    pid->measure[LAST] = pid->measure[NOW];
    pid->target[PREV] = pid->target[LAST];
    pid->target[LAST] = pid->target[NOW];

    return pid->output;
}

float pid_calc_by_error(PID_T *pid, float error)
{
    return pid_calc(pid, 0.0f, error);
}
