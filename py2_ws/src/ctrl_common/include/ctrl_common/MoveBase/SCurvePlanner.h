/**
 * @file pathplanning.h
 * @author 冯大帅将军
 * @brief 
 * @version 0.1
 * @date 2025-5-4
 * 
 * @copyright Copyright (c) 2025
 * 
 * @attention :
 * @note : 路径规划器
 *         1.s曲线规划器
 *         
 * @versioninfo :
 */
#ifndef PATHPLANNING_H
#define PATHPLANNING_H

#ifdef __cplusplus
extern "C"{
#endif

/*----------------------------------include-----------------------------------*/
#include "user_tool.h"
#include "pid_controller.h"

class SCurvePlanner {
    private:
        // 运动参数
        double q0_;          // 初始位置
        double q1_;          // 目标位置
        double v0_;          // 初始速度
        double v1_;          // 末端速度
        double v_max_;       // 最大速度
        double a_max_;       // 最大加速度
        double j_max_;       // 最大加加速度
        
        // 时间参数
        double Ta_, Tv_, Td_; // 加速/匀速/减速段时间
        double Tj1_, Tj2_;    // 加加速段时间
        double total_time_;   // 总时间
        double t_; // 当前时间
        bool initialized_;    // 初始化标志
    
        double calculateVelocityLimit(double q_total, double a_max__ ,double j_max__);
        void calculateTimingParameters(double desired_time);

    
    public:
        SCurvePlanner(double q_start, double q_end, 
                    double v_start, double v_end,
                    double v_limit, double a_limit,
                    double j_limit, double desired_time) 
            : q0_(q_start), q1_(q_end), v0_(v_start), v1_(v_end),
              v_max_(v_limit), a_max_(a_limit), j_max_(j_limit),
              t_(0.0), initialized_(false) 
        {            
            calculateTimingParameters(desired_time);
            initialized_ = true;
        }
    
        // 更新并获取下一时刻速度
        double update(double dt);
    
        // 获取规划进度
        double progress() const { 
            return t_ / total_time_; 
        }
        
        // 重置规划器
        void reset(double q_start_offset) {
            t_ = 0.0;
            q0_ += q_start_offset;
        }

        bool arrived();
};


#ifdef __cplusplus
}
#endif

#endif	/* PATHPLANNING_H */