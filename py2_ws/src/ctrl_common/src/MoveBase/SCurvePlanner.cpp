/**
 * @file pathplanning.cpp
 * @author 冯大帅将军
 * @brief 
 * @version 0.1
 * @date 2025-5-4
 * 
 * @copyright Copyright (c) 2025
 * 
 * @attention :
 * @note :
 * @versioninfo :
 */
#include "pathplanning.h"

double SCurvePlanner::calculateVelocityLimit(double q_total, double a_max__ ,double j_max__)
{
    return sqrt((2*j_max__*a_max__*q_total)/(j_max__ + a_max__));
}
void SCurvePlanner::calculateTimingParameters(double desired_time)
{
    double initial_v_max = v_max_;
    double initial_a_max = a_max_;
    double v_limit = 0.0;
    // 迭代参数调整逻辑（基于网页9、10的S曲线时间参数化原理）
    for(int iter=0; iter<100; iter++){
        v_limit = calculateVelocityLimit(q1_-q0_, a_max_, j_max_);
        v_max_ = fmin(v_max_, v_limit*1.05);
        // 加速段时间计算
        if((v_max_ - v0_)*j_max_ < a_max_*a_max_) {
            Tj1_ = sqrt((v_max_ - v0_)/j_max_);
            Ta_ = 2*Tj1_;
        } else {
            Tj1_ = a_max_/j_max_;
            Ta_ = Tj1_ + (v_max_ - v0_)/a_max_;
        }

        // 减速段时间计算
        if((v_max_ - v1_)*j_max_ < a_max_*a_max_) {
            Tj2_ = sqrt((v_max_ - v1_)/j_max_);
            Td_ = 2*Tj2_;
        } else {
            Tj2_ = a_max_/j_max_;
            Td_ = Tj2_ + (v_max_ - v1_)/a_max_;
        }

        // 计算匀速段时间（网页10的实现逻辑）
        Tv_ = (q1_ - q0_)/v_max_ - 0.5*Ta_*(1 + v0_/v_max_) 
            - 0.5*Td_*(1 + v1_/v_max_);
        
        // 时间调整策略（网页9的动态参数调整）
        if(Tv_ < -1e-6){
            Tv_ = 0;
            int count = 0;
            v_max_ = fmin(v_max_, calculateVelocityLimit(q1_-q0_, a_max_, j_max_));
            continue;
        }
        
        // 检查总时间收敛
        double current_time = Ta_ + Tv_ + Td_;
        if(fabs(current_time - desired_time) < 1e-3) break;
        
        // 调整策略
        if(current_time < desired_time){
            v_max_ = fmax(v_max_*0.95,(q1_-q0_)/desired_time);
            a_max_ = fmax(a_max_*0.95,initial_a_max*0.1);
        } else {
            v_max_ = fmin(initial_v_max, v_max_*1.05);
            a_max_ = fmin(initial_a_max,a_max_*1.05);
        }
        v_max_ = fmin(v_max_, initial_v_max);
        a_max_ = fmin(a_max_, initial_a_max);
    }
    total_time_ = Ta_ + Tv_ + Td_;
}

double SCurvePlanner::update(double dt)
{
    if(!initialized_) return v0_;
    
    t_ += dt;
    t_ = fmin(t_, total_time_);
    
    double t = t_;
    double v = v0_;

    // 七段式速度计算（网页10的实现逻辑）
    if(t < Tj1_) {
        // 段1：加加速
        v = v0_ + 0.5 * j_max_ * t*t;
    } 
    else if(t < Ta_ - Tj1_) {
        // 段2：匀加速
        double delta_t = t - Tj1_;
        v = v0_ + 0.5*j_max_*Tj1_*Tj1_ + a_max_*delta_t;
    }
    else if(t < Ta_) {
        // 段3：减加速
        double tau = t - (Ta_ - Tj1_);
        v = v_max_ - 0.5*j_max_*pow(Tj1_-tau,2);
    }
    else if(t < Ta_ + Tv_) {
        // 段4：匀速
        v = v_max_;
    }
    else if(t < total_time_ - Td_ + Tj2_) {
        // 段5：加减速
        double tau = t - (Ta_ + Tv_);
        v = v_max_ - 0.5*j_max_*tau*tau;
    }
    else if(t < total_time_ - Tj2_) {
        // 段6：匀减速
        double delta_t = t - (total_time_ - Td_ + Tj2_);
        v = v_max_ - 0.5*j_max_*Tj2_*Tj2_ - a_max_*delta_t;
    }
    else {
        // 段7：减减速
        double tau = t - (total_time_ - Tj2_);
        v = v1_ + 0.5*j_max_*pow(Tj2_-tau,2);
    }
    
    return v;
}
bool SCurvePlanner::arrived() {
    if (t_ >= total_time_) {
        return true;
    }
    return false;
}