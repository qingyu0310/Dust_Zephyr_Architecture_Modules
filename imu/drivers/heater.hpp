/**
 * @file heater.hpp
 * @author qingyu
 * @brief IMU 加热控制模块 — PWM + PID 恒温
 * @version 0.3
 * @date 2026-10-23
 */

#pragma once

#include "pid.hpp"
#include "pwm.hpp"

namespace heater {

/**
 * @brief 加热器配置：目标温度、占空比范围与控温 PID
 */
struct Config
{
    float target_temp = 0.0f;			// 目标温度 (℃)
    float duty_min    = 0.0f;			// 占空比下限
    float duty_max    = 0.0f;			// 占空比上限
    alg::pid::Pid::Config pid {};		// 控温 PID
};

class Heater final
{
public:
    /**
     * @brief 初始化 PWM 与控温 PID
     * @param cfg 加热器配置
     * @return true=成功
     */
    bool  Init(const Config& cfg);

    void  Update(float temperature);
    float GetDuty() const { return duty_; }
    bool  Preheat(float temperature, float dt_s);
    void  Off();

    /**
     * @brief 控温 PID，供用户区调参
     * @return PID 引用
     */
    alg::pid::Pid& Pid() { return pid_; }

private:
    Pwm                  heater_pwm_ {};
    alg::pid::Pid        pid_        {};
    Config               cfg_        {};

    float    duty_        = 0.0f;
    bool     initialized_ = false;
};

} // namespace heater
