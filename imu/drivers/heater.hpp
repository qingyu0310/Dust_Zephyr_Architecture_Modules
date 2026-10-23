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

class Heater final
{
public:
    bool  Init();
    void  Update(float temperature);
    float GetDuty()        const { return duty_; }
    bool  Preheat(float temperature, float dt_s);
    void  Off();

private:
    Pwm           heater_pwm_ {};
    alg::pid::Pid pid_        {};

    float    duty_             = 0.01f;
    bool     initialized_      = false;
};

} // namespace heater
