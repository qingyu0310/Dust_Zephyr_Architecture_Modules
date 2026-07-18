/**
 * @file heater.hpp
 * @author qingyu
 * @brief IMU 加热控制模块 — PWM + PID 恒温
 * @version 0.1
 * @date 2026-07-19
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
    bool  IsStable(float temperature, float reference, float tolerance, uint32_t required, float dt = 0.001f);
    float GetDuty() const { return duty_; }

private:
    static constexpr float   kMaxDuty        = 0.95f;
    static constexpr float   kMinDuty        = 0.001f;      // PWM最小不能为0
    static constexpr float   kTargetTemp     = 40.0f;

    // 稳定判据参数
    static constexpr float   kTempTolerance  = 0.2f;        // 温度偏差限 (°C)
    static constexpr float   kSlopeLimit     = 0.02f;       // 斜率限 (°C/s)
    static constexpr float   kNoiseLimit     = 0.1f;        // 极差限 (°C)

    Pwm           heater_pwm_ {};
    alg::pid::Pid pid_        {};
    
    float    duty_         = 0.01f;
    float    prev_temp_    = 0.0f;
    uint32_t stable_count_ = 0;
    bool     initialized_  = false;

    bool  StableSample(float temperature, float reference, float tolerance, float dt);
};

} // namespace heater
