/**
 * @file heater.cpp
 * @author qingyu
 * @brief 加热器 PWM 初始化、PID 控温与稳态判据
 * @version 0.3
 * @date 2026-10-23
 */

#include "heater.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include "log.hpp"
#include "stability.hpp"


namespace {
    static constexpr float kMaxDuty     = 0.95f;
    static constexpr float kMinDuty     = 0.0f;
    static constexpr float kTargetTemp  = 40.0f;

    static constexpr alg::pid::Pid::Config kPidConfig {
        .kp         = 0.19f, 
        .ki         = 0.03f, 
        .kd         = 0.0005f,
        .iOutMax    = 0.5f, 
        .outMax     = kMaxDuty, 
        .dt         = 0.001f,
    };
};


#pragma message "Compiling Modules/Imu/Drivers/Heater"

namespace heater {

/**
 * @brief 初始化 PWM 加热器与 PID 控制器
 *
 * 从 devicetree 获取 IMU 加热 PWM 实例，
 * 初始化 PID 控制器并设置初始占空比为 0。
 *
 * @return true  初始化成功
 * @return false PWM 初始化失败
 */
bool Heater::Init()
{
    static const pwm_dt_spec heater_pwm = PWM_DT_SPEC_GET(DT_ALIAS(imu_pwm));
    if (!heater_pwm_.init(heater_pwm)) return false;

    pid_.Init(kPidConfig);

    duty_ = kMinDuty;
    initialized_ = true;

    return heater_pwm_.SetDuty(duty_);
}

/**
 * @brief 更新 PWM 占空比
 *
 * PID 闭环控温，输出限幅后写入 PWM。
 *
 * @param temperature 当前 IMU 温度 (°C)
 */
void Heater::Update(float temperature)
{
    if (!initialized_) return;

    duty_ = pid_.Calc(kTargetTemp, temperature);

    duty_ = std::clamp(duty_, kMinDuty, kMaxDuty);
    (void)heater_pwm_.SetDuty(duty_);
}

/**
 * @brief 关闭加热输出
 *
 * 线程退出时调用：PID 积分与当前 duty 都清零，PWM 拉到 0。
 */
void Heater::Off()
{
    duty_ = kMinDuty;
    pid_.SetIntegralError(0.0f);
    (void)heater_pwm_.Stop();
}

/**
 * @brief 预热 — 更新 PID 并判断温度是否达到稳态
 * @param temperature 当前温度 (°C)
 * @param dt_s        采样间隔 (s)
 * @return true       温度稳定
 */
bool Heater::Preheat(float temperature, float dt_s)
{
    static stability::WinStable<200> *stable = nullptr;

    if (stable == nullptr) {
        stable = static_cast<stability::WinStable<200>*>(calloc(1, sizeof(stability::WinStable<200>)));
    }
    constexpr float kSlopeLimit = 2.0f;     // 允许最大斜率 (°C/s)
    constexpr float kNoiseLimit = 0.5f;     // 允许最大峰峰 (°C)
    constexpr float kDriftLimit = 0.1f;     // 允许窗口净漂移 (°C)，区分振荡与单向下漂
    constexpr float kTargetTol  = 0.5f;     // 目标温度容差 (°C)

    Update(temperature);
    const bool near_target = std::abs(temperature - kTargetTemp) <= kTargetTol;
    const bool stable_ok   = stable->Check(temperature, dt_s, kSlopeLimit, kNoiseLimit, kDriftLimit) && near_target;

    if (stable_ok) {
        DUST_LOG_INF("preheat stable");
        free(stable);
        stable = nullptr;
    }

    return stable_ok;
}

} // namespace heater
