/**
 * @file heater.cpp
 * @author qingyu
 * @brief 加热器 PWM 初始化、PID 控温与稳态判据
 * @version 0.1
 * @date 2026-07-19
 */

#include "heater.hpp"
#include <cmath>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>

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

    if (!heater_pwm_.init(heater_pwm)) {
        return false;
    }

    constexpr alg::pid::Pid::Config kDefaultPidConfig {
        .kp         = 0.13f,
        .ki         = 0.03f,
        .kd         = 0.0f,
        .iOutMax    = 0.5f,
        .outMax     = kMaxDuty,
        .dt         = 0.001f,
    };

    pid_.Init(kDefaultPidConfig);

    duty_ = 0.0f;
    initialized_ = true;
    return heater_pwm_.SetDuty(duty_);
}

/**
 * @brief 更新 PID 控制器并设置 PWM 占空比
 *
 * 根据当前温度与目标温度的偏差计算 PID 输出，
 * 输出钳位到 [1 - kMaxDuty, kMaxDuty] 范围。
 *
 * @param temperature 当前 IMU 温度，单位 °C
 */
void Heater::Update(float temperature)
{
    if (!initialized_) {
        return;
    }

    duty_ = pid_.Calc(kTargetTemp, temperature);
    if (duty_ < 1.f - kMaxDuty) {
        duty_ = 1.f - kMaxDuty;
    }
    if (duty_ > kMaxDuty) {
        duty_ = kMaxDuty;
    }
    (void)heater_pwm_.SetDuty(duty_);
}

/**
 * @brief 单帧稳定判据
 *
 * 同时检查温度偏差与瞬时斜率：
 * - |temp - ref| > tolerance → 不稳定，重置上一帧温度
 * - 首帧跳过斜率判断
 * - |斜率| > kSlopeLimit → 不稳定
 *
 * @param temperature 当前温度
 * @param reference   参考温度
 * @param tolerance   允许偏差
 * @param dt          采样间隔，用于斜率计算
 * @return true  单帧满足偏差和斜率条件
 * @return false 偏差或斜率超限
 */
bool Heater::StableSample(float temperature, float reference, float tolerance, float dt)
{
    if (std::abs(temperature - reference) > tolerance) {
        prev_temp_ = temperature;
        return false;
    }

    // 首帧跳过斜率判断
    if (prev_temp_ == 0.0f) {
        prev_temp_ = temperature;
        return false;
    }

    const float slope = (temperature - prev_temp_) / dt;
    prev_temp_ = temperature;

    return std::abs(slope) <= kSlopeLimit;
}

/**
 * @brief 连续稳定判据
 *
 * 内部维护连续 StableSample 通过计数，
 * 达到 required 帧即认为系统温度稳定。
 *
 * @param temperature 当前温度
 * @param reference   参考温度
 * @param tolerance   允许偏差
 * @param required    连续稳定的最少帧数
 * @param dt          采样间隔，传递给 StableSample
 * @return true  连续稳定帧数 >= required
 * @return false 未达到要求
 */
bool Heater::IsStable(float temperature, float reference, float tolerance, uint32_t required, float dt)
{
    if (StableSample(temperature, reference, tolerance, dt)) {
        stable_count_++;
    } else {
        stable_count_ = 0;
    }
    return stable_count_ >= required;
}

} // namespace heater
