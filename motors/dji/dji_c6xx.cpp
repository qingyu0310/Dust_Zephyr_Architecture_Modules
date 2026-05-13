/**
 * @file dev_dji_c6xx.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-04-28
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "dji_c6xx.hpp"

namespace {
    static constexpr float kpi  = 3.141592653589793f;
    static constexpr float k2pi = 6.283185307179586f;
} 

void DjiC6xx::CanCpltRxCallback(uint8_t* buffer)
{
    const uint8_t* data         = buffer;
    const uint16_t enc          = (static_cast<uint16_t>(data[0]) << 8)                      | static_cast<uint16_t>(data[1]);
    const int16_t  omega_rpm    = static_cast<int16_t>((static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3]));
    const int16_t  current_raw  = static_cast<int16_t>((static_cast<uint16_t>(data[4]) << 8) | static_cast<uint16_t>(data[5]));
    const uint8_t  temp         = data[6];

    if (last_enc_ != 0) {
        int32_t diff = static_cast<int32_t>(enc) - static_cast<int32_t>(last_enc_);
        if (diff > 4096) {
            total_round_ -= 1;
        } else if (diff < -4096) {
            total_round_ += 1;
        }
    }
    last_enc_ = enc;

    const int32_t total_enc = total_round_ * static_cast<int32_t>(cfg_.enc_per_round) + static_cast<int32_t>(enc);
    const float motor_angle = (static_cast<float>(total_enc) / static_cast<float>(cfg_.enc_per_round)) * k2pi;
    const float omega_motor = (static_cast<float>(omega_rpm) * k2pi) / 60.0f;
    const float velocity    = (cfg_.wheel_r != 0.0f) ? (omega_motor / cfg_.gearbox_ratio) * cfg_.wheel_r * 0.5f : 0.0f;
    const float current     = static_cast<float>(current_raw) * kCurrentK;
    const float torque      = current * kTorqueK;
    const float temp_float  = static_cast<float>(temp);

    /* seqlock 写锁：允许其他中断，线程读到冲突时会自旋重试 */
    atomic_inc(&seq_);
    now_angle_       = (cfg_.gearbox_ratio != 0.0f) ? (motor_angle / cfg_.gearbox_ratio) : motor_angle;
    now_omega_       = (cfg_.gearbox_ratio != 0.0f) ? (omega_motor / cfg_.gearbox_ratio) : omega_motor;
    now_velocity_    = velocity;
    now_current_     = current;
    now_torque_      = torque;
    now_temperature_ = temp_float;
    atomic_inc(&seq_);
}










