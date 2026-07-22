/**
 * @file dji_c6xx.cpp
 * @author qingyu
 * @brief DJI C610 / C620 电机驱动实现
 * @version 0.1
 * @date 2026-04-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "dji_c6xx.hpp"

#pragma message "Compiling Modules/Motors Dji"

namespace {
    static constexpr float kpi      = 3.141592653589793f;
    static constexpr float k2pi     = 6.283185307179586f;
    static constexpr float kRad2Deg = 57.2957795f;
}

namespace motor::dji {

/**
 * @brief CAN 接收回调，解析 C610 反馈帧
 * @param buffer 8 字节 CAN 数据
 */
void DjiC610::CanCpltRxCallback(uint8_t* buffer)
{
    const uint8_t* data       = buffer;
    const uint16_t angle_raw  = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    const int16_t  speed_rpm  = (static_cast<int16_t> (data[2]) << 8) | data[3];
    const int16_t  cur_raw    = (static_cast<int16_t> (data[4]) << 8) | data[5];

    if (last_enc_ != 0) {
        int32_t diff = static_cast<int32_t>(angle_raw) - static_cast<int32_t>(last_enc_);
        if (diff > 4096)
            total_round_ -= 1;
        else if (diff < -4096)
            total_round_ += 1;
    }
    last_enc_ = angle_raw;

    const int32_t total_enc   = total_round_ * static_cast<int32_t>(kEncPerRound)
                              + static_cast<int32_t>(angle_raw);
    const float   motor_angle = (static_cast<float>(total_enc) / static_cast<float>(kEncPerRound)) * k2pi;
    const float   omega_motor = (static_cast<float>(speed_rpm) * k2pi) / 60.0f;
    const float   current     = static_cast<float>(cur_raw) * kCurrentK;
    const float   torque      = current * cfg_.torque_k;
    const float   velocity    = (cfg_.wheel_r != 0.0f)
                              ? (omega_motor / cfg_.gearbox_ratio) * cfg_.wheel_r * 0.5f
                              : 0.0f;

    atomic_inc(&seq_);
    now_rad_      = (cfg_.gearbox_ratio != 0.0f) ? (motor_angle / cfg_.gearbox_ratio) : motor_angle;
    now_angle_    = now_rad_ * kRad2Deg;
    now_omega_    = (cfg_.gearbox_ratio != 0.0f) ? (omega_motor / cfg_.gearbox_ratio) : omega_motor;
    now_velocity_ = velocity;
    now_current_  = current;
    now_torque_   = torque;
    atomic_inc(&seq_);
}

/**
 * @brief CAN 接收回调，解析 C620 反馈帧
 * @param buffer 8 字节 CAN 数据
 */
void DjiC620::CanCpltRxCallback(uint8_t* buffer)
{
    const uint8_t* data       = buffer;
    const uint16_t enc        = (static_cast<uint16_t>(data[0]) << 8) | data[1];
    const int16_t  speed_rpm  = (static_cast<int16_t> (data[2]) << 8) | data[3];
    const int16_t  cur_raw    = (static_cast<int16_t> (data[4]) << 8) | data[5];
    const uint8_t  temp       = data[6];

    if (last_enc_ != 0) {
        int32_t diff = static_cast<int32_t>(enc) - static_cast<int32_t>(last_enc_);
        if (diff > static_cast<int32_t>(kEncPerRound) / 2)
            total_round_ -= 1;
        else if (diff < -static_cast<int32_t>(kEncPerRound) / 2)
            total_round_ += 1;
    }
    last_enc_ = enc;

    const int32_t total_enc   = total_round_ * static_cast<int32_t>(kEncPerRound)
                              + static_cast<int32_t>(enc);
    const float   motor_angle = (static_cast<float>(total_enc) / static_cast<float>(kEncPerRound)) * k2pi;
    const float   omega_motor = (static_cast<float>(speed_rpm) * k2pi) / 60.0f;
    const float   current     = static_cast<float>(cur_raw) * kCurrentK;
    const float   torque      = current * cfg_.torque_k;
    const float   velocity    = (cfg_.wheel_r != 0.0f)
                              ? (omega_motor / cfg_.gearbox_ratio) * cfg_.wheel_r * 0.5f
                              : 0.0f;

    atomic_inc(&seq_);
    now_rad_      = (cfg_.gearbox_ratio != 0.0f) ? (motor_angle / cfg_.gearbox_ratio) : motor_angle;
    now_angle_    = now_rad_ * kRad2Deg;
    now_omega_    = (cfg_.gearbox_ratio != 0.0f) ? (omega_motor / cfg_.gearbox_ratio) : omega_motor;
    now_velocity_ = velocity;
    now_current_  = current;
    now_torque_   = torque;
    now_temp_     = static_cast<float>(temp);
    atomic_inc(&seq_);
}

} // namespace motor::dji
