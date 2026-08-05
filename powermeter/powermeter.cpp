/**
 * @file horizon.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-04-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "powermeter.hpp"

#pragma message "Compiling Modules/Powermeter"

void PowerMeter::CanCpltRxCallback(uint8_t* buffer)
{
	// 标定系数：真值 = gain × 码值换算值 + offset（2026-08-04 实测标定）
 	constexpr float kCurrentGain   = 0.9742f;   // 电流增益
	constexpr float kCurrentOffset = 0.0036f;   // 电流零偏（A）

    frame_count_++;                          // 有效帧计数（HasFrame 用）

    const uint8_t* data         = buffer;
    const uint16_t shunt_raw    = (static_cast<int16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
    const uint16_t bus_raw      = (static_cast<int16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3]);
    const uint16_t current_raw  = (static_cast<int16_t>(data[4]) << 8) | static_cast<uint16_t>(data[5]);
    const uint16_t power_raw    = (static_cast<int16_t>(data[6]) << 8) | static_cast<uint16_t>(data[7]);

    /* seqlock 写锁：允许其他中断，线程读到冲突时会自旋重试 */
    atomic_inc(&seq_);
    shunt_volt = static_cast<float>(shunt_raw)   * 0.001f;  // V
    bus_volt   = static_cast<float>(bus_raw)     * 0.001f;  // V
    current    = static_cast<float>(current_raw) * 0.001f * kCurrentGain + kCurrentOffset;  // A
    power      = static_cast<float>(power_raw)   * 0.01f;   // W
    atomic_inc(&seq_);
}