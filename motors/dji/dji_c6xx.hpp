/**
 * @file dev_dji_c6xx.hpp
 * @author qingyu
 * @brief DJI C6xx 电机驱动 —— 仅作 CAN 数据解析与状态回馈
 *
 * @par 职责边界
 *      本类仅负责从 CAN 帧解析电机反馈数据（角度/角速度/电流/转速/温度），
 *      并通过 @ref ReadAll() 或单个 getter 提供给上层。
 *      @n **不包含** 任何控制算法（PID、功率限制等）。
 *      @n **不负责** CAN 帧发送（发送由专门的 CAN_TX 线程处理）。
 *
 * @par 线程安全
 *      - CAN 接收中断中调用 @ref CanCpltRxCallback() 写入数据
 *      - 控制线程通过 @ref ReadAll() 原子读取一致快照（seqlock 保护）
 *      - 单字段 getter 无锁（32-bit RISC-V 上 aligned float 访问天然原子）
 *
 * @par 数据流
 *      CAN 中断 ──→ CanCpltRxCallback ──→ (seqlock) ──→ ReadAll / getter
 * @version 0.1
 * @date 2026-04-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "stdint.h"
#include <zephyr/sys/atomic.h>

class DjiC6xx final
{
public:
    struct Config  {
        uint16_t rx_id      = 0x201;

        float gearbox_ratio = 3591.f / 187.f;
        float enc_per_round = 8192;
        float wheel_r       = 0.1f;
    };

    void Init(Config cfg) {
        cfg_ = cfg;
    };

    void CanCpltRxCallback(uint8_t* rx_data);

    /**
     * @brief 电机状态快照
     */
    struct Snapshot {
        float angle, omega, current, torque, velocity, temperature;
    };

    /** @brief 批量读——seqlock 保护，一次拿到所有值的一致快照 */
    Snapshot ReadAll() const
    {
        atomic_t seq;
        Snapshot snap;
        do {
            seq = atomic_get(&seq_);
            if (seq & 1) continue;
            snap.angle       = now_angle_;
            snap.omega       = now_omega_;
            snap.current     = now_current_;
            snap.torque      = now_torque_;
            snap.velocity    = now_velocity_;
            snap.temperature = now_temperature_;
        } while (atomic_get(&seq_) != seq);
        return snap;
    }

    /* 32-bit RISC-V 上 aligned float load/store 是原子的，单字段不需 seqlock */
    float GetNowAngle()       const { return now_angle_; }
    float GetNowOmega()       const { return now_omega_; }
    float GetNowCurrent()     const { return now_current_; }
    float GetNowTorque()      const { return now_torque_; }
    float GetNowVelocity()    const { return now_velocity_; }
    float GetNowTemperature() const { return now_temperature_; }

private:
    static constexpr float kTorqueK  = 0.3f;            // 转矩常数 N·m/A（手册数据）
    static constexpr float kCurrentK = 20.0f / 16384.0f; // 电流 AD 比例系数
    
    Config cfg_ {};

    uint32_t last_enc_     = 0;
    int32_t total_enc_     = 0;
    int32_t total_round_   = 0;

    float now_angle_       = 0.0f;
    float now_omega_       = 0.0f;
    float now_current_     = 0.0f;
    float now_torque_      = 0.0f;
    float now_velocity_    = 0.0f;
    float now_temperature_ = 0.0f;

    mutable atomic_t seq_ = 0;
};
