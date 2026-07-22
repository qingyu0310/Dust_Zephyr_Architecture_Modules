/**
 * @file dji_c6xx.hpp
 * @author qingyu
 * @brief DJI C610 / C620 电机驱动 —— CAN 数据解析与状态回馈
 * @version 0.1
 * @date 2026-04-28
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "stdint.h"
#include <zephyr/sys/atomic.h>

namespace motor::dji {

/**
 * @brief DJI C610 电调（M2006 电机）
 *
 * 反馈帧（1KHz）:
 *   [0-1] 转子机械角度  (0~8191 → 0~360°)
 *   [2-3] 转子转速       (rpm, int16)
 *   [4-5] 实际输出转矩   (raw * 5/16384 → A)
 *   [6-7] Null
 */
class DjiC610 final
{
public:
    struct Config {
        uint16_t rx_id         = 0x201;
        float    gearbox_ratio = 3591.f / 187.f;
        float    wheel_r       = 0.1f;
        float    torque_k      = 0.18f;       // M2006 转矩常数 (N·m/A)
    };

    struct Snapshot {
        float radian, angle, omega, current, torque, velocity;
    };

    void Init(Config cfg) { cfg_ = cfg; }
    void CanCpltRxCallback(uint8_t* buffer);

    Snapshot ReadAll() const
    {
        atomic_t seq;
        Snapshot snap;
        do {
            seq = atomic_get(&seq_);
            if (seq & 1) continue;
            snap.radian      = now_rad_;
            snap.angle       = now_angle_;
            snap.omega       = now_omega_;
            snap.current     = now_current_;
            snap.torque      = now_torque_;
            snap.velocity    = now_velocity_;
        } while (atomic_get(&seq_) != seq);
        return snap;
    }

    float GetCurrentMax()     const { return kCurrentMax;   }
    float GetNowRadian()      const { return now_rad_;      }
    float GetNowAngle()       const { return now_angle_;    }
    float GetNowOmega()       const { return now_omega_;    }
    float GetNowCurrent()     const { return now_current_;  }
    float GetNowTorque()      const { return now_torque_;   }
    float GetNowVelocity()    const { return now_velocity_; }

private:
    static constexpr uint16_t kEncPerRound = 8192;
    static constexpr float    kCurrentK    = 10.0f / 16384.0f;
    static constexpr float    kCurrentMax  = 10.0f;

    Config cfg_ {};

    uint32_t last_enc_   = 0;
    int32_t  total_enc_  = 0;
    int32_t  total_round_ = 0;

    float now_rad_      = 0.0f;
    float now_angle_    = 0.0f;
    float now_omega_    = 0.0f;
    float now_current_  = 0.0f;
    float now_torque_   = 0.0f;
    float now_velocity_ = 0.0f;

    mutable atomic_t seq_ = 0;
};

/**
 * @brief DJI C620 电调（M3508 电机）
 *
 * 反馈帧（1KHz）:
 *   [0-1] 编码器         (16bit)
 *   [2-3] 转子转速       (rpm, int16)
 *   [4-5] 实际电流       (raw * 20/16384 → A)
 *   [6]   MOS 温度
 *   [7]   Null
 */
class DjiC620 final
{
public:
    struct Config {
        uint16_t rx_id         = 0x201;
        float    gearbox_ratio = 3591.f / 187.f;
        float    wheel_r       = 0.1f;
        float    torque_k      = 0.3f;       // M3508 转矩常数 (N·m/A)
    };

    struct Snapshot {
        float radian, angle, omega, current, torque, velocity, temperature;
    };

    void Init(Config cfg) { cfg_ = cfg; }
    void CanCpltRxCallback(uint8_t* buffer);

    Snapshot ReadAll() const
    {
        atomic_t seq;
        Snapshot snap;
        do {
            seq = atomic_get(&seq_);
            if (seq & 1) continue;
            snap.radian      = now_rad_;
            snap.angle       = now_angle_;
            snap.omega       = now_omega_;
            snap.current     = now_current_;
            snap.torque      = now_torque_;
            snap.velocity    = now_velocity_;
            snap.temperature = now_temp_;
        } while (atomic_get(&seq_) != seq);
        return snap;
    }
    float GetCurrentMax()     const { return kCurrentMax;   }
    float GetNowRadian()      const { return now_rad_;      }
    float GetNowAngle()       const { return now_angle_;    }
    float GetNowOmega()       const { return now_omega_;    }
    float GetNowCurrent()     const { return now_current_;  }
    float GetNowTorque()      const { return now_torque_;   }
    float GetNowVelocity()    const { return now_velocity_; }
    float GetNowTemperature() const { return now_temp_;     }

private:
    static constexpr uint16_t kEncPerRound = 8192;
    static constexpr float    kCurrentK    = 20.0f / 16384.0f;
    static constexpr float    kCurrentMax  = 20.0f;

    Config cfg_ {};

    uint32_t last_enc_   = 0;
    int32_t  total_enc_  = 0;
    int32_t  total_round_ = 0;

    float now_rad_      = 0.0f;
    float now_angle_    = 0.0f;
    float now_omega_    = 0.0f;
    float now_current_  = 0.0f;
    float now_torque_   = 0.0f;
    float now_velocity_ = 0.0f;
    float now_temp_     = 0.0f;

    mutable atomic_t seq_ = 0;
};

} // namespace motor::dji
