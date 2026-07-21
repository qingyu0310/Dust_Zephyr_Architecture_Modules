/**
 * @file dm.hpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-05-14
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cstdint>
#include <zephyr/sys/atomic.h>

enum class DmErrorStatus : uint8_t
{
    Disable        = 0x0,       // 失能
    Enable         = 0x1,       // 使能
    EncoderUncalib = 0x2,       // 输出轴编码器未校准
    OverVoltage    = 0x8,       // 超压
    UnderVoltage   = 0x9,       // 欠压
    OverCurrent    = 0xA,       // 过电流
    MosOvertemp    = 0xB,       // MOS 过温
    CoilOvertemp   = 0xC,       // 电机线圈过温
    CommLost       = 0xD,       // 通讯丢失
    Overload       = 0xE,       // 过载
};

enum class ControlMethon : uint8_t
{
    Mit = 0,                    // MIT控制模式
    Pos,                        // 位置速度控制模式
    Spd,                        // 速度控制模式
    Psi,                        // 力位控制模式
};

/**
 * @brief DM 达妙电机驱动
 *
 * ## MIT 模式用法说明
 *
 * 1. kp=0, kd≠0 — 给定 v_des 实现匀速转动，存在静差，kd 过大会引起震荡
 * 2. kp=0, kd=0 — 给定 t_ff 实现恒力矩输出；空转/轻载时若 t_ff 过大会持续加速
 * 3. kp≠0, kd=0 — 位置控制时 kd 不能赋 0，否则震荡甚至失控
 * 4. kp≠0, kd≠0 — 具体看官方文档
 */
class DmMotor final
{
public:
    static constexpr float kRad2Deg = 57.2957795f;          // 180° / π

    struct Config  {
        ControlMethon ctrl_met  = ControlMethon::Mit;

        uint16_t can_id     = 0x01;             // 发送给电机的控制帧 ID（TX）
        uint16_t master_id  = 0x00;             // 电机反馈帧的 ID（RX），建议 ≠ can_id

        float gearbox_ratio = 10.f;             // 电机减速比 (J4310)
        float wheel_r       = 0.1f;             // 车轮半径 (m)

        // 无 vel_kp/vel_kd——DM 协议只有位置环 Kp/Kd，无独立速度环系数
        float kp            = 0.0f;             // 位置环比例系数 [0, 500]
        float kd            = 0.0f;             // 位置环微分系数 [0, 5]

        float PMAX          = 12.5f;            // 位置量程 ±12.5 rad
        float VMAX          = 45.0f;            // 速度量程 ±45 rad/s
        float TMAX          = 18.0f;            // 转矩量程 ±18 N*m
    };

    /**
     * @brief 电机状态快照
     */
    struct Snapshot {
        float radian, angle, velocity, omega, torque, tmos, tcoil;
        DmErrorStatus err;
    };

    enum class Cmd : uint8_t {
        ClearErr = 0xFB,
        Enable   = 0xFC,
        Disable  = 0xFD,
        SaveZero = 0xFE,
    };

    // 控制模式值（写寄存器 0x0A）
    enum class CtrlModeVal : uint32_t {
        Mit = 1,
        Pos = 2,
        Spd = 3,
        Psi = 4,
    };

    void Init(Config cfg);

    void PwrLossCheck();

    void CanCpltRxCallback(uint8_t* buffer);

    /**
     * @brief 批量读——seqlock 保护，一次拿到所有值的一致快照
     */
    Snapshot ReadAll() const
    {
        atomic_t seq;
        Snapshot snap;
        do {
            seq = atomic_get(&seq_);
            if (seq & 1) continue;
            snap.radian   = now_rad_;
            snap.angle    = now_ang_;
            snap.velocity = now_vel_;
            snap.omega    = now_omg_;
            snap.torque   = now_tor_;
            snap.tmos     = now_tmos_;
            snap.tcoil    = now_tcoil_;
            snap.err      = now_err_;
        } while (atomic_get(&seq_) != seq);
        return snap;
    }

    void PackCtrlFrame(uint8_t (&data)[8]);
    void PackCmdFrame (uint8_t (&data)[8], Cmd cmd);
    void PackSetCtrlMode(uint8_t (&data)[8], CtrlModeVal mode);

    // 读取当前状态
    float GetNowRadian()      const { return now_rad_;    }         // 电机轴位置 (rad)
    float GetNowAngle()       const { return now_ang_;    }         // 电机轴角度 (°)
    float GetNowVelocity()    const { return now_vel_;    }         // 线速度 (m/s)
    float GetNowOmega()       const { return now_omg_;    }         // 当前角速度 (rad/s)
    float GetNowTorque()      const { return now_tor_;    }         // 转矩 (N*m)
    float GetNowTmos()        const { return now_tmos_;   }         // MOSFET 温度 (°C)
    float GetNowTcoil()       const { return now_tcoil_;  }         // 线圈温度 (°C)
    uint16_t GetTxId()        const { return cfg_.can_id; }         // 控制帧CAN ID
    DmErrorStatus GetNowErr() const { return now_err_;    }         // 电机故障状态

    // 读取目标值
    float GetTargetRadian()   const { return ctrl.target_radian_;   }   // rad
    float GetTargetAngle()    const { return ctrl.target_angle_;    }   // °
    float GetTargetVelocity() const { return ctrl.target_velocity_; }   // m/s（线速度）
    float GetTargetOmega()    const { return ctrl.target_omega_;    }   // rad/s（角速度）
    float GetTargetTorque()   const { return ctrl.target_torque_;   }   // N*m

    // 设定目标值
    // CtrlData 只认 rad / rad/s，其他单位自动转成协议值后再赋给 target_radian_ / target_omega_
    // Init() 之后调用（依赖 cfg_ 的 wheel_r / gearbox_ratio）
    void SetTargetRadian(float v)   { ctrl.target_radian_ = v; ctrl.target_angle_ = v * kRad2Deg; }
    void SetTargetAngle(float v)    { ctrl.target_angle_ = v; ctrl.target_radian_ = v / kRad2Deg; }

    void SetTargetVelocity(float v) {
        ctrl.target_velocity_ = v;
        if (cfg_.wheel_r != 0.0f)
            ctrl.target_omega_ = v / (cfg_.wheel_r * 0.5f) * cfg_.gearbox_ratio;
    }
    void SetTargetOmega(float v) {
        ctrl.target_omega_ = v;
        if (cfg_.wheel_r != 0.0f)
            ctrl.target_velocity_ = v * cfg_.wheel_r * 0.5f / cfg_.gearbox_ratio;
    }

    void SetTargetTorque(float v)   { ctrl.target_torque_ = v; }               // N*m

private:
    Config cfg_ {};

    float now_rad_          = 0.0f;         // 电机轴位置 (rad)
    float now_ang_          = 0.0f;         // 电机轴角度 (°)
    float now_vel_          = 0.0f;         // 当前线速度 (m/s)
    float now_omg_          = 0.0f;         // 当前角速度 (rad/s)
    float now_tor_          = 0.0f;         // 当前转矩 (N*m)
    float now_tmos_         = 0.0f;         // MOSFET 温度 (°C)
    float now_tcoil_        = 0.0f;         // 线圈温度 (°C)

    uint16_t pre_encoder_   = 0;
    int32_t  total_round_   = 0;
    int32_t  total_encoder_ = 0;

    uint32_t flag_          = 0;            // 接收计数（每帧+1）
    uint32_t pre_flag_      = 0;            // 上次检测的快照
    bool     power_lost_    = true;         // 断电标志

    struct
    {
        float target_radian_   = 0.0f;      // rad（控制协议所需的发送数据）
        float target_angle_    = 0.0f;      // ° = rad × kRad2Deg
        float target_velocity_ = 0.0f;      // m/s = omg × wheel_r × 0.5 / gearbox_ratio
        float target_omega_    = 0.0f;      // rad/s（控制协议所需的发送数据）
        float target_torque_   = 0.0f;      // N*m
    } ctrl;

    DmErrorStatus now_err_ = DmErrorStatus::Disable;    // 电机状态

    mutable atomic_t seq_   = 0;
};




























