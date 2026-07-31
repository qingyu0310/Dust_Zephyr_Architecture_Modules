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

#ifdef CONFIG_MOD_DEV_IMU_IDENT

#include "stability.hpp"
#include "uart.hpp"

namespace ident {

enum class IdentStage : uint8_t {
    Cooldown    = 0,        // 等待温度降至基线
    Heating     = 1,        // 施加 duty 记录升温响应
    SafetyStop  = 2,        // 超温停止
    Finished    = 3,        // 全部阶段完成
};

enum class Cmd : uint8_t {
    OpenIdent   = 0,        // 开环辨识
    ClosedIdent = 1,        // 闭环辨识
    StopIdent   = 2,        // 立即停止
};

class Identifier
{
public:
    static constexpr float  kDutySeq[] {0.20f, 0.25f, 0.30f, 0.35f, 0.40f, 0.45f, 0.50f};

    bool     Init();
    float    GetDuty()   const { return duty_; }
    void     IdentLoop(float temperature);

private:
    UartDma         uart_       {};
    Cmd             active_cmd_ = Cmd::StopIdent;       // 当前执行的 PC 指令
    Cmd             prev_cmd_   = Cmd::StopIdent;
    uint32_t        last_cycle_ = 0;                    // 上一帧 cycle 计数
    float           duty_       = 0.0f;                 // 阶段占空比
    alg::pid::Pid   pid_        {};
    stability::WinStable<200> stable_ {};               // 滑动窗口稳定判据

    void Reset();
    void CheckCmd();
    void ExecLoop(float temperature, Cmd cmd);
    void OpenLoop(float temperature, uint32_t dt_us, IdentStage& state, uint8_t& stage);
    void ClosedLoop(float temperature, uint32_t dt_us, IdentStage& state);
};

} // namespace ident

#endif // CONFIG_MOD_DEV_IMU_IDENT

namespace heater {

class Heater final
{
public:
    bool  Init();
    void  Update(float temperature);
    float GetDuty()        const { return duty_; }
    bool  Preheat(float temperature, float dt_s);

private:
    enum class Mode : uint8_t {
        ClosedLoop = 0,
        AutoIdent,
    };

    Pwm           heater_pwm_ {};
    alg::pid::Pid pid_        {};

    Mode     mode_             = Mode::ClosedLoop;
    float    duty_             = 0.01f;
    bool     initialized_      = false;

#ifdef CONFIG_IMU_IDENTIFICATION
    ident::Identifier  ident_           {};
#endif
};

} // namespace heater
