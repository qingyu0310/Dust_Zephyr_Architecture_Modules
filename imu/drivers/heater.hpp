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
#include "stability.hpp"

#ifdef CONFIG_MOD_DEV_IMU_IDENT

#include "stability.hpp"
#include "uart.hpp"

namespace {
    static constexpr float kTargetTemp  = 40.0f;         // 目标温度
}

namespace ident {

enum class IdentStage : uint8_t {
    Cooldown    = 0,        // 等待温度降至目标并稳定
    Heating     = 1,        // 施加 duty 后立即记录温度响应
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
    static constexpr float  kDutySeq[] { 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f};

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
    stability::WinStable<100> stable_ {};               // 滑动窗口稳定判据

    void Reset();
    void CheckCmd();
    void ExecLoop(float temperature, Cmd cmd);
    void OpenLoop(float temperature, uint32_t dt_us, IdentStage& state, uint8_t& stage);
    void ClosedLoop(float temperature, uint32_t dt_us, IdentStage& state);
};

} // namespace ident

#endif // CONFIG_MOD_DEV_IMU_IDENT

namespace heater {

enum class Mode : uint8_t {
    Normal      = 0,    // PID 闭环控温
    AutoIdent   = 1,    // 自动辨识
};

class Heater final
{
public:
    static constexpr float   kSlopeLimit = 0.02f;           // 斜率限 (°C/s)
    static constexpr float   kNoiseLimit = 0.1f;            // 极差限 (°C)
    
    stability::MeanStable<100, 3> stable_ {};

    bool  Init();
    void  Update(float temperature);
    void  SetMode(Mode mode)     { mode_ = mode; }
    Mode  GetMode()        const { return mode_; }
    float GetDuty()        const { return duty_; }

private:

    Pwm           heater_pwm_ {};
    alg::pid::Pid pid_        {};

    Mode     mode_             = Mode::Normal;
    float    duty_             = 0.01f;
    bool     initialized_      = false;

#ifdef CONFIG_IMU_IDENTIFICATION
    ident::Identifier  ident_           {};
#endif
};

} // namespace heater
