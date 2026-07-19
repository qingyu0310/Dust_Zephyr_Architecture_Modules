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
#include "uart.hpp"

#ifdef CONFIG_IMU_IDENTIFICATION

namespace ident {

enum class OpenStage : uint8_t {
    Cooldown    = 0,        // 等待温度降至目标并稳定
    Heating     = 1,        // 施加 duty 后立即记录温度响应
    SafetyStop  = 2,        // 超温停止
    Finished    = 3,        // 全部阶段完成
};

enum class Cmd : uint8_t {
    StartIdent  = 0,        // 开始辨识
    Stop        = 1,        // 立即停止
};

class Identifier
{
public:
    static constexpr float  kDutySeq[] { 0.2f, 0.3f, 0.4f, 0.5f, 0.6f, 0.7f};

    Identifier() {}
    bool     Init();
    float    GetDuty()   const { return duty_; }
    float    Perturb(float pid_duty, float temp_c);
    void     IdentOpenLoop(float temperature);

private:
    static constexpr uint32_t kWinSamples = 100;       // 滑动窗口帧数

    struct {
        float buf[kWinSamples] {};
        uint32_t head   = 0;
        uint32_t cnt    = 0;
        float prev_c    = 0.0f;
    } win_ {};

    UartDma     uart_ {};
    Cmd         active_cmd_ = Cmd::Stop;        // 当前执行的 PC 指令
    Cmd         prev_cmd_   = Cmd::Stop;
    uint32_t    last_cycle_ = 0;                // 上一帧 cycle 计数
    uint32_t    pert_cnt_   = 0;                // 扰动计数器（帧数）
    float       duty_       = 0.0f;             // 阶段占空比
    
    void  Reset();
    void  CheckCmd();
    void  ExecOpenLoop(float temp_c);
    void  ResetWindow();
    void  PushWin(float temp_c);
    bool  StableCheck(float temp_c, uint32_t dt_us);
};

} // namespace ident

#endif // CONFIG_IMU_IDENTIFICATION

namespace heater {

enum class Mode : uint8_t {
    Normal      = 0,    // PI 闭环控温
    OpenIdent   = 1,    // 开环辨识，内部驱动 Identifier
};

class Heater final
{
public:
    bool  Init();
    void  Update(float temperature);
    bool  IsStable(float temperature, float reference, float tolerance, uint32_t required, float dt = 0.001f);
    
    void  SetMode(Mode mode)     { mode_ = mode; }

    float GetDuty()        const { return duty_; }

private:
    static constexpr float   kTargetTemp     = 40.0f;       // 目标温度
    static constexpr float   kTempTolerance  = 0.2f;        // 温度偏差限 (°C)
    static constexpr float   kSlopeLimit     = 0.02f;       // 斜率限 (°C/s)
    static constexpr float   kNoiseLimit     = 0.1f;        // 极差限 (°C)

    Pwm           heater_pwm_ {};
    alg::pid::Pid pid_        {};

    Mode     mode_             = Mode::Normal;
    float    duty_             = 0.01f;
    float    prev_temp_        = 0.0f;
    uint32_t stable_count_     = 0;
    bool     initialized_      = false;

#ifdef CONFIG_IMU_IDENTIFICATION
    ident::Identifier  ident_           {};
#endif

    bool  StableSample(float temperature, float reference, float tolerance, float dt);
};

} // namespace heater
