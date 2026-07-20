/**
 * @file heater.cpp
 * @author qingyu
 * @brief 加热器 PWM 初始化、PID 控温与稳态判据
 * @version 0.1
 * @date 2026-07-19
 */

#include "heater.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_IMU_IDENTIFICATION

#pragma message "Compiling Modules/Imu/Drivers/Ident"

LOG_MODULE_REGISTER(ident, LOG_LEVEL_INF);

namespace {
    static constexpr float kMaxDuty = 0.95f;
    static constexpr float kMinDuty = 0.001f;        // PWM最小不能为0
    static constexpr float kBaseC   = 38.0f;
    static constexpr float kBaseTol = 0.10f;

    static constexpr alg::pid::Pid::Config kPidConfig {
        .kp = 0.22f, 
        .ki = 0.05f, 
        .kd = 0.005f,
        .iOutMax = 0.5f, 
        .outMax = kMaxDuty, 
        .dt = 0.001f,
    };
};

namespace ident {

/**
 * @brief 辨识主循环（公共部分）
 *
 * 每帧测量时间、检查超温、处理 Finished→Cooldown 复位，
 * 然后根据 cmd 分发到 OpenLoop / ClosedLoop。
 *
 * @param temperature 当前温度 (°C)
 * @param cmd         PC 下发的指令
 */
void Identifier::ExecLoop(float temperature, Cmd cmd)
{
    constexpr float     kOverTempC  = 80.0f;

    static IdentStage   state       = IdentStage::Finished;
    static uint8_t      stage       = 0;
    static uint32_t     seq         = 0;

    const uint32_t      now         =  k_cycle_get_32();
    const uint64_t      t_us        =  k_cyc_to_ns_floor64(now) / 1000;
    const uint32_t      dt_us       =  (last_cycle_ == 0) ? 1000 : (uint32_t)(k_cyc_to_ns_floor64(now - last_cycle_) / 1000);

    if (temperature > kOverTempC) {
        duty_ = kMinDuty;
        state = IdentStage::SafetyStop;
        LOG_INF("Safety Stop");
        return;
    }

    if (state == IdentStage::Finished) {
        state = IdentStage::Cooldown;
        stage = 0;
        seq   = 0;
        Reset();
    }

    LOG_INF("seq=%u,t_us=%llu,dt_us=%u,stage=%u,state=%u,temp_c=%.3f,duty=%.3f",
               (unsigned)++seq, (unsigned long long)t_us, (unsigned)dt_us,
               (unsigned)stage, (unsigned)state, (double)temperature, (double)duty_);

    if (cmd == Cmd::OpenIdent) {
        OpenLoop(temperature, dt_us, state, stage);
    } 
    else if (cmd == Cmd::ClosedIdent) {
        ClosedLoop(temperature, dt_us, state);
    }

    last_cycle_ = now;
}

/**
 * @brief 开环辨识状态机
 *
 * Cooldown → 温度降到基线后开始阶梯加热；
 * Heating → 每级 duty 稳定后跳下一级，全部完成后置 Finished。
 *
 * @param temperature 当前温度 (°C)
 * @param dt_us       与上一帧的时间间隔 (µs)
 * @param state       状态（由 ExecLoop 维护）
 * @param stage       当前阶梯序号
 */
void Identifier::OpenLoop(float temperature, uint32_t dt_us, IdentStage& state, uint8_t& stage)
{
    constexpr uint16_t kNumStages = sizeof(kDutySeq) / sizeof(kDutySeq[0]);

    switch (state) 
    {
        case IdentStage::Cooldown: 
        {
            if (temperature < kBaseC || (temperature <= kBaseC + kBaseTol && stable_.Check(temperature, dt_us * 1e-6f))) {
                state = IdentStage::Heating;
                duty_ = kDutySeq[0];
                stable_.Reset();
                LOG_INF("Cooldown Done");
            }
        break;
    }
    case IdentStage::Heating: 
    {
        if (stable_.Check(temperature, dt_us * 1e-6f)) 
        {
            if (++stage < kNumStages) {
                duty_ = kDutySeq[stage];
                stable_.Reset();
                LOG_INF("Stage Done");
            }
            else {
                duty_ = kMinDuty;
                state = IdentStage::Finished;
                LOG_INF("Finished");
            }
        }
        break;
    }
    default:
        duty_ = kMinDuty;
        break;
    }
}

/**
 * @brief 闭环辨识状态机
 *
 * Cooldown → 温度降到基线后开始 PID 控温；
 * Heating → 持续 PID 控温 + 数据采集，不自动切换阶段。
 * 需要由 PC 下发 Stop 指令停止。
 *
 * @param temperature 当前温度 (°C)
 * @param dt_us       与上一帧的时间间隔 (µs)
 * @param state       状态（由 ExecLoop 维护）
 */
void Identifier::ClosedLoop(float temperature, uint32_t dt_us, IdentStage& state)
{
    switch (state)
    {
        case IdentStage::Cooldown: 
        {
            if (temperature < kBaseC || (temperature <= kBaseC + kBaseTol && stable_.Check(temperature, dt_us * 1e-6f))) {
                state = IdentStage::Heating;
                stable_.Reset();
                LOG_INF("Cooldown Done");
            }
            break;
        }
        case IdentStage::Heating:
        {
            duty_ = pid_.Calc(kTargetTemp, temperature);
            duty_ = std::clamp(duty_, kMinDuty, kMaxDuty);
            break;
        }
        default:
            duty_ = kMinDuty;
            break;
    }
}

/**
 * @brief 读取 UART 指令并更新 active_cmd_
 *
 * 非阻塞读取 UART 缓冲，匹配命令表后更新当前指令。
 * 不匹配的输入静默丢弃。
 */
void Identifier::CheckCmd()
{
    struct CmdEntry {
        const char* name;
        Cmd id;
    }  constexpr kCmds[]  {                                       // PC下发指令
        { "OpenIdent",   Cmd::OpenIdent    },
        { "ClosedIdent", Cmd::ClosedIdent  },
        { "Stop",        Cmd::StopIdent    },
    };

    uint8_t buf[16] {};
    if (uart_.Read(buf, sizeof(buf) - 1) == 0)
        return;

    for (const auto& e : kCmds)
    {
        if (strcmp((const char*)buf, e.name) == 0) {
            active_cmd_ = e.id;
            break;
        }
    }
}

/**
 * @brief 初始化辨识器（UART 接收）
 * @return true  初始化成功
 */
bool Identifier::Init()
{
    RxStream::Config cfg {};
    cfg.buf_size   = 256;
    cfg.rx_timeout = 1000;

    pid_.Init(kPidConfig);

    if (!uart_.Init(DEVICE_DT_GET(DT_NODELABEL(uart3)), cfg)) {
        LOG_ERR("uart3 init failed");
        return false;
    }
    return true;
}

/**
 * @brief 复位辨识器内部状态
 */
void Identifier::Reset()
{
    duty_       = kMinDuty;
    last_cycle_ = 0;
    stable_.Reset();
}

/**
 * @brief 开环辨识主入口（每帧调用）
 *
 * 读取 UART 指令，分发到停止或状态机更新。
 *
 * @param temperature 当前温度 (°C)
 */
void Identifier::IdentLoop(float temperature)
{
    CheckCmd();

    if (active_cmd_ == Cmd::StopIdent) {
        duty_ = kMinDuty;
        return;
    }

    if (prev_cmd_ == Cmd::StopIdent && active_cmd_ != Cmd::StopIdent) {
        LOG_INF("Ident Starting");
    }
    prev_cmd_ = active_cmd_;

    ExecLoop(temperature, active_cmd_);
}

} // namespace ident

#endif // CONFIG_IMU_IDENTIFICATION

#pragma message "Compiling Modules/Imu/Drivers/Heater"

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
    if (!heater_pwm_.init(heater_pwm)) return false;

    pid_.Init(kPidConfig);

    duty_ = kMinDuty;
    initialized_ = true;

#ifdef CONFIG_IMU_IDENTIFICATION
    if (!ident_.Init()) {
        return false;
    }
#endif

    return heater_pwm_.SetDuty(duty_);
}

/**
 * @brief 更新 PWM 占空比
 *
 * Normal: PID 闭环控温
 * Ident: 驱动辨识状态机，输出辨识器计算的 duty + 日志
 *
 * @param temperature 当前 IMU 温度 (°C)
 */
void Heater::Update(float temperature)
{
    if (!initialized_) return;

    if (mode_ == Mode::Normal)
    {
        duty_ = pid_.Calc(kTargetTemp, temperature);
    }
#ifdef CONFIG_IMU_IDENTIFICATION
    else if (mode_ == Mode::AutoIdent)
    {
        ident_.IdentLoop(temperature);
        duty_ = ident_.GetDuty();
    }
#endif // CONFIG_IMU_IDENTIFICATION

    duty_ = std::clamp(duty_, kMinDuty, kMaxDuty);
    (void)heater_pwm_.SetDuty(duty_);
}

} // namespace heater
