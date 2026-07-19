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
#include <cstdint>
#include <cstring>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

#pragma message "Compiling Modules/Imu/Drivers/Heater"

#ifdef CONFIG_IMU_IDENTIFICATION

#pragma message "Compiling Modules/Imu/Drivers/Ident"

LOG_MODULE_REGISTER(ident, LOG_LEVEL_INF);

namespace {
    static constexpr float kMaxDuty = 0.95f;
    static constexpr float kMinDuty = 0.001f;      // PWM最小不能为0
};

namespace ident {

/**
 * @brief 主更新函数，每帧调用
 *
 * 推进辨识状态机：Cooldown → Heating → Finished。
 * 超温时直接切 SafetyStop。
 * 状态切换时输出事件行，每帧输出采样行供 PC 采集。
 *
 * @param temp_c  当前温度 (°C)
 * @param t_us    当前 MCU 时间 (µs)
 * @param dt_us   与上一帧的时间间隔 (µs)
 * @return Event  当前帧的事件（None/StepStart/StageEnd/Finished/SafetyStop）
 */
void Identifier::ExecOpenLoop(float temp_c)
{
    constexpr float     kOverTempC  = 80.0f;
    constexpr uint16_t  kNumStages  = sizeof(kDutySeq) / sizeof(kDutySeq[0]);

    static OpenStage    state       = OpenStage::Finished;    // 开环状态机阶段
    static uint8_t      stage       = 0;                      // 当前阶梯序号
    static uint32_t     seq         = 0;                      // 采样帧序号

    const uint32_t      now         =  k_cycle_get_32();
    const uint64_t      t_us        =  k_cyc_to_ns_floor64(now) / 1000;
    const uint32_t      dt_us       =  (last_cycle_ == 0) ? 1000 : (uint32_t)(k_cyc_to_ns_floor64(now - last_cycle_) / 1000);

    if (temp_c > kOverTempC) {
        duty_ = kMinDuty;
        state = OpenStage::SafetyStop;
        LOG_INF("Safety Stop");
        return;
    }

    if (state == OpenStage::Finished)
    {
        state           = OpenStage::Cooldown;
        stage           = 0;
        seq             = 0;
        Reset();
    }

    LOG_INF("seq=%u,t_us=%llu,dt_us=%u,stage=%u,state=%u,temp_c=%.3f,duty=%.3f",
               (unsigned)++seq, (unsigned long long)t_us, (unsigned)dt_us,
               (unsigned)stage, (unsigned)state, (double)temp_c, (double)duty_);

    switch (state)
    {
        case OpenStage::Cooldown:
        {
            constexpr float kBaselineTempC  = 38.0f;            // 基线温度阈值 (°C)
            constexpr float kBaselineTol    = 0.2f;             // 基线温度容限 (°C)

            if (temp_c < kBaselineTempC) {
                state = OpenStage::Heating;
                duty_ = kDutySeq[0];
                ResetWindow();
                LOG_INF("Cooldown Skip");
                LOG_INF("Step Start");
            }
            else if (temp_c <= kBaselineTempC + kBaselineTol && StableCheck(temp_c, dt_us)) {
                state = OpenStage::Heating;
                duty_ = kDutySeq[0];
                ResetWindow();
                LOG_INF("Cooldown Stable");
                LOG_INF("Step Start");
            }
            break;
        }
        case OpenStage::Heating:
        {
            if (StableCheck(temp_c, dt_us)) 
            {
                if (++stage < kNumStages) {
                    duty_ = kDutySeq[stage];
                    ResetWindow();
                    LOG_INF("Stage End");
                    LOG_INF("Step Start");
                }
                else {
                    duty_ = kMinDuty;
                    state = OpenStage::Finished;
                    LOG_INF("Finished");
                }
            }
            break;
        }
        default:
            duty_ = kMinDuty;
            break;
    }
    
    last_cycle_    = now;
}

/**
 * @brief 在 PID 输出上叠加方波扰动
 *
 * @param pid_duty PID 计算的占空比
 * @param temp_c   当前温度 (°C)
 * @return 叠加扰动后的占空比
 */
float Identifier::Perturb(float pid_duty, float temp_c)
{
    constexpr float     kAmp     = 0.02f;    // 扰动幅值 (duty)
    constexpr uint32_t  kPeriod  = 1000;     // 扰动半周期（帧数）
    constexpr float     kStartC  = 35.0f;    // 开始扰动温度阈值 (°C)

    if (temp_c < kStartC) return pid_duty;

    const float delta = (++pert_cnt_ / kPeriod) % 2 ? kAmp : -kAmp;
    return std::clamp(pid_duty + delta, kMinDuty, kMaxDuty);
}

/**
 * @brief 滑动窗口稳定判据
 *
 * @param temp_c  当前温度 (°C)
 * @param dt_us   采样间隔 (µs)
 * @return true  窗口满且温度稳定
 */
bool Identifier::StableCheck(float temp_c, uint32_t dt_us)
{
    constexpr float kSlopeLimit = 0.01f;     // 稳定斜率限 (°C/s)
    constexpr float kNoiseLimit = 0.10f;     // 稳定极差限 (°C)

    if (win_.cnt < kWinSamples) {
        PushWin(temp_c);
        return false;
    }

    float mn = win_.buf[0], mx = win_.buf[0];
    for (uint32_t i = 1; i < kWinSamples; i++) {
        if (win_.buf[i] < mn) mn = win_.buf[i];
        if (win_.buf[i] > mx) mx = win_.buf[i];
    }

    const float dt_s = (dt_us != 0) ? (static_cast<float>(dt_us) * 1e-6f) : 0.001f;
    const float slope = (temp_c - win_.prev_c) / dt_s;
    PushWin(temp_c);

    return std::abs(slope) <= kSlopeLimit && (mx - mn) <= kNoiseLimit;
}

/**
 * @brief 向滑动窗口推入一帧温度
 *
 * @param temp_c 当前温度 (°C)
 */
void Identifier::CheckCmd()
{
    struct CmdEntry { 
        const char* name; 
        Cmd id; 
    }  constexpr kCmds[]  {                                       // PC下发指令
        { "StartIdent",   Cmd::StartIdent  },       
        { "Stop",         Cmd::Stop        }, 
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
 * @brief 将当前温度推入滑动窗口的环形缓冲区
 *
 * @param temp_c 当前温度 (°C)
 */
void Identifier::PushWin(float temp_c)
{
    win_.buf[win_.head] = temp_c;
    win_.head = (win_.head + 1) % kWinSamples;
    if (win_.cnt < kWinSamples) win_.cnt++;
    win_.prev_c = temp_c;
}

/**
 * @brief 清空滑动窗口缓存
 */
void Identifier::ResetWindow()
{
    win_.head = 0; win_.cnt = 0; win_.prev_c = 0.0f;
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
    pert_cnt_   = 0;
    ResetWindow();
}

/**
 * @brief 开环辨识主入口（每帧调用）
 *
 * 读取 UART 指令，分发到停止或状态机更新。
 *
 * @param temperature 当前温度 (°C)
 */
void Identifier::IdentOpenLoop(float temperature)
{
    CheckCmd();

    if (active_cmd_ == Cmd::Stop) {
        duty_ = kMinDuty;
        return;
    }

    if (prev_cmd_ == Cmd::Stop && active_cmd_ != Cmd::Stop) {
        LOG_INF("Ident Starting");
    }
    prev_cmd_ = active_cmd_;

    ExecOpenLoop(temperature);
}

} // namespace ident

#endif // CONFIG_IMU_IDENTIFICATION

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

    constexpr alg::pid::Pid::Config kPidConfig {
        .kp = 0.13f, 
        .ki = 0.03f, 
        .kd = 0.0f,
        .iOutMax = 0.5f, 
        .outMax = kMaxDuty, 
        .dt = 0.001f,
    };
    pid_.Init(kPidConfig);

    duty_ = 0.0f;
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
        duty_ = std::clamp(duty_, kMinDuty, kMaxDuty);
    }
#ifdef CONFIG_IMU_IDENTIFICATION
    else if (mode_ == Mode::OpenIdent)
    {
        ident_.IdentOpenLoop(temperature);
        duty_ = ident_.GetDuty();
    }
#endif // CONFIG_IMU_IDENTIFICATION

    (void)heater_pwm_.SetDuty(duty_);
}

/**
 * @brief 单帧稳定判据：检查偏差与瞬时斜率
 *
 * @param temperature 当前温度 (°C)
 * @param reference   参考温度 (°C)
 * @param tolerance   允许偏差 (°C)
 * @param dt          与上一帧的时间间隔 (s)
 * @return true  偏差和斜率均在限内
 */
bool Heater::StableSample(float temperature, float reference, float tolerance, float dt)
{
    if (std::abs(temperature - reference) > tolerance) {
        prev_temp_ = temperature;
        return false;
    }
    if (prev_temp_ == 0.0f) {
        prev_temp_ = temperature;
        return false;
    }
    const float slope = (temperature - prev_temp_) / dt;
    prev_temp_ = temperature;
    return std::abs(slope) <= kSlopeLimit;
}

/**
 * @brief 连续稳定判据：累计 StableSample 通过帧数
 *
 * 内部计数连续达 required 帧即判稳，不满足则重置。
 *
 * @param temperature 当前温度 (°C)
 * @param reference   参考温度 (°C)
 * @param tolerance   允许偏差 (°C)
 * @param required    连续稳定最少帧数
 * @param dt          采样间隔 (s)，传给 StableSample
 * @return true  连续 >= required 帧满足单帧判据
 */
bool Heater::IsStable(float temperature, float reference, float tolerance, uint32_t required, float dt)
{
    if (StableSample(temperature, reference, tolerance, dt)) {
        stable_count_++;
    } else {
        stable_count_ = 0;
    }
    return stable_count_ >= required;
}

} // namespace heater
