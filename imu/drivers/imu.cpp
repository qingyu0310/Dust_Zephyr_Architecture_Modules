/**
 * @file imu.cpp
 * @author qingyu
 * @brief IMU 数据源管理、加热控制与姿态发布
 * @version 0.2
 * @date 2026-10-19
 */

#include "imu.hpp"
#include "heater.hpp"
#include <cstddef>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include "log.hpp"

#pragma message "Compiling Modules/Imu/Drivers/Imu"


extern const imu::ImuEntry __imu_start[];
extern const imu::ImuEntry __imu_end[];

namespace imu {

/**
 * @brief 初始化 IMU 管理器
 *
 * 按依赖顺序串联各子模块：
 * 选择数据源 → 初始化驱动 → 初始化加热器
 * → 可选自动校准（AutoCalib）或辨识（AutoIdent）
 * → LateInit：flash 有校准数据则加载，无则保持 reg 默认
 * → 初始化姿态解算器。
 *
 * 校准数据优先级：
 * - AutoCalib 模式：在线采集 → 计算 offset → 写入 flash
 * - Normal 模式：不跑 Calibrate，由 LateInit 决定是否从 flash 加载
 * - 既无 flash 数据也未跑 Calibrate：使用 reg 出厂默认值
 *
 * @param mode ImuStartMode::Normal / AutoCalib / AutoIdent
 */
bool ImuManager::Init(ImuStartMode mode)
{
    ready_ = false;

    if (!InitSource()) {
        DUST_LOG_ERR("InitSource failed");
        return false;
    }
    DUST_LOG_INF("InitSource done");

    if (!heater_.Init()) {
        DUST_LOG_ERR("heater init failed");
        return false;
    }

    // DUST_LOG_INF("start preheat");
    // Preheat();

    if (mode == ImuStartMode::AutoCalib)
    {
        if (!source_->Calibrate()) { 
            DUST_LOG_ERR("calibrate failed"); 
            return false; 
        }
        DUST_LOG_INF("calibrate done");
    }

    source_->LateInit();

    attitude_.Init();

    ready_ = true;
    DUST_LOG_INF("imu ready");
    
    return true;
}

/**
 * @brief 任务函数 — 单拍：读一帧 → 控温 → 姿态解算
 *
 * 循环、停止检查与 1ms 节拍由 Thread 外壳负责。
 *
 * @param context ImuManager 实例
 * @return Continue=正常；Retry=读取失败
 */
TaskResult ImuManager::Task(void* context)
{
    auto* self = static_cast<ImuManager*>(context);

    const auto kMeasureDT = [&]() -> float
    {
        static uint32_t last_cycle = 0;
        const uint32_t now_cycle = k_cycle_get_32();
        if (last_cycle == 0) {
            last_cycle = now_cycle;
            return self->sample_.dt;
        }
        const uint64_t delta_ns = k_cyc_to_ns_floor64(now_cycle - last_cycle);
        last_cycle = now_cycle;
        return (delta_ns == 0) ? self->sample_.dt : static_cast<float>(delta_ns) * 1.0e-9f;
    };

    self->log_timer_.SetPeriod(1);
    self->log_timer_.Update();

    if (self->source_ == nullptr || !self->source_->Read(self->sample_)) {
        return TaskResult::Retry;
    }

    // 用真实经过时间修正当前样本积分周期，避免调度抖动直接传进 EKF。
    self->sample_.dt = kMeasureDT();

    self->heater_.Update(self->sample_.temp);

    self->attitude_.Process(self->sample_, self->pub_);

    // zbus_chan_pub(&pub_imu_to, &self->pub_, K_MSEC(1));

    self->log_timer_.Clock([&]()  {
        // printk("%f,%f,%f\n", (double)self->pub_.roll, (double)self->pub_.pitch, (double)self->pub_.yaw);
        // printk("%f, %f\n", (double)self->sample_.temp, (double)self->heater_.GetDuty());
    });

    return TaskResult::Continue;
}

/**
 * @brief 根据注册表选择底层 IMU 数据源
 *
 * 遍历 .imu linker section 中的 ImuEntry，要求恰好注册一个设备。
 * 0 个报错，2+ 报错，仅 1 个时调用 Init()。
 *
 * @return 是否成功选定并初始化数据源
 */
bool ImuManager::InitSource()
{
    ptrdiff_t count = __imu_end - __imu_start;

    if (count == 0) {
        DUST_LOG_ERR("no imu registered");
        return false;
    }

    if (count > 1) {
        DUST_LOG_ERR("multiple imu registered (%td), only one allowed", count);
        return false;
    }

    const ImuEntry *e = __imu_start;
    DUST_LOG_INF("init %s ...", e->name);

    source_ = e->source;
    return source_->Init();
}

/**
 * @brief 等待加热器达到目标温度
 *
 * 循环采集温度样本，直到加热器稳定在目标温度附近。
 *
 * @return true  温度稳定
 * @return false 数据源失效
 */
bool ImuManager::Preheat()
{
    constexpr uint32_t kWaitUs    = 1000;               // 采样间隔 (µs)
    constexpr uint32_t kTimeoutMs = 10000;              // 超时时间 (ms)

    const uint32_t Start_ms = k_uptime_get();

    // 用真实经过时间算采样间隔，避免 IMU 输出率/日志开销导致 dt 虚高
    const auto kMeasureDT = [&]() -> float
    {
        static uint32_t last_cycle = 0;
        const uint32_t now_cycle = k_cycle_get_32();
        if (last_cycle == 0) {
            last_cycle = now_cycle;
            return 0.001f;
        }
        const uint64_t delta_ns = k_cyc_to_ns_floor64(now_cycle - last_cycle);
        last_cycle = now_cycle;
        return (delta_ns == 0) ? 0.001f : static_cast<float>(delta_ns) * 1.0e-9f;
    };

    log_timer_.SetPeriod(10);

    while (true)
    {
        log_timer_.Update();

        if (source_ == nullptr || !source_->Read(sample_)) {
            k_busy_wait(kWaitUs);
            continue;
        }

        if (heater_.Preheat(sample_.temp, kMeasureDT())) {
            return true;
        }

        // 预热打印，用于判断是否被稳定判据卡住
        log_timer_.Clock([&](){
            DUST_LOG_INF("%f,%f", (double)sample_.temp, (double)heater_.GetDuty());
        });

        if (k_uptime_get() - Start_ms >= kTimeoutMs) {
            DUST_LOG_ERR("preheat timeout");
            return false;
        }

        k_busy_wait(kWaitUs);
    }
}

} // namespace imu
