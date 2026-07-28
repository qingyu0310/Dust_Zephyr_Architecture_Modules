/**
 * @file imu.cpp
 * @author qingyu
 * @brief IMU 数据源管理、加热控制与姿态发布
 * @version 0.1
 * @date 2026-06-01
 */

#include "imu.hpp"
#include "heater.hpp"
#include <cstddef>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#pragma message "Compiling Modules/Imu/Drivers/Imu"

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

extern const imu::ImuEntry __imu_start[];
extern const imu::ImuEntry __imu_end[];

/**
 * @brief imu
 *
 */
namespace imu {

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
        LOG_ERR("no imu registered");
        return false;
    }

    if (count > 1) {
        LOG_ERR("multiple imu registered (%td), only one allowed", count);
        return false;
    }

    const ImuEntry *e = __imu_start;
    LOG_INF("init %s ...", e->name);

    source_ = e->source;
    return source_->Init();
}

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
        LOG_ERR("InitSource failed");
        return false;
    }
    LOG_INF("InitSource done");

    if (!heater_.Init()) {
        LOG_ERR("heater init failed");
        return false;
    }
    heater_.SetMode(heater::Mode::ClosedLoop);

    // LOG_INF("start preheat");
    // Preheat();

    if (mode == ImuStartMode::AutoCalib)
    {
        if (!source_->Calibrate()) { 
            LOG_ERR("calibrate failed"); 
            return false; 
        }
        LOG_INF("calibrate done");
    }
#ifdef CONFIG_IMU_IDENTIFICATION
    else if (mode == ImuStartMode::AutoIdent)
    {
        heater_.SetMode(heater::Mode::AutoIdent);
        LOG_INF("set autoident mode");
    }
#endif // CONFIG_IMU_IDENTIFICATION

    source_->LateInit();

    attitude_.Init();

    ready_ = true;
    LOG_INF("imu ready");
    
    return true;
}

/**
 * @brief 任务函数
 *
 */
void ImuManager::Task()
{
    const auto kMeasureDT = [&]() -> float
    {
        static uint32_t last_cycle = 0;
        const uint32_t now_cycle = k_cycle_get_32();
        if (last_cycle == 0) {
            last_cycle = now_cycle;
            return sample_.dt;
        }
        const uint64_t delta_ns = k_cyc_to_ns_floor64(now_cycle - last_cycle);
        last_cycle = now_cycle;
        return (delta_ns == 0) ? sample_.dt : static_cast<float>(delta_ns) * 1.0e-9f;
    };

    log_timer_.SetPeriod(10);

    for (;;)
    {
        log_timer_.Update();

        if (source_ != nullptr && source_->Read(sample_))
        {
            // 用真实经过时间修正当前样本积分周期，避免调度抖动直接传进 EKF。
            sample_.dt = kMeasureDT();

            heater_.Update(sample_.temp);

            attitude_.Process(sample_, pub_);

            // zbus_chan_pub(&pub_imu_to, &pub_, K_MSEC(1));
        }

        log_timer_.Clock([&]()
        {
            if (heater_.GetMode() == heater::Mode::ClosedLoop)
            {
                // LOG_INF("%f,%f,%f", (double)pub_.roll, (double)pub_.pitch, (double)pub_.yaw);
                // LOG_INF("%f,%f", (double)sample_.temp, (double)heater_.GetDuty());
            }
        });
        
        k_msleep(1);
    }
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

    log_timer_.SetPeriod(10);

    while (true)
    {
        log_timer_.Update();

        if (source_ == nullptr || !source_->Read(sample_)) {
            k_busy_wait(kWaitUs);
            continue;
        }

        if (heater_.Preheat(sample_.temp, kWaitUs * 1e-6f)) {
            return true;
        }

        // 预热打印，用于判断是否被稳定判据卡住
        log_timer_.Clock([&](){
            LOG_INF("%f,%f", (double)sample_.temp, (double)heater_.GetDuty());
        });

        if (k_uptime_get() - Start_ms >= kTimeoutMs) {
            LOG_ERR("preheat timeout");
            return false;
        }

        k_busy_wait(kWaitUs);
    }
}

} // namespace imu
