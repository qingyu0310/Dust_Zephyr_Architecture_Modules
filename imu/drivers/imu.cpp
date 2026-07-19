/**
 * @file imu.cpp
 * @author qingyu
 * @brief IMU 数据源管理、加热控制与姿态发布
 * @version 0.1
 * @date 2026-06-01
 */

#include "imu.hpp"
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_MOD_DEV_IMU_BMI088
#include "bmi088/bmi088.hpp"
#endif
#ifdef CONFIG_MOD_DEV_IMU_ICM42688P
#include "icm42688p/icm42688p.hpp"
#endif

#pragma message "Compiling Modules/Imu/Drivers/Imu"

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

namespace {

using RegisterFromDevicetreeFn = bool (*)();

/**
 * @brief 将 IMU 驱动注册到 devicetree 对应的总线
 *
 * 驱动内部通过 Devicetree 宏查找 SPI/I2C 总线实例，注册为 Zephyr 设备。
 * 具体选哪个传感器由 Kconfig + Devicetree 决定，此处只传采样周期。
 *
 * @param register_fn 驱动的注册函数
 * @return true  注册成功
 * @return false 注册失败
 */
bool RegisterSourceFromDevicetree(RegisterFromDevicetreeFn register_fn)
{
    if (register_fn == nullptr || !register_fn()) {
        return false;
    }

    return true;
}

} // namespace

/**
 * @brief imu
 *
 */
namespace imu {

/**
 * @brief 根据 Kconfig 选择底层 IMU 数据源
 *
 * 通过 CONFIG_MOD_DEV_IMU_* 编译时宏决定实例化哪个驱动，
 * 驱动在对应板的 .conf 中开启，代码层无需频繁切换。
 *
 * @return 是否成功选定并注册了数据源
 */
bool ImuManager::SelectSource()
{
#if CONFIG_MOD_DEV_IMU_ICM42688P
{
#pragma  message "Select IMU driver: ICM42688P"
    source_ = &icm42688p::Instance();
    return source_ != nullptr && RegisterSourceFromDevicetree(icm42688p::RegisterFromDevicetree);
}
#elif CONFIG_MOD_DEV_IMU_BMI088
{
#pragma message "Select IMU driver: BMI088"
    source_ = &bmi088::Instance();
    return source_ != nullptr && RegisterSourceFromDevicetree(bmi088::RegisterFromDevicetree);
}
#else
{
#warning "No IMU driver selected"
    return false;
}
#endif
}

/**
 * @brief 初始化 IMU 管理器
 *
 * 按依赖顺序串联各子模块：选择数据源 → 初始化驱动 → 初始化加热器
 * → 可选的预热与静态校准 → 初始化姿态解算器。
 *
 * @param enable_auto_calibration 是否在启动阶段执行预热后的静态校准
 */
void ImuManager::Init(ImuStartMode mode)
{
    ready_ = false;

    if (source_ == nullptr && !SelectSource()) {
        LOG_ERR("SelectSource failed");
        return;
    }
    if (source_ == nullptr) {
        LOG_ERR("source is null");
        return;
    }
    if (!source_->Init()) {
        LOG_ERR("source init failed");
        return;
    }
    if (!heater_.Init()) {
        LOG_ERR("heater init failed");
        return;
    }
    heater_.SetMode(heater::Mode::Normal);

    // Preheat();
    LOG_INF("start preheat");

    if (mode == ImuStartMode::AutoCalib)
    {
        if (!source_->Calibrate()) { 
            LOG_ERR("calibrate failed"); 
            return; 
        }
        LOG_INF("calibrate done");
    }
#ifdef CONFIG_IMU_IDENTIFICATION
    else if (mode == ImuStartMode::OpenIdent)
    {
        heater_.SetMode(heater::Mode::OpenIdent);
        LOG_INF("open_ident_start");
    }
#endif

    attitude_.Init();
    ready_ = true;
    LOG_INF("imu ready");
}

/**
 * @brief 启动 IMU 工作线程
 *
 * @param prio 线程优先级
 */
void ImuManager::Start(uint8_t prio)
{
    if (!ready_) {
        return;
    }

    thread_.Start(TaskEntry, prio, this);
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
    constexpr float    kTargetTemp  = 40.0f;            // 预热目标温度 (°C)
    constexpr float    kStableTol   = 0.5f;             // 稳定温度容限 (°C)
    constexpr uint32_t kStableCnt   = 500;              // 持续稳定帧数
    constexpr uint32_t kWaitUs      = 1000;             // 采样间隔 (µs)

    log_timer_.SetPeriod(10);

    while (true)
    {
        log_timer_.Update();

        if (source_ == nullptr || !source_->Read(sample_)) {
            k_busy_wait(kWaitUs);
            continue;
        }

        heater_.Update(sample_.temp);
        if (heater_.IsStable(sample_.temp, kTargetTemp, kStableTol, kStableCnt)) {
            return true;
        }

        log_timer_.Clock([&](){
            LOG_INF("%f,%f", (double)sample_.temp, (double)heater_.GetDuty());
        });

        k_busy_wait(kWaitUs);
    }
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

        log_timer_.Clock([&](){
            // LOG_INF("%f,%f,%f", (double)pub_.roll, (double)pub_.pitch, (double)pub_.yaw);
            // LOG_INF("%f,%f", (double)sample_.temp, (double)heater_.GetDuty());
        });

        k_msleep(1);
    }
}

} // namespace imu
