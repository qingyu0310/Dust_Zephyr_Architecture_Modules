/**
 * @file imu_mid_layer.hpp
 * @author qingyu
 * @brief IMU 数据源公共基类与共享数据结构
 * @version 0.1
 * @date 2026-06-03
 */

#pragma once

#pragma message "Compiling Modules/Imu/Devices/DeviceLayer"

#include <math.h>
#include <stdint.h>
#include <zephyr/kernel.h>

/**
 * @brief 一帧 IMU 工程量样本
 *
 * 具体 IMU 驱动负责完成寄存器读取、单位换算与驱动侧校准，
 * 上层姿态线程统一消费这个结构。
 */
struct Sample
{
    float gyro[3]  = {0.0f, 0.0f, 0.0f};
    float accel[3] = {0.0f, 0.0f, 0.0f};
    float temp     = 0.0f;
    float dt       = 0.001f;           // 积分步长(s)，EKF 姿态解算使用
};

/**
 * @brief 具体 IMU 驱动需要实现的统一接口
 */
class Source
{
public:
    virtual ~Source() = default;

    virtual bool Init() = 0;
    virtual bool Read(Sample& sample) = 0;
    virtual bool Calibrate() { return true; }
};

/**
 * @brief 原始 IMU 寄存器样本
 *
 * 统一保存 accel/gyro/temperature 原始计数，供公共后处理流程复用。
 */
struct ImuRawSample
{
    int16_t accel[3] = {0, 0, 0};
    int16_t gyro [3] = {0, 0, 0};
    int16_t temp     = 0;
};

/**
 * @brief IMU 静态校准参数
 */
struct ImuCalibration
{
    float gyro_offset [3] = {0.0f, 0.0f, 0.0f};
    float gyro_scale  [3] = {1.0f, 1.0f, 1.0f};
    float accel_offset[3] = {0.0f, 0.0f, 0.0f};
    float accel_scale [3] = {1.0f, 1.0f, 1.0f};
};

/**
 * @brief 带公共校准与单位换算流程的 IMU 数据源基类
 *
 * 子类只需提供原始样本读取与工程量换算函数，
 * 基类自动处理校准参数维护和工程量转换。
 */
class CalibratedImuSource : public Source
{
public:
    bool Calibrate() override
    {
        constexpr uint32_t kCalibPeriodMs   = 1;
        constexpr uint32_t kCalibCnt        = 200;
        constexpr float    kGravity         = 9.807f;

        // 忙等直到传感器瞬态稳定，避免上电漂移污染静态均值；不切任务。
        constexpr uint32_t kSettleMs = 50;
        k_busy_wait(kSettleMs * 1000);

        float gyro_sum [3] = {0.0f, 0.0f, 0.0f};
        float accel_sum[3] = {0.0f, 0.0f, 0.0f};

        for (uint16_t sample_idx = 0; sample_idx < kCalibCnt; sample_idx++)
        {
            ImuRawSample raw {};
            if (!ReadRaw(raw)) {
                return false;
            }

            for (uint8_t axis = 0; axis < 3; axis++) {
                gyro_sum [axis] += ConvertGyro (raw.gyro [axis]);
                accel_sum[axis] += ConvertAccel(raw.accel[axis]);
            }

            k_busy_wait(kCalibPeriodMs * 1000);
        }

        const float inv_samples = 1.0f / static_cast<float>(kCalibCnt);
        float accel_mean[3] = {0.0f, 0.0f, 0.0f};
        float accel_norm_sq = 0.0f;

        for (uint8_t axis = 0; axis < 3; axis++) {
            static_calibration_.gyro_offset[axis] = gyro_sum[axis] * inv_samples;
            accel_mean[axis]  = accel_sum [axis]  * inv_samples;
            accel_norm_sq    += accel_mean[axis]  * accel_mean[axis];
        }

        if (accel_norm_sq > 0.0f)
        {
            const float accel_norm = sqrtf(accel_norm_sq);
            const float gravity_scale = kGravity / accel_norm;

            for (uint8_t axis = 0; axis < 3; axis++) {
                const float expected_gravity = accel_mean[axis] * gravity_scale;
                static_calibration_.accel_offset[axis] = accel_mean[axis] - expected_gravity;
            }
        }

        return true;
    }

    bool Read(Sample& sample) override
    {
        ImuRawSample raw {};
        if (!ReadRaw(raw)) {
            return false;
        }

        for (uint8_t axis = 0; axis < 3; axis++)
        {
            const float accel = ConvertAccel(raw.accel[axis]);
            const float gyro  = ConvertGyro (raw.gyro [axis]);

            sample.accel[axis] = Correct(accel,
                                         static_calibration_.accel_offset[axis],
                                         static_calibration_.accel_scale[axis]);
            sample.gyro [axis] = Correct(gyro,
                                         static_calibration_.gyro_offset[axis],
                                         static_calibration_.gyro_scale[axis]);
        }

        sample.temp = ConvertTemperature(raw.temp);
        sample.dt = 0.001f;
        return true;
    }

protected:
    ImuCalibration static_calibration_ {};

    virtual bool  ReadRaw(ImuRawSample& raw)      = 0;
    virtual float ConvertAccel(int16_t raw) const = 0;
    virtual float ConvertGyro(int16_t raw)  const = 0;
    virtual float ConvertTemperature(int16_t raw) const = 0;

    static float Correct(float value, float offset, float scale)
    {
        return (value - offset) * scale;
    }
};
