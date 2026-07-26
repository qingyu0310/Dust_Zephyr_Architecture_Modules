/**
 * @file bmi088.hpp
 * @author qingyu
 * @brief BMI088 IMU Source 接口与配置定义
 * @version 0.1
 * @date 2026-06-02
 */

#pragma once

#include "imu_device_layer.hpp"
#include "bmi088_reg.hpp"
#include "spi.hpp"

#include <cstdint>
#include <zephyr/drivers/spi.h>

namespace bmi088 {

/**
 * @brief BMI088 驱动错误类型
 */
enum class Error : uint8_t
{
    None = 0,
    AccelNotReady,
    GyroNotReady,
    AccelChipId,
    GyroChipId,
    AccelConfig,
    GyroConfig,
    ReadFailed,
};

/**
 * @brief BMI088 数据源实现
 *
 * BMI088 由 accel/gyro 两个 SPI 设备组成：
 * - 子类负责底层寄存器访问与初始化
 * - 公共工程量转换、校准与读样流程由基类复用
 */
class Bmi088 final : public CalibSource
{
public:
    /**
     * @brief BMI088 运行时配置
     */
    struct Config {
        const struct spi_dt_spec *accel = nullptr;
        const struct spi_dt_spec *gyro = nullptr;
    };

    Bmi088();
    bool Init() override;
    bool LateInit() override;
    Error LastError() const { return last_error_; }

private:
    Config config_ {};
    Spi accel_ {};
    Spi gyro_  {};
    Error last_error_ = Error::None;

    static constexpr uint32_t kSpiBufferSize = 8;
    uint8_t tx_[kSpiBufferSize] {};
    uint8_t rx_[kSpiBufferSize] {};

    bool  InitAccel          ();
    bool  InitGyro           ();
    bool  ReadRaw            (ImuRawSample& raw) override;
    
    bool  ReadAccel          (uint8_t addr, uint8_t  *data, uint32_t len);
    bool  ReadGyro           (uint8_t addr, uint8_t  *data, uint32_t len);
    bool  WriteCheckedAccel  (uint8_t addr, uint8_t  value);
    bool  WriteCheckedGyro   (uint8_t addr, uint8_t  value);

    bool WriteAccel(uint8_t addr, uint8_t value)
    {
        tx_[0] = addr & reg::kWriteFlag;
        tx_[1] = value;
        return accel_.Send(tx_, 2);
    }

    bool WriteGyro(uint8_t addr, uint8_t value)
    {
        tx_[0] = addr & reg::kWriteFlag;
        tx_[1] = value;
        return gyro_.Send(tx_, 2);
    }

    bool ReadAccelReg(uint8_t addr, uint8_t& value)
    {
        return ReadAccel(addr, &value, 1);
    }

    bool ReadGyroReg(uint8_t addr, uint8_t& value)
    {
        return ReadGyro(addr, &value, 1);
    }

    float ConvertAccel(int16_t raw) const override
    {
        constexpr float kSens6g = 0.00179443359375f;
        return static_cast<float>(raw) * kSens6g;
    }

    float ConvertGyro(int16_t raw) const override
    {
        constexpr float kSens2000Dps = 0.0010652644360316953f;
        return static_cast<float>(raw) * kSens2000Dps;
    }

    float ConvertTemperature(int16_t raw) const override
    {
        constexpr float kFactor = 0.125f, kOff = 23.0f;
        return static_cast<float>(raw) * kFactor + kOff;
    }
};

} // namespace bmi088
