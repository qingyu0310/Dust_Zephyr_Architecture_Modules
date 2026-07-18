/**
 * @file bmi088.hpp
 * @author qingyu
 * @brief BMI088 IMU Source 接口与配置定义
 * @version 0.1
 * @date 2026-06-02
 */

#pragma once

#include "imu_device_layer.hpp"
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
class Bmi088 final : public CalibratedImuSource
{
public:
    /**
     * @brief BMI088 运行时配置
     */
    struct Config {
        const struct spi_dt_spec *accel = nullptr;
        const struct spi_dt_spec *gyro = nullptr;
        ImuCalibration static_calibration {};
    };

    bool Init() override;

    friend bool RegisterFromDevicetree();

    Error LastError() const;

private:
    bool  InitAccel          ();
    bool  InitGyro           ();
    bool  ReadRaw            (ImuRawSample& raw) override;
    bool  WriteAccel         (uint8_t addr, uint8_t  value);
    bool  WriteGyro          (uint8_t addr, uint8_t  value);
    bool  ReadAccel          (uint8_t addr, uint8_t  *data, uint32_t len);
    bool  ReadGyro           (uint8_t addr, uint8_t  *data, uint32_t len);
    bool  ReadAccelReg       (uint8_t addr, uint8_t& value);
    bool  ReadGyroReg        (uint8_t addr, uint8_t& value);
    bool  WriteCheckedAccel  (uint8_t addr, uint8_t  value);
    bool  WriteCheckedGyro   (uint8_t addr, uint8_t  value);
    float ConvertAccel       (int16_t raw) const override;
    float ConvertGyro        (int16_t raw) const override;
    float ConvertTemperature (int16_t raw) const override;

    Config config_ {};
    Spi accel_ {};
    Spi gyro_  {};
    Error last_error_ = Error::None;

    static constexpr uint32_t kSpiBufferSize = 8;
    uint8_t tx_[kSpiBufferSize] {};
    uint8_t rx_[kSpiBufferSize] {};
};

/**
 * @brief 返回 BMI088 单例数据源
 */
Bmi088& Instance();

/**
 * @brief 根据 devicetree alias 构造 BMI088 配置
 */
bool RegisterFromDevicetree();

} // namespace bmi088
