/**
 * @file icm42688p.hpp
 * @author qingyu
 * @brief ICM42688P IMU Source 接口与配置定义
 * @version 0.1
 * @date 2026-06-02
 */

#pragma once

#include "imu_device_layer.hpp"
#include "spi.hpp"

#include <cstdint>
#include <zephyr/drivers/spi.h>

namespace icm42688p {

/**
 * @brief ICM42688P 驱动错误类型
 */
enum class Error : uint8_t
{
    None = 0,
    DeviceNotReady,
    ChipId,
    Config,
    ReadFailed,
};

/**
 * @brief ICM42688P 数据源实现
 *
 * ICM42688P 使用单 SPI 设备访问，寄存器分段通过 `SEG_SEL` 切换。
 * 公共工程量转换、校准与读样流程由基类复用。
 */
class Icm42688p final : public CalibratedImuSource
{
public:
    /**
     * @brief ICM42688P 运行时配置
     */
    struct Config {
        const struct spi_dt_spec *spi = nullptr;
        ImuCalibration static_calibration {};
    };

    bool Init() override;
    Error LastError() const;

    friend bool RegisterFromDevicetree();

private:
    Config  config_ {};
    Spi     spi_    {};
    Error   last_error_      = Error::None;
    uint8_t current_segment_ = 0xFFU;        // 当前寄存器段，0xFF 表示未选择

    static constexpr uint32_t kSpiBufferSize = 16;
    uint8_t tx_[kSpiBufferSize] {};
    uint8_t rx_[kSpiBufferSize] {};

    bool  SoftReset          ();
    bool  InitRegisters      ();
    bool  SelectSegment      (uint8_t segment);
    bool  ReadRaw            (ImuRawSample& raw) override;
    bool  WriteReg           (uint8_t addr, uint8_t  value);
    bool  ReadReg            (uint8_t addr, uint8_t& value);
    bool  ReadRegs           (uint8_t addr, uint8_t  *data, uint32_t len);
    bool  WriteChecked       (uint8_t addr, uint8_t  value);
    float ConvertAccel       (int16_t raw) const override;
    float ConvertGyro        (int16_t raw) const override;
    float ConvertTemperature (int16_t raw) const override;
};

/**
 * @brief 返回 ICM42688P 单例数据源
 */
Icm42688p& Instance();

/**
 * @brief 根据 devicetree alias 构造 ICM42688P 配置
 */
bool RegisterFromDevicetree();

} // namespace icm42688p
