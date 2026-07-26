/**
 * @file icm42688p.hpp
 * @author qingyu
 * @brief ICM42688P IMU Source 接口与配置定义
 * @version 0.1
 * @date 2026-06-02
 */

#pragma once

#include "imu_device_layer.hpp"
#include "icm42688p_reg.hpp"
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
class Icm42688p final : public CalibSource
{
public:
    Icm42688p()
    {
        offset_ = reg::kStaticOffset;
        scale_  = reg::kStaticScale;
    }

    bool Init() override;
    bool LateInit() override;
    Error LastError() const { return last_error_; }

private:
    Spi     spi_    {};
    Error   last_error_      = Error::None;
    uint8_t current_segment_ = 0xFFU;        // 当前寄存器段，0xFF 表示未选择

    static constexpr uint32_t kSpiBufferSize = 16;
    uint8_t tx_[kSpiBufferSize] {};
    uint8_t rx_[kSpiBufferSize] {};

    bool SoftReset          ();
    bool InitRegisters      ();
    bool SelectSegment      (uint8_t segment);
    bool ReadRaw            (ImuRawSample& raw) override;

    bool ReadRegs           (uint8_t addr, uint8_t  *data, uint32_t len);
    bool WriteChecked       (uint8_t addr, uint8_t  value);

    bool WriteReg(uint8_t addr, uint8_t value)
    {
        tx_[0] = addr;
        tx_[1] = value;
        return spi_.Send(tx_, 2);
    }

    bool ReadReg(uint8_t addr, uint8_t& value)
    {
        return ReadRegs(addr, &value, 1);
    }

    float ConvertAccel(int16_t raw) const override
    {
        constexpr float kSens16g = 16.0f / 32768.0f;
        return static_cast<float>(raw) * kSens16g * 9.8f;
    }

    float ConvertGyro(int16_t raw) const override
    {
        constexpr float kSens2000Dps = 0.0010652644360316953f;
        return static_cast<float>(raw) * kSens2000Dps;
    }

    float ConvertTemperature(int16_t raw) const override
    {
        constexpr float kFactor = 1.0f / 512.0f, kOff = 23.0f;
        return static_cast<float>(raw) * kFactor + kOff;
    }
};

} // namespace icm42688p
