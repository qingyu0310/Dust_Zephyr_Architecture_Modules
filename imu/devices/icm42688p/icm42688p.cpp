/**
 * @file icm42688p.cpp
 * @author qingyu
 * @brief ICM42688P IMU Source 实现
 * @version 0.1
 * @date 2026-06-02
 */

#include "icm42688p.hpp"
#include "icm42688p_reg.hpp"
#include "imu.hpp"

#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include "log.hpp"

#pragma message "Compiling Modules/Imu/Devices/ICM42688P"


namespace icm42688p {

namespace {

/**
 * @brief ICM42688P 单寄存器初始化项
 */
struct RegConfig {
    uint8_t reg;
    uint8_t value;
};

/**
 * @brief 当前已确认的通用寄存器初始化表
 */
constexpr RegConfig kInitConfig[] {
    {reg::kComCfg,       reg::kComCfgDefault    },
    {reg::kHpfLpfCfg,    reg::kHpfLpfCfgDefault },
    {reg::kStepCfg,      reg::kStepCfgDefault   },
    {reg::kAccConf,      reg::kAccConfDefault   },
    {reg::kAccRange,     reg::kAccRangeDefault  },
    {reg::kGyrConf,      reg::kGyrConfDefault   },
    {reg::kFifoDowns,    reg::kFifoDownsDefault },
};

} // namespace

/**
 * @brief 初始化 ICM42688P 并按需执行自动标定
 */
bool Icm42688p::Init()
{
    current_segment_ = 0xFFU;
    last_error_ = Error::None;

    constexpr uint32_t kSpiOperation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA;
    static const struct spi_dt_spec imu = SPI_DT_SPEC_GET(DT_ALIAS(imu_spi), kSpiOperation);

    if (!spi_.Init(imu)) {
        DUST_LOG_ERR("SPI init failed");
        last_error_ = Error::DeviceNotReady;
        return false;
    }

    // SPI 初始化完成后忙等 3ms，待器件就绪再软复位
    k_busy_wait(3000);

    if (!SoftReset()) {
        DUST_LOG_ERR("Soft reset failed");
        last_error_ = Error::Config;
        return false;
    }

    if (!SelectSegment(reg::kSegSelGeneral)) {
        DUST_LOG_ERR("Select general segment failed");
        last_error_ = Error::Config;
        return false;
    }

    uint8_t chip_id = 0;
    if (!ReadReg(reg::kWhoAmI, chip_id)) {
        DUST_LOG_ERR("Read WHO_AM_I failed");
        last_error_ = Error::ChipId;
        return false;
    }
    if (chip_id != reg::kWhoAmIValue) {
        DUST_LOG_ERR("WHO_AM_I mismatch, got 0x%02X, expected 0x%02X", chip_id, reg::kWhoAmIValue);
        last_error_ = Error::ChipId;
        return false;
    }

    if (!InitRegisters()) {
        DUST_LOG_ERR("Init registers failed");
        last_error_ = Error::Config;
        return false;
    }

    return true;
}

/**
 * @brief 延迟初始化 — 从 flash 读取校准参数
 *
 * 如果已完成在线标定（calibrated_ == true），跳过。
 * 否则尝试从 flash 分区读取零偏数据。flash 有有效数据时
 * 覆盖 offset_ 并标记 calibrated_，后续不再跑自动标定。
 */
bool Icm42688p::LateInit()
{
    if (calibrated_) return true;

    ImuOffsetData calib{};
    if (!EXEC_FLASH_READ(flash::kPartCalib.offset, &calib, sizeof(ImuOffsetData))) {
        DUST_LOG_ERR("read calib from flash failed");
    }
    
    if (calib.gyro_offset[0] != 0.0f || calib.gyro_offset[1] != 0.0f || calib.gyro_offset[2] != 0.0f) {
        offset_ = calib;
        calibrated_ = true;
        DUST_LOG_INF("load calib from flash");
    }
    else {
        DUST_LOG_INF("no calib data, use reg default");
    }

    return true;
}

/**
 * @brief 读取一帧 ICM42688P 原始寄存器样本
 */
bool Icm42688p::ReadRaw(ImuRawSample& raw)
{
    uint8_t motion_data[12] {};
    uint8_t temp_data[2]    {};

    if (!SelectSegment(reg::kSegSelGeneral) ||
        !ReadRegs(reg::kAccXH, motion_data, sizeof(motion_data)) ||
        !ReadRegs(reg::kTempH, temp_data  , sizeof(temp_data))) 
    {
        last_error_ = Error::ReadFailed;
        return false;
    }

    raw.accel[0] = static_cast<int16_t>((static_cast<uint16_t>(motion_data[0])  << 8) | static_cast<uint16_t>(motion_data[1]));
    raw.accel[1] = static_cast<int16_t>((static_cast<uint16_t>(motion_data[2])  << 8) | static_cast<uint16_t>(motion_data[3]));
    raw.accel[2] = static_cast<int16_t>((static_cast<uint16_t>(motion_data[4])  << 8) | static_cast<uint16_t>(motion_data[5]));

    raw.gyro [0] = static_cast<int16_t>((static_cast<uint16_t>(motion_data[6])  << 8) | static_cast<uint16_t>(motion_data[7]));
    raw.gyro [1] = static_cast<int16_t>((static_cast<uint16_t>(motion_data[8])  << 8) | static_cast<uint16_t>(motion_data[9]));
    raw.gyro [2] = static_cast<int16_t>((static_cast<uint16_t>(motion_data[10]) << 8) | static_cast<uint16_t>(motion_data[11]));

    raw.temp     = static_cast<int16_t>((static_cast<uint16_t>(temp_data[0])    << 8) | static_cast<uint16_t>(temp_data[1]));

    last_error_ = Error::None;
    return true;
}

/**
 * @brief 触发芯片软复位并清空本地段选择状态
 */
bool Icm42688p::SoftReset()
{
    current_segment_ = reg::kSegSelGeneral;
    if (!WriteReg(reg::kSegSel, reg::kSegSelGeneral) ||
        !WriteReg(reg::kSoftRstReg, 0x01)) {
        return false;
    }

    k_busy_wait(2000);
    current_segment_ = 0xFFU;
    return true;
}

/**
 * @brief 写入启动寄存器表并打开 accel/gyro 工作模式
 */
bool Icm42688p::InitRegisters()
{
    if (!SelectSegment(reg::kSegSelGeneral)) {
        return false;
    }

    for (const auto& cfg : kInitConfig) 
    {
        if (!WriteChecked(cfg.reg, cfg.value)) {
            return false;
        }
        k_busy_wait(1000);
    }

    if (!WriteChecked(reg::kPwrCtrl, reg::kPwrOnAll)) {
        return false;
    }
    k_busy_wait(50000);

    // GYR_NOISE_PERF 配置序列：先关电源 → 写 GYR_CONF → 重新上电
    if (!WriteReg(reg::kPwrCtrl, reg::kPwrOff) ||
        !WriteChecked(reg::kGyrConf, reg::kGyrConfNoiseOpt) ||
        !WriteReg(reg::kPwrCtrl, reg::kPwrOnAll)) {
        return false;
    }
    k_busy_wait(50000);

    return true;
}

/**
 * @brief 仅在段变化时切换寄存器段
 */
bool Icm42688p::SelectSegment(uint8_t segment)
{
    if (current_segment_ == segment) {
        return true;
    }

    tx_[0] = reg::kSegSel;
    tx_[1] = segment;

    if (!spi_.Send(tx_, 2)) {
        return false;
    }

    current_segment_ = segment;
    return true;
}

/**
 * @brief 通过一次 SPI transceive 连续读取多个寄存器
 */
bool Icm42688p::ReadRegs(uint8_t addr, uint8_t *data, uint32_t len)
{
    constexpr uint8_t kDummyByte = 0x00;

    if (data == nullptr || len == 0 || len + 1 > sizeof(tx_)) {
        return false;
    }

    memset(tx_, kDummyByte, len + 1);
    memset(rx_, 0x00, len + 1);
    tx_[0] = addr | reg::kReadFlag;

    if (!spi_.Transceive(tx_, rx_, len + 1)) {
        return false;
    }

    memcpy(data, &rx_[1], len);
    return true;
}

/**
 * @brief 写入寄存器并执行基本回读校验
 */
bool Icm42688p::WriteChecked(uint8_t addr, uint8_t value)
{
    uint8_t readback = 0;
    if (!WriteReg(addr, value)) {
        return false;
    }

    k_busy_wait(1000);
    return ReadReg(addr, readback) && readback == value;
}

REGISTER_IMU(Icm42688p, icm42688p);

} // namespace icm42688p

