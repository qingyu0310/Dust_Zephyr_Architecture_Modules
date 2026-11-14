/**
 * @file sbus.cpp
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-07-22
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "protocol_base.hpp"
#include "remote.hpp"
#include "sbus.hpp"
#include "log.hpp"

using namespace topic::remote_to;

namespace sbus {

static constexpr uint16_t kFrameSizeSBUS = 25;

class SbusProtocol final : public remote::RemoteProtocol
{
public:
    SbusProtocol()
    {
        line_cfg_.baudrate  = 100000;
        line_cfg_.parity    = UART_CFG_PARITY_EVEN;
        line_cfg_.stop_bits = UART_CFG_STOP_BITS_2;
        line_cfg_.data_bits = UART_CFG_DATA_BITS_8;
        line_cfg_.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
    }

    bool Validate(const uint8_t *buffer, uint8_t len) override;
    bool Decode(const uint8_t *buffer, uint8_t len, topic::remote_to::Message &pub) override;

private:
    static constexpr uint16_t kMin    = 353;
    static constexpr uint16_t kMax    = 1694;
    static constexpr uint16_t kCenter = 1024;

    static bool  IsValidChannel(uint16_t v) { return v >= kMin && v <= kMax; }
    static void  ClassifyTwoPos(uint16_t v, TwoPos &status);
    static void  ClassifyThreePos(uint16_t v, ThreePos &status);
    static void  DefaultFillOutputData(const OutputData &od, topic::remote_to::Message &pub);
};

/**
 * @brief 帧校验
 * @param buffer  原始帧数据
 * @param len     帧长度
 * @return true   校验通过
 */
bool SbusProtocol::Validate(const uint8_t* buffer, uint8_t len)
{
    if (len != kFrameSizeSBUS)  return false;
    if (buffer[0] != 0x0F)      return false;
    if (buffer[24] != 0x00)     return false;

    // 11bit 通道解包
    const uint8_t *d = buffer + 1;
    uint16_t ch0  = (d[0]      | d[1]  << 8) & 0x07FF;
    uint16_t ch1  = (d[1] >> 3 | d[2]  << 5) & 0x07FF;
    uint16_t ch2  = (d[2] >> 6 | d[3]  << 2  | d[4] << 10) & 0x07FF;
    uint16_t ch3  = (d[4] >> 1 | d[5]  << 7) & 0x07FF;

    if (!IsValidChannel(ch0) || !IsValidChannel(ch1) || !IsValidChannel(ch2) || !IsValidChannel(ch3)) return false;

    return true;
}

/**
 * @brief 帧解码
 * @param buffer  原始帧数据
 * @param len     帧长度
 * @param pub     输出消息
 * @return true   解码成功
 */
bool SbusProtocol::Decode(const uint8_t* buffer, uint8_t len, Message& pub)
{
    constexpr float kInvChannelSpan = 1.0f / 660.0f;

    OutputData od{};

    // 11bit 通道解包
    const uint8_t *d = buffer + 1;
    uint16_t raw_ch0 = (d[0]      | d[1]  << 8) & 0x07FF;
    uint16_t raw_ch1 = (d[1] >> 3 | d[2]  << 5) & 0x07FF;
    uint16_t raw_ch2 = (d[2] >> 6 | d[3]  << 2  | d[4] << 10) & 0x07FF;
    uint16_t raw_ch3 = (d[4] >> 1 | d[5]  << 7) & 0x07FF;
    // uint16_t raw_ch4 = (d[5] >> 4 | d[6]  << 4) & 0x07FF;        // 未用
    uint16_t raw_ch5 = (d[6] >> 7 | d[7]  << 1  | d[8] << 9) & 0x07FF;
    uint16_t raw_ch6 = (d[8] >> 2 | d[9]  << 6) & 0x07FF;
    uint16_t raw_ch7 = (d[9] >> 5 | d[10] << 3) & 0x07FF;

    if (!IsValidChannel(raw_ch0) || !IsValidChannel(raw_ch1) || !IsValidChannel(raw_ch2) || !IsValidChannel(raw_ch3))
    {
        DUST_LOG_ERR("channel out of range");
        return false;
    }

    od.ch.chassisx  = normChannelInv(raw_ch0, kCenter, kInvChannelSpan);
    od.ch.chassisy  = normChannelInv(raw_ch1, kCenter, kInvChannelSpan);
    od.ch.yaw       = normChannelInv(raw_ch3, kCenter, kInvChannelSpan);
    od.ch.pitch     = normChannelInv(raw_ch2, kCenter, kInvChannelSpan);

    ClassifyTwoPos(raw_ch5, od.sw.sa);
    ClassifyThreePos(raw_ch6, od.sw.sb);
    ClassifyThreePos(raw_ch7, od.sw.sc);

    DefaultFillOutputData(od, pub);
    FillOutputData(od, pub);
    ValidateOutput(pub);

    return true;
}

/**
 * @brief  两档拨杆分类
 * @param v  通道原始值
 * @param status  输出 Up / Down
 */
void SbusProtocol::ClassifyTwoPos(uint16_t v, TwoPos &status)
{
    status = (v < kCenter) ? TwoPos::Up : TwoPos::Down;
}

/**
 * @brief  三档拨杆分类
 * @param v  通道原始值
 * @param status  输出 Up / Mid / Down
 */
void SbusProtocol::ClassifyThreePos(uint16_t v, ThreePos &status)
{
    if      (v == kMin)  status = ThreePos::Up;
    else if (v == kMax)  status = ThreePos::Down;
    else                 status = ThreePos::Mid;
}

/**
 * @brief 默认映射：协议输出 → 业务语义消息
 * @param od   协议解析后的内部数据
 * @param pub  输出消息
 */
void SbusProtocol::DefaultFillOutputData(const OutputData& od, Message& pub)
{
    pub.chassisx = od.ch.chassisx;
    pub.chassisy = od.ch.chassisy;
    pub.yaw      = od.ch.yaw;
    pub.pitch    = od.ch.pitch;

    switch (od.sw.sa)
    {
        case TwoPos::Up:
            pub.chassis_mode = ChassisMode::Normal;
            break;
        case TwoPos::Down:
        default:
			pub.chassis_mode = ChassisMode::Spin;
            break;
    }

    switch (od.sw.sb)
    {
        case ThreePos::Up:
            pub.shoot_ctrl  = StartMode::On;
            break;
        case ThreePos::Down:
            pub.reload_ctrl = StartMode::On;
            break;
        case ThreePos::Mid:
        default:
            pub.shoot_ctrl  = StartMode::Off;
            pub.reload_ctrl = StartMode::Off;
            break;
    }

    switch (od.sw.sc)
    {
        case ThreePos::Up:
            pub.autoaim_ctrl = StartMode::On;
            break;
        default:
            pub.autoaim_ctrl = StartMode::Off;
            break;
    }
}

/**
 * @brief 后处理入口（__weak），用户区按需覆盖
 * @param od   协议解析后的内部数据
 * @param pub  已被默认映射填好的业务消息
 */
__weak void FillOutputData(const OutputData& od, topic::remote_to::Message& pub)
{
    (void)od;
    (void)pub;
}

REGISTER_REMOTE(SbusProtocol, kFrameSizeSBUS, remote::Priority::High, 3, sbus);

}
