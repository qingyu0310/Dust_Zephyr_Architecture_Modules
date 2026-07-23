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
#include <zephyr/logging/log.h>

using namespace topic::remote_to;

LOG_MODULE_REGISTER(sbus, LOG_LEVEL_INF);

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

    enum SwitchStatus {
        SWITCH_UP   = 1,
        SWITCH_MID  = 3,
        SWITCH_DOWN = 2,
    };
    
    struct Switch {
        SwitchStatus sw1;
        SwitchStatus sw2;
        SwitchStatus sw3;
    };

    struct OutputData {
        topic::remote_to::Channel  ch;
        Switch                     sw;
    };

    static bool  IsValidChannel(uint16_t v) { return v >= kMin && v <= kMax; }
    static void  ClassifySwitch(uint16_t v, SwitchStatus &status);
    static void  FillOutputData(const OutputData &od, topic::remote_to::Message &pub);
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

    if (!IsValidChannel(ch0) || !IsValidChannel(ch1) ||
        !IsValidChannel(ch2) || !IsValidChannel(ch3)) return false;

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

    if (!IsValidChannel(raw_ch0) || !IsValidChannel(raw_ch1) ||
        !IsValidChannel(raw_ch2) || !IsValidChannel(raw_ch3))
    {
        LOG_ERR("channel out of range");
        return false;
    }

    od.ch.chassisx  = normChannelInv(raw_ch0, kCenter, kInvChannelSpan);
    od.ch.chassisy  = normChannelInv(raw_ch1, kCenter, kInvChannelSpan);
    od.ch.yaw       = normChannelInv(raw_ch3, kCenter, kInvChannelSpan);
    od.ch.pitch     = normChannelInv(raw_ch2, kCenter, kInvChannelSpan);

    ClassifySwitch(raw_ch5, od.sw.sw1);
    ClassifySwitch(raw_ch6, od.sw.sw2);
    ClassifySwitch(raw_ch7, od.sw.sw3);

    FillOutputData(od, pub);
    return true;
}

/**
 * @brief  根据原始值三档分类拨杆位置
 * @param v  通道原始值
 * @return SwitchStatus  SWITCH_UP / SWITCH_MID / SWITCH_DOWN
 */
void SbusProtocol::ClassifySwitch(uint16_t v, SwitchStatus &status)
{
    if      (v == kMin)  status = SWITCH_UP;
    else if (v == kMax)  status = SWITCH_DOWN;
    else                 status = SWITCH_MID;
}

/**
 * @brief 填充输出数据到共享消息
 * @param od   协议解析后的内部数据
 * @param pub  输出消息
 */
void SbusProtocol::FillOutputData(const OutputData& od, Message& pub)
{
    pub.chassisx = od.ch.chassisx;
    pub.chassisy = od.ch.chassisy;
    pub.yaw      = od.ch.yaw;
    pub.pitch    = od.ch.pitch;

    switch (od.sw.sw1)
    {
        case SWITCH_UP:
            pub.chassis_mode = ChassisMode::Spin;
            break;
        case SWITCH_MID:
        default:
            pub.chassis_mode = ChassisMode::Normal;
            break;
    }

    switch (od.sw.sw2)
    {
        case SWITCH_UP:
            pub.shoot_ctrl  = StartMode::On;
            break;
        case SWITCH_DOWN:
            pub.reload_ctrl = StartMode::On;
            break;
        case SWITCH_MID:
        default:
            pub.shoot_ctrl  = StartMode::Off;
            pub.reload_ctrl = StartMode::Off;
            break;
    }

    switch (od.sw.sw3)
    {
        case SWITCH_UP:
            pub.autoaim_ctrl = StartMode::On;
            break;
        default:
            pub.autoaim_ctrl = StartMode::Off;
            break;
    }

    pub.version++;
}

REGISTER_REMOTE(SbusProtocol, kFrameSizeSBUS, remote::Priority::High, 3, sbus);

}
