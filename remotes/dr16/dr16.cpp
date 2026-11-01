/**
 * @file dr16.cpp
 * @author qingyu
 * @brief
 * @version 0.1
 * @date 2026-05-12
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "protocol_base.hpp"
#include "remote.hpp"
#include "dr16.hpp"
#include "log.hpp"

using namespace topic::remote_to;

namespace dr16 {

static constexpr uint16_t kFrameSizeDR16 = 18;

class Dr16Protocol final : public remote::RemoteProtocol
{
public:
    Dr16Protocol()
    {
        line_cfg_.baudrate  = 100000;
        line_cfg_.parity    = UART_CFG_PARITY_EVEN;
        line_cfg_.stop_bits = UART_CFG_STOP_BITS_1;
        line_cfg_.data_bits = UART_CFG_DATA_BITS_8;
        line_cfg_.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
    }

    bool Validate(const uint8_t *buffer, uint8_t len) override;
    bool Decode(const uint8_t *buffer, uint8_t len, topic::remote_to::Message &pub) override;

private:
    static constexpr uint16_t kMinChannel = 364;
    static constexpr uint16_t kMaxChannel = 1684;

    static bool IsValidChannel(uint16_t ch0, uint16_t ch1, uint16_t ch2, uint16_t ch3);
    static bool IsValidSwitch(uint8_t sw1, uint8_t sw2);
    static void DefaultFillOutputData(const OutputData &od, topic::remote_to::Message &pub);
};

/**
 * @brief 帧校验 — Remote 协议锁定用，做完整内容校验
 * @param buffer  原始帧数据
 * @param len     帧长度
 * @return true   校验通过
 */
bool Dr16Protocol::Validate(const uint8_t* buffer, uint8_t len)
{
    if (len != kFrameSizeDR16) return false;

    uint16_t ch0 = ( buffer[0]       | (buffer[1] << 8)) & 0x07FF;
    uint16_t ch1 = ((buffer[1] >> 3) | (buffer[2] << 5)) & 0x07FF;
    uint16_t ch2 = ((buffer[2] >> 6) | (buffer[3] << 2)  | (buffer[4] << 10)) & 0x07FF;
    uint16_t ch3 = ((buffer[4] >> 1) | (buffer[5] << 7)) & 0x07FF;

    if (!IsValidChannel(ch0, ch1, ch2, ch3)) return false;

    uint8_t sw1 = ((buffer[5] >> 4) & 0x0C) >> 2;
    uint8_t sw2 = ((buffer[5] >> 4) & 0x03);

    return IsValidSwitch(sw1, sw2);
}

/**
 * @brief 帧解码
 * @param buffer  原始帧数据
 * @param len     帧长度
 * @param pub     输出消息
 * @return true   解码成功
 */
bool Dr16Protocol::Decode(const uint8_t* buffer, uint8_t len, Message& pub)
{
    uint16_t ch0 = ( buffer[0]       | (buffer[1] << 8)) & 0x07FF;
    uint16_t ch1 = ((buffer[1] >> 3) | (buffer[2] << 5)) & 0x07FF;
    uint16_t ch2 = ((buffer[2] >> 6) | (buffer[3] << 2)  | (buffer[4] << 10)) & 0x07FF;
    uint16_t ch3 = ((buffer[4] >> 1) | (buffer[5] << 7)) & 0x07FF;

    if (!IsValidChannel(ch0, ch1, ch2, ch3))
    {
        DUST_LOG_ERR("channel out of range");
        return false;
    }

    uint8_t sw1 = ((buffer[5] >> 4) & 0x0C) >> 2;
    uint8_t sw2 = ((buffer[5] >> 4) & 0x03);

    if (!IsValidSwitch(sw1, sw2))
    {
        DUST_LOG_ERR("invalid switch");
        return false;
    }

    OutputData od{};

    od.sw.sw1 = sw1;
    od.sw.sw2 = sw2;

    int16_t dx   = buffer[6]  | (buffer[7] << 8);
    int16_t dy   = buffer[8]  | (buffer[9] << 8);
    int16_t dz   = buffer[10] | (buffer[11] << 8);

    constexpr int16_t kCenter = 1024;
    constexpr float   kInvChannelSpan = 1.0f / 660.0f;

    od.ch.chassisx = normChannelInv(ch0, kCenter, kInvChannelSpan);
    od.ch.chassisy = normChannelInv(ch1, kCenter, kInvChannelSpan);
    od.ch.yaw      = normChannelInv(ch2, kCenter, kInvChannelSpan);
    od.ch.pitch    = normChannelInv(ch3, kCenter, kInvChannelSpan);

    od.mouse.x     = normMouse(static_cast<float>(dx));
    od.mouse.y     = normMouse(static_cast<float>(dy));
    od.mouse.z     = normMouse(static_cast<float>(dz));

    od.mouse.left  = buffer[12] != 0;
    od.mouse.right = buffer[13] != 0;

    od.keyboard.all = buffer[14];    // 原始按键位，长按/切换由映射层处理

    DefaultFillOutputData(od, pub);
    FillOutputData(od, pub);
    ValidateOutput(pub);

    return true;
}

/**
 * @brief 默认映射：协议输出 → 业务语义消息
 * @param od   协议解析后的内部数据
 * @param pub  输出消息
 */
void Dr16Protocol::DefaultFillOutputData(const OutputData& od, Message& pub)
{
    constexpr float kMouseSensitivityX = 30.0f;
    constexpr float kMouseSensitivityY = 2.0f;
    constexpr uint16_t kKeyboardMode = 0;    // 全键长按；改位掩码即可让指定键走切换

    static KeyboardState keyboard_state_ { kKeyboardMode };

    Keyboard kb {};

    keyboard_state_.Process(kb, od.keyboard);

    pub.chassisy = kb.w ? 1.0f : kb.s ? -1.0f : od.ch.chassisy;
    pub.chassisx = kb.a ? 1.0f : kb.d ? -1.0f : od.ch.chassisx;
    pub.pitch    = ClampAxis(od.mouse.y * kMouseSensitivityY + od.ch.pitch);
    pub.yaw      = ClampAxis(od.mouse.x * kMouseSensitivityX + od.ch.yaw);

    switch (od.sw.sw1)
    {
        case SWITCH_UP:
            pub.shoot_ctrl = StartMode::On;
            break;
        default:
            pub.shoot_ctrl = StartMode::Off;
            break;
    }

    switch (od.sw.sw2)
    {
        case SWITCH_UP:
            pub.reload_ctrl = StartMode::On;
            break;
        default:
            pub.reload_ctrl = StartMode::Off;
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

/**
 * @brief 通道值范围校验
 * @param ch0-ch3  四个通道原始值
 * @return true    范围合法
 */
bool Dr16Protocol::IsValidChannel(uint16_t ch0, uint16_t ch1, uint16_t ch2, uint16_t ch3)
{
    if (ch0 < kMinChannel || ch0 > kMaxChannel) return false;
    if (ch1 < kMinChannel || ch1 > kMaxChannel) return false;
    if (ch2 < kMinChannel || ch2 > kMaxChannel) return false;
    if (ch3 < kMinChannel || ch3 > kMaxChannel) return false;

    return true;
}

/**
 * @brief 拨杆位置合法性校验
 * @param sw1-sw2  拨杆原始值
 * @return true    位置合法
 */
bool Dr16Protocol::IsValidSwitch(uint8_t sw1, uint8_t sw2)
{
    if (sw1 != SWITCH_UP && sw1 != SWITCH_MID && sw1 != SWITCH_DOWN) return false;
    if (sw2 != SWITCH_UP && sw2 != SWITCH_MID && sw2 != SWITCH_DOWN) return false;

    return true;
}

REGISTER_REMOTE(Dr16Protocol, kFrameSizeDR16, remote::Priority::Low, 3, dr16);

}
