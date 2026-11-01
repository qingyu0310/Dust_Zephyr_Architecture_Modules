/**
 * @file vt13.cpp
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
#include "vt13.hpp"
#include "log.hpp"

using namespace topic::remote_to;


namespace vt13 {

static constexpr uint16_t kFrameSizeVT13 = 21;

class Vt13Protocol final : public remote::RemoteProtocol
{
public:
    Vt13Protocol()
    {
        line_cfg_.baudrate  = 921600;
        line_cfg_.parity    = UART_CFG_PARITY_NONE;
        line_cfg_.stop_bits = UART_CFG_STOP_BITS_1;
        line_cfg_.data_bits = UART_CFG_DATA_BITS_8;
        line_cfg_.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
    }

    bool Validate(const uint8_t *buffer, uint8_t len) override;
    bool Decode(const uint8_t *buffer, uint8_t len, topic::remote_to::Message &pub) override;

private:
    static constexpr uint8_t  kSoF1       = 0xA9;
    static constexpr uint8_t  kSoF2       = 0x53;
    static constexpr uint16_t kMinChannel = 364;
    static constexpr uint16_t kMaxChannel = 1684;
    
    struct RawData {
        uint8_t  start_of_frame_1;
        uint8_t  start_of_frame_2;
        uint64_t channel_0  : 11;
        uint64_t channel_1  : 11;
        uint64_t channel_2  : 11;
        uint64_t channel_3  : 11;
        uint64_t cns        : 2;
        uint64_t pause      : 1;
        uint64_t fn_1       : 1;
        uint64_t fn_2       : 1;
        uint64_t wheel      : 11;
        uint64_t trigger    : 1;
        uint64_t reserved_1 : 3;
        int16_t  mouse_x;
        int16_t  mouse_y;
        int16_t  mouse_z;
        uint8_t  mouse_l    : 2;
        uint8_t  mouse_r    : 2;
        uint8_t  mouse_m    : 2;
        uint8_t  reserved_2 : 2;
        uint16_t keyboard;
        uint16_t crc16;
    } __attribute__((packed));

    static bool IsValidChannel(const RawData *raw);
    static void DefaultFillOutputData(const OutputData &od, topic::remote_to::Message &pub);
};

/**
 * @brief 帧校验
 * @param buffer  原始帧数据
 * @param len     帧长度
 * @return true   校验通过
 */
bool Vt13Protocol::Validate(const uint8_t* buffer, uint8_t len)
{
    if (len != kFrameSizeVT13) return false;
    if (buffer[0] != kSoF1 || buffer[1] != kSoF2) return false;

    const RawData* raw_data = reinterpret_cast<RawData const*>(buffer);

    return IsValidChannel(raw_data);
}

/**
 * @brief 帧解码
 * @param buffer  原始帧数据
 * @param len     帧长度
 * @param pub     输出消息
 * @return true   解码成功
 */
bool Vt13Protocol::Decode(const uint8_t* buffer, uint8_t len, Message& pub)
{
    const RawData* raw_data = reinterpret_cast<RawData const*>(buffer);

    if (raw_data->start_of_frame_1 != kSoF1 || raw_data->start_of_frame_2 != kSoF2)
    {
        DUST_LOG_ERR("invalid header");
        return false;
    }

    if (!IsValidChannel(raw_data))
    {
        DUST_LOG_ERR("channel out of range");
        return false;
    }

    OutputData od{};

    constexpr int16_t kCenter = 1024;
    constexpr float   kInvChannelSpan = 1.0f / 660.0f;

    od.ch.chassisx  = normChannelInv(raw_data->channel_0, kCenter, kInvChannelSpan);
    od.ch.chassisy  = normChannelInv(raw_data->channel_1, kCenter, kInvChannelSpan);
    od.ch.yaw       = normChannelInv(raw_data->channel_2, kCenter, kInvChannelSpan);
    od.ch.pitch     = normChannelInv(raw_data->channel_3, kCenter, kInvChannelSpan);

    od.sw.fn1       = raw_data->fn_1;
    od.sw.fn2       = raw_data->fn_2;
    od.sw.trigger   = raw_data->trigger;
    od.sw.pause     = raw_data->pause;
    od.sw.cns       = raw_data->cns;
    od.sw.wheel     = normChannelInv(raw_data->wheel, kCenter, kInvChannelSpan);

    od.mouse.x      = normMouse(static_cast<float>(raw_data->mouse_x));
    od.mouse.y      = normMouse(static_cast<float>(raw_data->mouse_y));
    od.mouse.z      = normMouse(static_cast<float>(raw_data->mouse_z));

    od.mouse.left   = raw_data->mouse_l != 0;
    od.mouse.right  = raw_data->mouse_r != 0;

    od.keyboard.all = raw_data->keyboard;    // 原始按键位，长按/切换由映射层处理

    DefaultFillOutputData(od, pub);
    FillOutputData(od, pub);
    ValidateOutput(pub);

    return true;
}

/**
 * @brief 通道值范围校验
 * @param raw  原始帧数据
 * @return true  范围合法
 */
bool Vt13Protocol::IsValidChannel(const RawData *raw)
{
    if (raw->channel_0 < kMinChannel || raw->channel_0 > kMaxChannel) return false;
    if (raw->channel_1 < kMinChannel || raw->channel_1 > kMaxChannel) return false;
    if (raw->channel_2 < kMinChannel || raw->channel_2 > kMaxChannel) return false;
    if (raw->channel_3 < kMinChannel || raw->channel_3 > kMaxChannel) return false;
    if (raw->wheel     < kMinChannel || raw->wheel     > kMaxChannel) return false;
    return true;
}

/**
 * @brief 默认映射：协议输出 → 业务语义消息
 * @param od   协议解析后的内部数据
 * @param pub  输出消息
 */
void Vt13Protocol::DefaultFillOutputData(const OutputData& od, Message& pub)
{
    constexpr float    kMouseSensitivityX = 30.0f;
    constexpr float    kMouseSensitivityY = 2.0f;
    constexpr uint16_t kKeyboardMode      = 0;    // 全键长按；改位掩码即可让指定键走切换

    static KeyboardState keyboard_state_ { kKeyboardMode };

    Keyboard kb {};

    keyboard_state_.Process(kb, od.keyboard);

    pub.chassisy = kb.w ? 1.0f : kb.s ? -1.0f : od.ch.chassisy;
    pub.chassisx = kb.a ? 1.0f : kb.d ? -1.0f : od.ch.chassisx;
    pub.pitch    = ClampAxis(od.mouse.y * kMouseSensitivityY + od.ch.pitch);
    pub.yaw      = ClampAxis(od.mouse.x * kMouseSensitivityX + od.ch.yaw);

    switch (od.sw.cns)
    {
        case SWITCH_C:
            pub.chassis_mode = ChassisMode::Spin;
            break;
        case SWITCH_N:
            pub.chassis_mode = ChassisMode::Normal;
            break;
        case SWITCH_S:
            break;
        default:
            pub.chassis_mode = ChassisMode::Normal;
            break;
    }

    pub.reload_ctrl   = od.sw.fn1 ? StartMode::On : StartMode::Off;
    pub.shoot_ctrl    = od.sw.fn2 ? StartMode::On : StartMode::Off;
    pub.autoaim_ctrl  = od.sw.fn2 ? StartMode::On : StartMode::Off;
    pub.supercap_ctrl = kb.v      ? StartMode::On : StartMode::Off;
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

REGISTER_REMOTE(Vt13Protocol, kFrameSizeVT13, remote::Priority::High, 3, vt13);

}
