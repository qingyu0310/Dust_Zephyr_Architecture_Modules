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
#include <zephyr/logging/log.h>

using namespace topic::remote_to;

LOG_MODULE_REGISTER(vt13, LOG_LEVEL_INF);

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
    static constexpr uint8_t  kSOF1       = 0xA9;
    static constexpr uint8_t  kSOF2       = 0x53;
    static constexpr uint16_t kMinChannel = 364;
    static constexpr uint16_t kMaxChannel = 1684;
    
    enum SwitchCNS { 
        SWITCH_C = 0, 
        SWITCH_N, 
        SWITCH_S 
    };

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

    struct Switch {
        uint8_t fn1, fn2;
        uint8_t trigger;
        uint8_t pause;
        uint8_t cns;
        float   wheel;
    };

    struct OutputData {
        topic::remote_to::Channel  ch;
        Switch                     sw;
        Mouse    mouse;
        Keyboard keyboard;
    };

    static bool IsValidChannel(const RawData *raw);
    static void FillOutputData(const OutputData &od, topic::remote_to::Message &pub);
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
    if (buffer[0] != kSOF1 || buffer[1] != kSOF2) return false;

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

    if (raw_data->start_of_frame_1 != kSOF1 || raw_data->start_of_frame_2 != kSOF2)
    {
        LOG_ERR("invalid header");
        return false;
    }

    if (!IsValidChannel(raw_data))
    {
        LOG_ERR("channel out of range");
        return false;
    }

    static KeyboardState keyboard_state_{};

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

    constexpr float kMouseScaleX = 30.0f;
    constexpr float kMouseScaleY = 2.0f;
    constexpr float kMouseScaleZ = 1.0f;

    od.mouse.x      = normMouse(static_cast<float>(raw_data->mouse_x), kMouseScaleX);
    od.mouse.y      = normMouse(static_cast<float>(raw_data->mouse_y), kMouseScaleY);
    od.mouse.z      = normMouse(static_cast<float>(raw_data->mouse_z), kMouseScaleZ);

    od.mouse.left   = raw_data->mouse_l != 0;
    od.mouse.right  = raw_data->mouse_r != 0;

    Keyboard cur_raw { .all = raw_data->keyboard };
    keyboard_state_.Process(od.keyboard, cur_raw);

    FillOutputData(od, pub);
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
 * @brief 填充输出数据到共享消息
 * @param od   协议解析后的内部数据
 * @param pub  输出消息
 */
void Vt13Protocol::FillOutputData(const OutputData& od, Message& pub)
{
    processChannel(pub, od);

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

    pub.reload_ctrl   = od.sw.fn1     ? StartMode::On : StartMode::Off;
    pub.shoot_ctrl    = od.sw.fn2     ? StartMode::On : StartMode::Off;
    pub.autoaim_ctrl  = od.sw.fn2     ? StartMode::On : StartMode::Off;
    pub.supercap_ctrl = od.keyboard.v ? StartMode::On : StartMode::Off;

    pub.version++;
}

REGISTER_REMOTE(Vt13Protocol, kFrameSizeVT13, remote::Priority::High, 3, vt13);

}
