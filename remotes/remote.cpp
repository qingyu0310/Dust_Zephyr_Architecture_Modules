/**
 * @file remote.cpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-05-06
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "remote.hpp"
#include "remote_to.hpp"
#include <cstdint>

namespace {

using Channel = topic::remote_to::Channel;

struct Mouse
{
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float left = 0.0f, right = 0.0f;
};

template<typename Keyboard>
struct KeyboardState 
{
    Keyboard toggle_output{};
    uint16_t last_raw_all = 0;
    uint16_t keyboard_mode = 0;

    void Process(Keyboard& current_output, Keyboard& current_raw) 
    {
        uint16_t trigger = current_raw.all & (~last_raw_all);
        uint16_t toggle_mask = keyboard_mode;
        toggle_output.all ^= (trigger & toggle_mask);
        uint16_t normal_mask = ~toggle_mask;
        current_output.all = (toggle_output.all & toggle_mask) | (current_raw.all & normal_mask);
        last_raw_all = current_raw.all;
    }
};

struct Sign
{
    float x;
    float y;
};

union Keyboard
{
    uint16_t all;
    struct
    {
        uint8_t w : 1;
        uint8_t s : 1;
        uint8_t a : 1;
        uint8_t d : 1;
        uint8_t shift : 1;
        uint8_t ctrl : 1;
        uint8_t q : 1;
        uint8_t e : 1;
        uint8_t r : 1;
        uint8_t f : 1;
        uint8_t g : 1;
        uint8_t z : 1;
        uint8_t x : 1;
        uint8_t c : 1;
        uint8_t v : 1;
        uint8_t b : 1;
    };
};

/* 归一化：将 uint16_t 值映射到 [-1, 1] */
inline static float normChannel(int16_t v, int16_t center, int16_t max)
{
    float maxDist = max - center;
    float r = (static_cast<float>(v) - center) / maxDist;
    if (r > 1.0f) r = 1.0f;
    if (r < -1.0f) r = -1.0f;
    return r;
};

inline static float normMouse(float v, float scale)
{
    constexpr float kInvNorm = 1.0f / 32767.0f;

    v *= scale * kInvNorm;
    if (v > 1.0f) return 1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
}

static void processChannel(topic::remote_to::RemoteData& pub, Channel& ch, Keyboard& kb)
{
    pub.chassisy = kb.w ? 1.0f : kb.s ? -1.0f : ch.chassisy;
    pub.chassisx = kb.a ? 1.0f : kb.d ? -1.0f : ch.chassisx;
}

static void processMouse(topic::remote_to::RemoteData& pub, Channel& ch, Mouse& ms)
{
    pub.pitch = ms.y + ch.pitch;
    pub.yaw   = ms.x + ch.yaw;
}

} // namespace


namespace dr16 {

enum SwitchStatus
{
    SWITCH_UP = 1,
    SWITCH_MID = 3,
    SWITCH_DOWN = 2,
};

struct Switch
{
    uint8_t sw1;
    uint8_t sw2;
};

struct OutputData
{
    Channel ch;
    Switch sw;
    Mouse mouse;
    Keyboard keyboard;
};

static KeyboardState<Keyboard> keyboard_state_{};

static void dataprocess(uint8_t* buffer, uint8_t len)
{
    if (len < 15) return;

    OutputData od{};

    /* 摇杆通道 11bit 解包 */
    uint16_t ch0 = ( buffer[0]       | (buffer[1] << 8)) & 0x07FF;
    uint16_t ch1 = ((buffer[1] >> 3) | (buffer[2] << 5)) & 0x07FF;
    uint16_t ch2 = ((buffer[2] >> 6) | (buffer[3] << 2)  | (buffer[4] << 10)) & 0x07FF;
    uint16_t ch3 = ((buffer[4] >> 1) | (buffer[5] << 7)) & 0x07FF;

    /* 三段开关 */
    od.sw.sw1 = ((buffer[5] >> 4) & 0x0C) >> 2;
    od.sw.sw2 = ((buffer[5] >> 4) & 0x03);

    /* 鼠标原始值 */
    int16_t dx = buffer[6]  | (buffer[7] << 8);
    int16_t dy = buffer[8]  | (buffer[9] << 8);
    int16_t dz = buffer[10] | (buffer[11] << 8);

    /* 归一化 */
    constexpr int16_t kCenter = 1024;
    constexpr int16_t kMax    = 1684;

    od.ch.chassisx = normChannel(ch0, kCenter, kMax);
    od.ch.chassisy = normChannel(ch1, kCenter, kMax);
    od.ch.yaw      = normChannel(ch2, kCenter, kMax);
    od.ch.pitch    = normChannel(ch3, kCenter, kMax);

    constexpr float kMouseScaleX = 30.0f;
    constexpr float kMouseScaleY = 2.0f;
    constexpr float kMouseScaleZ = 1.0f;

    od.mouse.x     = normMouse(static_cast<float>(dx), kMouseScaleX);
    od.mouse.y     = normMouse(static_cast<float>(dy), kMouseScaleY);
    od.mouse.z     = normMouse(static_cast<float>(dz), kMouseScaleZ);

    od.mouse.left  = buffer[12] != 0;
    od.mouse.right = buffer[13] != 0;

    /* 键盘 toggle */
    Keyboard cur_raw { .all = buffer[14] };
    keyboard_state_.Process(od.keyboard, cur_raw);

    static topic::remote_to::RemoteData pub{};

    pub.chassisx    = od.ch.chassisx;
    pub.chassisy    = od.ch.chassisy;
    pub.yaw         = od.ch.yaw;
    pub.pitch       = od.ch.pitch;

    switch (od.sw.sw1)
    {
        case (SWITCH_UP):
            pub.chassis_mode = topic::remote_to::ChassisMode::Spin;
            break;
        case (SWITCH_MID):
            pub.chassis_mode = topic::remote_to::ChassisMode::Normal;
            break;
        case (SWITCH_DOWN):
            pub.chassis_mode = topic::remote_to::ChassisMode::Follow;
            break;
    }

    // pub.mouse.x     = od.mouse.x;
    // pub.mouse.y     = od.mouse.y;
    // pub.mouse.z     = od.mouse.z;
    // pub.mouse.left  = od.mouse.left;
    // pub.mouse.right = od.mouse.right;
    // pub.keyboard    = od.keyboard;
    pub.version++;
    zbus_chan_pub(&pub_remote_to, &pub, K_MSEC(1));
}

} // namespace dr16


namespace vt12 {

struct OutputData
{
    Mouse mouse;
    Keyboard keyboard;
};

static KeyboardState<Keyboard> keyboard_state_{};

static void dataprocess(uint8_t* buffer, uint8_t len)
{
    if (len < 16) return;

    /* 协议头 */
    uint8_t  start_of_frame = buffer[0];
    uint16_t data_length    = (uint16_t)buffer[1] | ((uint16_t)buffer[2] << 8);
    uint8_t  seq            = buffer[3];
    uint8_t  crc8           = buffer[4];
    uint8_t  cmd_id         = buffer[5];

    OutputData od{};

    int16_t dx = (int16_t)((uint16_t)buffer[6]  | ((uint16_t)buffer[7]  << 8));
    int16_t dy = (int16_t)((uint16_t)buffer[8]  | ((uint16_t)buffer[9]  << 8));
    int16_t dz = (int16_t)((uint16_t)buffer[10] | ((uint16_t)buffer[11] << 8));

    constexpr float kMouseScaleX = 30.0f;
    constexpr float kMouseScaleY = 2.0f;
    constexpr float kMouseScaleZ = 1.0f;

    od.mouse.x      = normMouse(static_cast<float>(dx), kMouseScaleX);
    od.mouse.y      = normMouse(static_cast<float>(dy), kMouseScaleY);
    od.mouse.z      = normMouse(static_cast<float>(dz), kMouseScaleZ);
    od.mouse.left   = buffer[12] != 0;
    od.mouse.right  = buffer[13] != 0;

    Keyboard cur_raw { .all = static_cast<uint16_t>((uint16_t)buffer[14] | ((uint16_t)buffer[15] << 8)) };
    keyboard_state_.Process(od.keyboard, cur_raw);

    static topic::remote_to::RemoteData pub{};

    pub.chassis_mode = topic::remote_to::ChassisMode::Normal;
    // pub.mouse.x      = od.mouse.x;
    // pub.mouse.y      = od.mouse.y;
    // pub.mouse.z      = od.mouse.z;
    // pub.mouse.left   = od.mouse.left;
    // pub.mouse.right  = od.mouse.right;
    // pub.keyboard     = od.keyboard;
    pub.version++;
    zbus_chan_pub(&pub_remote_to, &pub, K_MSEC(1));
}

} // namespace vt12

namespace vt13 {

struct RawData
{
    uint8_t start_of_frame_1 = 0xA9;
    uint8_t start_of_frame_2 = 0x53;
    uint64_t channel_0 : 11;
    uint64_t channel_1 : 11;
    uint64_t channel_2 : 11;
    uint64_t channel_3 : 11;
    uint64_t cns    : 2;
    uint64_t pause  : 1;
    uint64_t fn_1   : 1;
    uint64_t fn_2   : 1;
    uint64_t wheel  : 11;
    uint64_t trigger    : 1;
    uint64_t reserved_1 : 3;
    int16_t  mouse_x;
    int16_t  mouse_y;
    int16_t  mouse_z;
    uint8_t  mouse_l : 2;
    uint8_t  mouse_r : 2;
    uint8_t  mouse_m : 2;
    uint8_t  reserved_2 : 2;
    uint16_t keyboard;
    uint16_t crc16;
} __attribute__((packed));

struct Switch
{
    uint8_t fn1, fn2;
    uint8_t trigger;
    uint8_t pause;
    uint8_t cns;
    float wheel;
};

struct OutputData
{
    Channel ch;
    Switch sw;
    Mouse mouse;
    Keyboard keyboard;
};

static KeyboardState<Keyboard> keyboard_state_{};

static void dataprocess(uint8_t* buffer, uint8_t len)
{
    const RawData* raw_data = reinterpret_cast<RawData const*>(buffer);

    if (raw_data->start_of_frame_1 != 0xA9 && raw_data->start_of_frame_2 != 0x53) {
        return;
    }

    OutputData od{};

    constexpr int16_t kCenter = 1024;
    constexpr int16_t kMax    = 1684;

    od.ch.chassisx  = normChannel(raw_data->channel_0, kCenter, kMax);
    od.ch.chassisy  = normChannel(raw_data->channel_1, kCenter, kMax);
    od.ch.yaw       = normChannel(raw_data->channel_2, kCenter, kMax);
    od.ch.pitch     = normChannel(raw_data->channel_3, kCenter, kMax);

    od.sw.fn1       = raw_data->fn_1;
    od.sw.fn2       = raw_data->fn_2;
    od.sw.trigger   = raw_data->trigger;
    od.sw.pause     = raw_data->pause;
    od.sw.cns       = raw_data->cns;
    od.sw.wheel     = normChannel(raw_data->wheel, kCenter, kMax);

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

    static topic::remote_to::RemoteData pub{};
    pub.chassisx = od.ch.chassisx;
    pub.chassisy = od.ch.chassisy;
    pub.yaw      = od.ch.yaw;
    pub.pitch    = od.ch.pitch;
    // pub.chassis_mode = od.sw.fn1;
    // pub.chassis_follow = od.sw.fn2;
    // pub.mouse.x  = od.mouse.x;
    // pub.mouse.y  = od.mouse.y;
    // pub.mouse.z  = od.mouse.z;
    // pub.mouse.left  = od.mouse.left;
    // pub.mouse.right = od.mouse.right;
    // pub.keyboard.all = od.keyboard.all;
    pub.version++;
    zbus_chan_pub(&pub_remote_to, &pub, K_MSEC(1));
}

} // namespace vt13


namespace thread::remote {

void Remote::GetProcessFunc()
{
    switch (type_) 
    {
        case RemoteType::DR16:
            DataProcess = dr16::dataprocess;
            break;
        case RemoteType::VT12:
            DataProcess = vt12::dataprocess;
            break;
        case RemoteType::VT13:
            DataProcess = vt13::dataprocess;
            break;
        default:
            DataProcess = nullptr;
            break;
    }
}

}