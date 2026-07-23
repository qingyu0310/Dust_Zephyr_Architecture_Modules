/**
 * @file vt12.cpp
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

LOG_MODULE_REGISTER(vt12, LOG_LEVEL_INF);

namespace vt12 {

static constexpr uint16_t kFrameSizeVT12 = 16;

class Vt12Protocol final : public remote::RemoteProtocol
{
public:
    Vt12Protocol()
    {
        line_cfg_.baudrate  = 115200;
        line_cfg_.parity    = UART_CFG_PARITY_NONE;
        line_cfg_.stop_bits = UART_CFG_STOP_BITS_1;
        line_cfg_.data_bits = UART_CFG_DATA_BITS_8;
        line_cfg_.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
    }

    bool Validate(const uint8_t *buffer, uint8_t len) override;
    bool Decode(const uint8_t *buffer, uint8_t len, topic::remote_to::Message &pub) override;

private:
    struct OutputData {
        Mouse    mouse;
        Keyboard keyboard;
    };

    static void FillOutputData(const OutputData &od, topic::remote_to::Message &pub);
};

/**
 * @brief 帧校验
 * @param buffer  原始帧数据
 * @param len     帧长度
 * @return true   校验通过
 */
bool Vt12Protocol::Validate(const uint8_t* buffer, uint8_t len)
{
    if (len != kFrameSizeVT12) return false;
    if (buffer[0] != 0xA5) return false;

    // 鼠标差值合理性
    int16_t dx = (int16_t)((uint16_t)buffer[6]  | ((uint16_t)buffer[7]  << 8));
    int16_t dy = (int16_t)((uint16_t)buffer[8]  | ((uint16_t)buffer[9]  << 8));
    int16_t dz = (int16_t)((uint16_t)buffer[10] | ((uint16_t)buffer[11] << 8));

    constexpr int16_t kMouseMax = 32767;
    if (dx < -kMouseMax || dx > kMouseMax) return false;
    if (dy < -kMouseMax || dy > kMouseMax) return false;
    if (dz < -kMouseMax || dz > kMouseMax) return false;

    // 鼠标按键值合法
    if (buffer[12] > 1) return false;
    if (buffer[13] > 1) return false;

    return true;
}

/**
 * @brief 帧解码
 * @param buffer  原始帧数据
 * @param len     帧长度
 * @param pub     输出消息
 * @return true   解码成功
 */
bool Vt12Protocol::Decode(const uint8_t* buffer, uint8_t len, Message& pub)
{
    static KeyboardState keyboard_state_{};

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

    FillOutputData(od, pub);
    return true;
}

/**
 * @brief 填充输出数据到共享消息
 * @param od   协议解析后的内部数据
 * @param pub  输出消息
 */
void Vt12Protocol::FillOutputData(const OutputData& od, Message& pub)
{
    pub.chassisy = od.keyboard.w ? 1.0f : od.keyboard.s ? -1.0f : 0.0f;
    pub.chassisx = od.keyboard.a ? 1.0f : od.keyboard.d ? -1.0f : 0.0f;
    pub.pitch    = od.mouse.y;
    pub.yaw      = od.mouse.x;

    if (od.keyboard.shift) {
        pub.chassis_mode = ChassisMode::Spin;
    }  else {
        pub.chassis_mode = ChassisMode::Normal;
    }

    pub.shoot_ctrl    = od.keyboard.f  ? StartMode::On : StartMode::Off;
    pub.reload_ctrl   = od.mouse.left  ? StartMode::On : StartMode::Off;
    pub.autoaim_ctrl  = od.mouse.right ? StartMode::On : StartMode::Off;
    pub.supercap_ctrl = od.keyboard.v  ? StartMode::On : StartMode::Off;

    pub.version++;
}

REGISTER_REMOTE(Vt12Protocol, kFrameSizeVT12, remote::Priority::Medium, 3, vt12);

}
