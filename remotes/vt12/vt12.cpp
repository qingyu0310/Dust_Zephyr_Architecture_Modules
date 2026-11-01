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
#include "vt12.hpp"

using namespace topic::remote_to;


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
    static void DefaultFillOutputData(const OutputData &od, topic::remote_to::Message &pub);
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
    OutputData od{};

    int16_t dx = (int16_t)((uint16_t)buffer[6]  | ((uint16_t)buffer[7]  << 8));
    int16_t dy = (int16_t)((uint16_t)buffer[8]  | ((uint16_t)buffer[9]  << 8));
    int16_t dz = (int16_t)((uint16_t)buffer[10] | ((uint16_t)buffer[11] << 8));

    od.mouse.x      = normMouse(static_cast<float>(dx));
    od.mouse.y      = normMouse(static_cast<float>(dy));
    od.mouse.z      = normMouse(static_cast<float>(dz));
    od.mouse.left   = buffer[12] != 0;
    od.mouse.right  = buffer[13] != 0;

    od.keyboard.all = static_cast<uint16_t>((uint16_t)buffer[14] | ((uint16_t)buffer[15] << 8));    // 原始按键位，长按/切换由映射层处理

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
void Vt12Protocol::DefaultFillOutputData(const OutputData& od, Message& pub)
{
    constexpr float kMouseSensitivityX = 30.0f;
    constexpr float kMouseSensitivityY = 2.0f;
    constexpr uint16_t kKeyboardMode = 0;    // 全键长按；改位掩码即可让指定键走切换

    static KeyboardState keyboard_state_ { kKeyboardMode };

    Keyboard kb {};

    keyboard_state_.Process(kb, od.keyboard);

    pub.chassisy = kb.w ? 1.0f : kb.s ? -1.0f : 0.0f;
    pub.chassisx = kb.a ? 1.0f : kb.d ? -1.0f : 0.0f;
    pub.pitch    = ClampAxis(od.mouse.y * kMouseSensitivityY);
    pub.yaw      = ClampAxis(od.mouse.x * kMouseSensitivityX);

    if (kb.shift) {
        pub.chassis_mode = ChassisMode::Spin;
    }  else {
        pub.chassis_mode = ChassisMode::Normal;
    }

    pub.shoot_ctrl    = kb.f           ? StartMode::On : StartMode::Off;
    pub.reload_ctrl   = od.mouse.left  ? StartMode::On : StartMode::Off;
    pub.autoaim_ctrl  = od.mouse.right ? StartMode::On : StartMode::Off;
    pub.supercap_ctrl = kb.v           ? StartMode::On : StartMode::Off;
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

REGISTER_REMOTE(Vt12Protocol, kFrameSizeVT12, remote::Priority::Medium, 3, vt12);

}
