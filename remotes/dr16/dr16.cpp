/**
 * @file dr16.cpp
 * @author qingyu
 * @brief DR16 协议解析实现
 * @version 0.1
 * @date 2026-05-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "dr16.hpp"
#include "remote_to.hpp"

namespace thread::remote {

void DR16::DataProcess(uint8_t* buffer, uint8_t len)
{
    if (len < 15) return;

    /* 摇杆通道 11bit 解包 */
    uint16_t ch0 = ( buffer[0]       | (buffer[1] << 8)) & 0x07FF;
    uint16_t ch1 = ((buffer[1] >> 3) | (buffer[2] << 5)) & 0x07FF;
    uint16_t ch2 = ((buffer[2] >> 6) | (buffer[3] << 2) | (buffer[4] << 10)) & 0x07FF;
    uint16_t ch3 = ((buffer[4] >> 1) | (buffer[5] << 7)) & 0x07FF;

    /* 三段开关 */
    output_.sw.right = ((buffer[5] >> 4) & 0x0C) >> 2;
    output_.sw.left  = ((buffer[5] >> 4) & 0x03);

    /* 鼠标原始值 */
    int16_t dx = buffer[6] | (buffer[7] << 8);
    int16_t dy = buffer[8] | (buffer[9] << 8);
    int16_t dz = buffer[10] | (buffer[11] << 8);

    /* 归一化 */
    auto normCh = [](uint16_t v)  {
        constexpr float kMax = 660.0f;
        constexpr int16_t kCenter = 1024;
        float r = (static_cast<float>(v) - kCenter) / kMax;
        if (r > 1.0f) r = 1.0f;
        if (r < -1.0f) r = -1.0f;
        return r;
    };

    constexpr float kInvNorm = 1.0f / 32767.0f;

    auto norm = [](float v, float scale = 1.0f)  {
        v *= scale * kInvNorm;
        if (v > 1.0f) return 1.0f;
        if (v < -1.0f) return -1.0f;
        return v;
    };

    output_.axis.x = normCh(ch0);
    output_.axis.y = normCh(ch1);
    output_.axis.yaw   = normCh(ch2);
    output_.axis.pitch = normCh(ch3);

    output_.mouse.x = norm(static_cast<float>(dx), 30.0f);
    output_.mouse.y = norm(static_cast<float>(dy), 2.0f);
    output_.mouse.z = norm(static_cast<float>(dz), 1.0f);

    output_.mouse.left  = buffer[12] != 0;
    output_.mouse.right = buffer[13] != 0;

    /* 键盘 */
    DR16Keyboard cur_raw { .all = buffer[14] };
    Process_Keyboard_Toggle(output_.keyboard, cur_raw);

    zbus_chan_pub(&pub_dr16_to, &output_, K_MSEC(1));
}

void DR16::ClearData()
{
    output_.axis  = {};
    output_.sw    = {};
    output_.mouse = {};
    output_.keyboard = {};
}

}
