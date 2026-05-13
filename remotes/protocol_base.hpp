#pragma once

#include <cstdint>
#include "remote_to.hpp"

/*  共用类型 — 各协议解析的内部数据结构                                    */
struct Mouse
{
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float left = 0.0f, right = 0.0f;
};

struct Keyboard {
    uint16_t all;

    bool w()     const { return all & (1 << 0); }
    bool s()     const { return all & (1 << 1); }
    bool a()     const { return all & (1 << 2); }
    bool d()     const { return all & (1 << 3); }
    bool shift() const { return all & (1 << 4); }
    bool ctrl()  const { return all & (1 << 5); }
    bool q()     const { return all & (1 << 6); }
    bool e()     const { return all & (1 << 7); }
    bool r()     const { return all & (1 << 8); }
    bool f()     const { return all & (1 << 9); }
    bool g()     const { return all & (1 << 10); }
    bool z()     const { return all & (1 << 11); }
    bool x()     const { return all & (1 << 12); }
    bool c()     const { return all & (1 << 13); }
    bool v()     const { return all & (1 << 14); }
    bool b()     const { return all & (1 << 15); }
};

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

/*  归一化函数                                                              */
inline float normChannel(int16_t v, int16_t center, int16_t max)
{
    float maxDist = max - center;
    float r = (static_cast<float>(v) - center) / maxDist;
    if (r >  1.0f) r =  1.0f;
    if (r < -1.0f) r = -1.0f;
    return r;
}

inline float normMouse(float v, float scale)
{
    constexpr float kInvNorm = 1.0f / 32767.0f;
    v *= scale * kInvNorm;
    if (v >  1.0f) return  1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
}

template<typename OutData>
inline void processChannel(topic::remote_to::RemoteData& pub, OutData od)
{
    pub.chassisy = od.keyboard.w() ? 1.0f : od.keyboard.s() ? -1.0f : od.ch.chassisy;
    pub.chassisx = od.keyboard.a() ? 1.0f : od.keyboard.d() ? -1.0f : od.ch.chassisx;
    pub.pitch    = od.mouse.y             + od.ch.pitch;
    pub.yaw      = od.mouse.x             + od.ch.yaw;
}
