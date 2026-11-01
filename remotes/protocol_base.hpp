/**
 * @file protocol_base.hpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-05-12
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cstdint>
#include "remote_to.hpp"

// 共用类型 — 各协议解析的内部数据结构

struct Mouse
{
    float x = 0.0f, y = 0.0f, z = 0.0f;
    float left = 0.0f, right = 0.0f;
};

union Keyboard 
{
    uint16_t all;
    struct {
        uint16_t w     : 1;  // bit 0
        uint16_t s     : 1;  // bit 1
        uint16_t a     : 1;  // bit 2
        uint16_t d     : 1;  // bit 3
        uint16_t shift : 1;  // bit 4
        uint16_t ctrl  : 1;  // bit 5
        uint16_t q     : 1;  // bit 6
        uint16_t e     : 1;  // bit 7
        uint16_t r     : 1;  // bit 8
        uint16_t f     : 1;  // bit 9
        uint16_t g     : 1;  // bit 10
        uint16_t z     : 1;  // bit 11
        uint16_t x     : 1;  // bit 12
        uint16_t c     : 1;  // bit 13
        uint16_t v     : 1;  // bit 14
        uint16_t b     : 1;  // bit 15
    };
};

/**
 * @brief 键盘按键模式处理 — 把原始按键位转成"长按 / 切换"两种语义
 *
 * 每个 bit 对应一个键：bit=0 长按（按住才是 1），bit=1 切换（按一下翻转）。
 * 由映射层持有，映射规则和状态都归映射层。
 */
struct KeyboardState
{
public:
    /**
     * @brief 默认构造 — 全键长按
     */
    KeyboardState() : keyboard_mode(0) {}

    /**
     * @brief 构造并指定切换键掩码
     * @param mode  bit=1 的键走切换模式
     */
    explicit KeyboardState(uint16_t mode) : keyboard_mode(mode) {}

    /**
     * @brief 处理一帧原始按键
     * @param current_output  处理结果（长按键取原始值，切换键取翻转值）
     * @param current_raw     本帧原始按键位
     */
    void Process(Keyboard& current_output, const Keyboard& current_raw)
    {
        uint16_t trigger     = current_raw.all & (~last_raw_all);   	// 本帧新按下的键
		uint16_t toggle_mask = keyboard_mode;                       	// bit=1 的键走切换模式
		toggle_output.all   ^= (trigger & toggle_mask);             	// 切换键：按一次翻转
		uint16_t normal_mask = ~toggle_mask;
		current_output.all   = (toggle_output.all & toggle_mask)    	// 切换键：用翻转后的值
							 | (current_raw.all   & normal_mask);   	// 其余键：用当前原始值（长按）

		last_raw_all = current_raw.all;
    }

private:
	const uint16_t keyboard_mode;     		// bit=1 的键走切换模式，默认全 0（全长按）

    Keyboard toggle_output {};     			// 切换键的保持状态
    uint16_t last_raw_all  = 0;    			// 上一帧原始按键位，用于边沿检测
};

/**
 * @brief 通道值归一化（整型中心 → [-1, 1]）
 * @param v       原始通道值
 * @param center  中心值（如 1024）
 * @param max     最大值
 * @return float  归一化结果
 */
inline float normChannel(int16_t v, int16_t center, int16_t max)
{
    float maxDist = max - center;
    float r = (static_cast<float>(v) - center) / maxDist;
    if (r >  1.0f) r =  1.0f;
    if (r < -1.0f) r = -1.0f;
    return r;
}

/**
 * @brief 通道值归一化（预求倒数，避免除法）
 * @param v             原始通道值
 * @param center        中心值
 * @param inv_max_dist  1 / (max - center)
 * @return float        归一化结果
 */
inline float normChannelInv(int16_t v, int16_t center, float inv_max_dist)
{
    float r = (static_cast<float>(v) - static_cast<float>(center)) * inv_max_dist;
    if (r >  1.0f) r =  1.0f;
    if (r < -1.0f) r = -1.0f;
    return r;
}

/**
 * @brief 鼠标原始差值归一化 — 解析层用，不做灵敏度
 * @param v      原始鼠标差值
 * @return float 归一化结果 [-1, 1]
 */
inline float normMouse(float v)
{
    constexpr float kInvNorm = 1.0f / 32767.0f;
    return v * kInvNorm;
}

/**
 * @brief 连续量钳位 — 越界钳到 ±1，NaN 归零
 * @param v  输入值
 * @return float  钳位结果
 */
inline float ClampAxis(float v)
{
    if (!(v == v)) return  0.0f;
    if (v >  1.0f) return  1.0f;
    if (v < -1.0f) return -1.0f;
    return v;
}

/**
 * @brief 输出消息兜底校验 — 弱函数跑完后由框架调用
 *
 * 连续量钳到边界，NaN 归零，枚举越界归到安全默认值。
 * 用户区不可见：不提供声明给用户覆盖，也不在协议头文件里出现。
 *
 * @param pub  待校验的业务消息
 */
inline void ValidateOutput(topic::remote_to::Message& pub)
{
    pub.chassisx = ClampAxis(pub.chassisx);
    pub.chassisy = ClampAxis(pub.chassisy);
    pub.yaw      = ClampAxis(pub.yaw);
    pub.pitch    = ClampAxis(pub.pitch);

    if (pub.chassis_mode  > topic::remote_to::ChassisMode::Spin) pub.chassis_mode  = topic::remote_to::ChassisMode::Normal;
    if (pub.shoot_ctrl    > topic::remote_to::StartMode::On)     pub.shoot_ctrl    = topic::remote_to::StartMode::Off;
    if (pub.reload_ctrl   > topic::remote_to::StartMode::On)     pub.reload_ctrl   = topic::remote_to::StartMode::Off;
    if (pub.autoaim_ctrl  > topic::remote_to::StartMode::On)     pub.autoaim_ctrl  = topic::remote_to::StartMode::Off;
    if (pub.supercap_ctrl > topic::remote_to::StartMode::On)     pub.supercap_ctrl = topic::remote_to::StartMode::Off;
}
