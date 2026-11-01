/**
 * @file vt12.hpp
 * @author qingyu
 * @brief VT12 协议输出数据结构与后处理接口
 * @version 0.1
 * @date 2026-11-01
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "protocol_base.hpp"
#include "remote_to.hpp"

namespace vt12 {

struct OutputData
{
    Mouse    mouse;
    Keyboard keyboard;
};

/**
 * @brief 后处理入口（__weak），用户区按需覆盖
 *
 * 默认实现为空；调用时 DefaultFillOutputData 已经把默认映射填好。
 * 用户在这里改字段、或按自己的灵敏度重算鼠标轴。
 *
 * @param in   协议输出（原始数据）
 * @param out  已被默认映射填好的业务消息
 */
void FillOutputData(const OutputData& in, topic::remote_to::Message& out);

} // namespace vt12
