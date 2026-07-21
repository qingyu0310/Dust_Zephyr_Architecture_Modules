/**
 * @file imu.hpp
 * @author qingyu
 * @brief IMU 模块公共接口与采样数据结构
 * @version 0.1
 * @date 2026-06-02
 */

#pragma once

#include <cstdint>
#include "imu_to.hpp"
#include "quaternion.hpp"
#include "thread.hpp"
#include "timer.hpp"
#include "imu_device_layer.hpp"
#include "heater.hpp"
#include "processor.hpp"

namespace imu {

enum class ImuStartMode : uint8_t
{
    Normal = 0,         // 开环启动，用于调试
    AutoCalib,          // 自动校准，设置陀螺仪偏置
    AutoIdent,          // 自动辨识，配合脚本使用
};

class ImuManager final
{
public:
    bool Init(ImuStartMode mode = ImuStartMode::Normal);
    bool Start(uint8_t prio);

private:
    Source                  *source_    = nullptr;
    Sample                  sample_     {};
    attitude::Processor     attitude_   {};
    heater::Heater          heater_     {};

    Timer                   log_timer_  {10};
    topic::imu_to::Message  pub_        {};
    
    Thread<4096>            thread_     {};
    bool                    ready_      = false;

    void Task();

    static void TaskEntry(void *p1, void *p2, void *p3)
    {
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);
        auto *self = static_cast<ImuManager*>(p1);
        self->Task();
    }

    bool SelectSource();
    bool Preheat();
};

} // namespace imu
