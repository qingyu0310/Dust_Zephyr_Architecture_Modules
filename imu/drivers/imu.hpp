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

struct ImuEntry {
    const char  *name;
    Source      *source;
};

enum class ImuStartMode : uint8_t
{
    Normal = 0,         // 开环启动，用于调试
    AutoCalib,          // 自动校准，设置陀螺仪偏置
};

class ImuManager final
{
public:
    bool Init(ImuStartMode mode = ImuStartMode::Normal);
    
    bool Start(ThreadPrio prio = ThreadPrio::Normal)
    {
        if (!ready_) {
            return false;
        }
        thread_.Start(TaskEntry, prio, this, "imu");
        return true;
    }

    bool ReadSample(Sample& sample) { return source_->Read(sample); }

private:
    Source                  *source_    = nullptr;
    Sample                  sample_     {};
    attitude::Processor     attitude_   {};
    heater::Heater          heater_     {};

    Timer                   log_timer_  {10};
    topic::imu_to::Message  pub_        {};
    
    Thread<2048>            thread_     {};
    bool                    ready_      = false;

    bool InitSource();
    bool Preheat();

    void Task();

    static void TaskEntry(void *p1, void *p2, void *p3)
    {
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);
        auto *self = static_cast<ImuManager*>(p1);
        self->Task();
    }
};


/**
 * @brief 注册 IMU 设备到 .imu linker section
 *
 * 创建静态设备实例和 ImuEntry，链接时由 InitSource() 遍历。
 * 单个二进制只允许注册一个 IMU 设备，多注册在 InitSource() 时报错。
 *
 * @param ImuType  设备类名（需继承 Source）
 * @param name_    设备名称，同时作为变量名后缀和运行时标识
 */
#define REGISTER_IMU(ImuType, name_)                                                \
    static ImuType kImuSource_##name_;                                              \
    static const imu::ImuEntry kImuEntry_##name_                                         \
    __attribute__((used, __section__(".imu"))) = { #name_, &kImuSource_##name_ }

} // namespace imu
