/**
 * @file remote.hpp
 * @author qingyu
 * @brief Remote 遥控器基类 — Template Method 模式
 * @version 0.1
 * @date 2026-05-02
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "thread.hpp"
#include "zephyr/kernel.h"
#include <cstdint>

enum class RemoteType : uint8_t
{
    DR16 = 0,
    VT12,
    VT13,
    None,
};

namespace thread::remote {

class Remote final
{
public:
    void Init(RemoteType type) {
        type_ = type;
        GetProcessFunc();
    };

    void Start(uint8_t prio = 5) {
        thread_.Start(TaskEntry, prio, this);
    };

    void UartRxCpltCallback(uint8_t* buffer, uint8_t len) {
        flag_ += 1;
        DataProcess(buffer, len);
    };

    bool IsAlive() const { return alive_status_; }

private:

    Thread<> thread_{};

    RemoteType type_ = RemoteType::None;

    uint32_t flag_     = 0;
    uint32_t pre_flag_ = 0;
    bool     alive_status_ = false;

    using DataProcessFunc = void(*)(uint8_t* buffer, uint8_t len);
    DataProcessFunc DataProcess = nullptr;

    void GetProcessFunc();

    void AlivePeriodElapsedCallback()
    {
        if (flag_ == pre_flag_) {
            alive_status_ = false;
        } else {
            alive_status_ = true;
        }
        pre_flag_ = flag_;
    }

    void Task()
    {
        for (;;) {
            AlivePeriodElapsedCallback();
            k_msleep(50);
        }
    };

    static void TaskEntry(void* p1, void* p2, void* p3) {
        auto self = static_cast<Remote*>(p1);
        self->Task();
    };
};

}
