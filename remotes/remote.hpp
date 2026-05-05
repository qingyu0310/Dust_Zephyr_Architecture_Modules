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

namespace thread::remote {

class Remote
{
public:
    void Init(uint16_t keyboard_mode, uint8_t detect_time = 50) {
        keyboard_mode_ = keyboard_mode;
        detect_time_ = detect_time;
    };

    void Start(uint8_t prio = 5) {
        thread_.Start(TaskEntry, prio, this);
    };

    void UartRxCpltCallback(uint8_t* buffer, uint8_t len) {
        flag_ += 1;
        DataProcess(buffer, len);
    };

    bool IsAlive() const { return alive_status_; }

protected:
    uint16_t keyboard_mode_ = 0x0000;

    virtual void ClearData() = 0;

    template<typename Keyboard>
    void Process_Keyboard_Toggle(Keyboard& current_output, Keyboard current_raw) {
        uint16_t trigger = current_raw.all & (~last_raw_all_);

        uint16_t toggle_mask = keyboard_mode_;

        toggle_output_raw_ ^= (trigger & toggle_mask);

        uint16_t normal_mask = ~toggle_mask;
        current_output.all = (toggle_output_raw_ & toggle_mask) | (current_raw.all & normal_mask);

        last_raw_all_ = current_raw.all;
    };

private:
    Thread<> thread_{};

    uint32_t flag_     = 0;
    uint32_t pre_flag_ = 0;

    bool     alive_status_ = false;
    uint8_t  detect_time_  = 50;

    uint16_t last_raw_all_      = 0;
    uint16_t toggle_output_raw_ = 0;
    
    void AlivePeriodElapsedCallback()
    {
        if (flag_ == pre_flag_) {
            alive_status_ = false;
        } else {
            alive_status_ = true;
        }
        pre_flag_ = flag_;
    }

    virtual void DataProcess(uint8_t* buffer, uint8_t len) = 0;

    virtual void Task()
    {
        for (;;) {
            AlivePeriodElapsedCallback();
            k_msleep(detect_time_);
        }
    };

    static void TaskEntry(void* p1, void* p2, void* p3) {
        auto self = static_cast<Remote*>(p1);
        self->Task();
    };
};

}
