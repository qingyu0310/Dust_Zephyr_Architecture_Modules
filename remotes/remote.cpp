/**
 * @file remote.cpp
 * @author qingyu
 * @brief 遥控器线程 — 信号量驱动 + 线程内帧同步
 * @version 0.3
 * @date 2026-05-13
 *
 * ## 数据流
 *
 * UART DMA 接收 → ISR 写环缓冲区 → k_sem_give()
 *                                    ↓
 *                           Remote::Task() 被唤醒
 *                                    ↓
 *                  uart_->Read() 从环缓冲区取数据
 *                                    ↓
 *                  累积到 frame_buf_（线程内帧缓冲区）
 *                                    ↓
 *                  帧长足够 → 协议解析 → zbus 发布
 *                  超时无数据 → pub_ 归零（掉线处理）
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "remote.hpp"
#include "remote_to.hpp"
#include "thread.hpp"
#include "uart.hpp"

#include "dr16.hpp"
#include "vt12.hpp"
#include "vt13.hpp"

using namespace topic::remote_to;

namespace thread::remote {

class Remote final
{
public:
    using DataProcessFunc = bool(*)(uint8_t* buffer, uint8_t len, RemoteData& pub);

    struct Protocol {
        DataProcessFunc func;
        uint16_t        frame_size;
    };

    void Init(RemoteType type, RxStream& uart) {
        type_ = type;
        k_sem_init(&rx_sem_, 0, 1);
        uart_ = &uart;
        uart_->SetNotify(&rx_sem_);
        GetProcessFunc();
    };

    void Start(uint8_t prio = 5) {
        thread_.Start(TaskEntry, prio, this);
    };

private:

    Thread<> thread_ {};
    RxStream* uart_ = nullptr;

    RemoteType type_ = RemoteType::None;

    uint8_t  frame_buf_[64]{};
    uint16_t frame_pos_ = 0;
    k_sem    rx_sem_;

    RemoteData pub_  {};
    Protocol proto_  { nullptr, 0 };

    void GetProcessFunc();

    void ClearPubData() {
        pub_ = {0};
    }

    void Task()
    {
        for (;;)
        {
            if (k_sem_take(&rx_sem_, K_MSEC(50)) == 0) 
            {
                uint8_t tmp[32];
                uint16_t n = uart_->Read(tmp, sizeof(tmp));
                if (n == 0) continue;

                uint16_t cap = sizeof(frame_buf_) - frame_pos_;
                if (n > cap) n = cap;
                memcpy(frame_buf_ + frame_pos_, tmp, n);
                frame_pos_ += n;

                // 滑动窗口：尽可能多地处理已收齐的帧
                while (proto_.frame_size > 0 && frame_pos_ >= proto_.frame_size) 
                {
                    if (proto_.func(frame_buf_, proto_.frame_size, pub_)) 
                    {
                        uint16_t rem = frame_pos_ - proto_.frame_size;
                        if (rem > 0) {
                            memmove(frame_buf_, frame_buf_ + proto_.frame_size, rem);
                        }
                        frame_pos_ = rem;
                    } else {
                        // dataprocess 返回 false → 帧错位，挪 1 字节重试
                        frame_pos_--;
                        memmove(frame_buf_, frame_buf_ + 1, frame_pos_);
                    }
                }
            } else {
                // 当信号量超时，代表remote掉线
                ClearPubData();
            }
        }
    }

    static void TaskEntry(void* p1, void* p2, void* p3) {
        auto self = static_cast<Remote*>(p1);
        self->Task();
    };
};

static constexpr Remote::Protocol kProtocols[] {
    { dr16::dataprocess, dr16::kFrameSizeDR16 },   // DR16
    { vt12::dataprocess, vt12::kFrameSizeVT12 },   // VT12
    { vt13::dataprocess, vt13::kFrameSizeVT13 },   // VT13
    { nullptr,           0                    },   // None
};

static Remote remote_{};

void thread_init()
{
    static UartDma rx {};

    RxStream::Config cfg {};
    cfg.buf_size   = 128;
    cfg.rx_timeout = 0;
    rx.Init(DEVICE_DT_GET(DT_ALIAS(uart_remote)), cfg);
    
    remote_.Init(RemoteType::DR16, rx);
}

void thread_start(uint8_t prio)
{
    remote_.Start(prio);
}

void Remote::GetProcessFunc()
{
    uint8_t idx = static_cast<uint8_t>(type_);
    if (idx < sizeof(kProtocols) / sizeof(kProtocols[0])) {
        proto_ = kProtocols[idx];
    }
};

}
