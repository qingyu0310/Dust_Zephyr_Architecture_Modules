/**
 * @file remote.hpp
 * @author qingyu
 * @brief 遥控器接收机 — 线程内协议解析
 * @version 0.3
 * @date 2026-05-13
 */

#pragma once

#include <cstdint>
#include "thread.hpp"
#include "uart.hpp"
#include "remote_to.hpp"

namespace remote {

class RemoteProtocol
{
public:
    const uart_config &GetLineCfg() const { return line_cfg_; }

    /**
     * @brief 帧解码
     * @param buffer  原始帧数据
     * @param len     帧长度
     * @param pub     输出消息
     * @note 通道值需满足：右/上摇杆 = 通道最大值
     */
    virtual bool Decode(const uint8_t *buffer, uint8_t len, topic::remote_to::Message &pub);
    /**
     * @brief 帧内容校验 — Remote 协议锁定用，Decode 不碰 Validate
     * @param buffer  原始帧数据
     * @param len     帧长度
     * @return true   校验通过
     */
    virtual bool Validate(const uint8_t *buffer, uint8_t len);

protected:
    uart_config line_cfg_ {};      // 不同协议的 UART 参数（波特率/校验/数据位）
};

enum class Priority : uint8_t { High, Medium, Low };

struct RemoteEntry {
    const char     *name;          // 协议名称
    uint16_t        frame_size;    // 帧长度
    RemoteProtocol *protocol;      // 协议处理
    Priority        priority;      // 优先级
    uint8_t         need_hits;     // 锁定所需连续命中次数
};

class Remote final
{
public:
    bool Init(UartDma &uart)
    {
        uart_ = &uart;
        detect_.state = DetectState::Detecting;
        InitRange();
        ResetDetect();
        ready_ = true;
        return true;
    }

    bool Start(ThreadPrio prio = ThreadPrio::Normal)
    {
        if (!ready_) return false;
        thread_.Start(TaskEntry, prio, this);
        return true;
    }

private:
    static constexpr uint16_t kFrameBufSize     = 64;                   // 帧缓冲区大小
    static constexpr uint8_t  kUnlockFailLimit  = 5;                    // 连续失败解锁阈值
    static constexpr uint32_t kRemoteTimeoutMs  = 100;                  // 遥控器超时时间

    enum class DetectState : uint8_t
    {
        Detecting,
        Locked,
    };

    struct Probe {
        const RemoteEntry  *entry   = nullptr;                          // 当前候选协议
        uint8_t             hits    = 0;                                // 命中计数
        uint8_t             retry   = 0;                                // 重试计数
    };

    struct {
        DetectState         state           = DetectState::Detecting;
        uint8_t             fail_count      = 0;
        uint16_t            last_valid_ms   = 0;
        uint8_t             min_frame_size  = 0;
        uint8_t             max_frame_size  = 0;
        const RemoteEntry  *locked          = nullptr;
        Probe               probe           {};
    } detect_ {};

    struct {
        uint8_t  frame_buf_[kFrameBufSize] {};
        uint16_t frame_pos_ = 0;
    } frame_ {};

    topic::remote_to::Message pub_ {};
    UartDma            *uart_   = nullptr;             // UART 数据流
    bool                ready_  = false;
    Thread<1024 * 5>    thread_ {};                    // 遥控器解析线程

    void SwitchProto(const RemoteEntry *e);
    void InitRange();
    void ResetDetect();
    void Consume(uint16_t len);
    void HandleLocked();
    void HandleDetecting();
    void Dispatch();
    void ProcessChunk(const uint8_t *data, uint16_t len);
    
    void Task();

    static void TaskEntry(void *p1, void *p2, void *p3)
    {
        ARG_UNUSED(p2);
        ARG_UNUSED(p3);
        auto self = static_cast<Remote*>(p1);
        self->Task();
    }
};

#define REGISTER_REMOTE(RemoteType, frame_size_, priority_, lock_score_, name_)    \
    static RemoteType kRemoteProtocol_##name_;                                      \
    static const remote::RemoteEntry kRemoteEntry_##name_                           \
    __attribute__((used, __section__(".remote"))) = { #name_, frame_size_, &kRemoteProtocol_##name_, priority_, lock_score_ }

} // namespace remote
