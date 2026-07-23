/**
 * @file remote.cpp
 * @author qingyu
 * @brief 遥控器接收机 — 线程内协议解析
 * @version 0.3
 * @date 2026-05-13
 */

#include "remote.hpp"
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(remote, LOG_LEVEL_INF);

extern const remote::RemoteEntry __remote_start[];
extern const remote::RemoteEntry __remote_end[];

namespace remote {

/**
 * @brief 遥控器线程主循环 — 接收 UART 数据并送入解析器。
 *        锁定超时后发布归零数据并回到探测状态。
 */
void Remote::Task()
{
    for (;;)
    {
        if (k_sem_take(&uart_->sem_, K_MSEC(50)) == 0)
        {
            uint8_t tmp[32];
            while (true)
            {
                uint16_t n = uart_->Read(tmp, sizeof(tmp));
                if (n == 0) break;

                ProcessChunk(tmp, n);
            }
        }
        else
        {
            uint32_t now = k_uptime_get_32();
            if (detect_.locked != nullptr && now - detect_.last_valid_ms >= kRemoteTimeoutMs)
            {
                if (detect_.last_valid_ms != 0) {
                    LOG_ERR("lost %s", detect_.locked->name);
                    detect_.last_valid_ms = 0;
                }
                pub_ = {};
                zbus_chan_pub(&pub_remote_to, &pub_, K_MSEC(1));
            }
        }
    }
}

/**
 * @brief 追加一段 UART RX 数据，并尝试解析其中的完整帧。
 */
void Remote::ProcessChunk(const uint8_t *data, uint16_t len)
{
    while (len > 0)
    {
        uint16_t cap = kFrameBufSize - frame_.frame_pos_;
        uint16_t n = len > cap ? cap : len;
        memcpy(frame_.frame_buf_ + frame_.frame_pos_, data, n);
        frame_.frame_pos_ += n;
        data += n;
        len -= n;

        Dispatch();
    }
}

/**
 * @brief 基于当前帧缓冲区运行解析状态机。
 */
void Remote::Dispatch()
{
    if (detect_.state == DetectState::Locked) {
        HandleLocked();
    } else {
        HandleDetecting();
    }
}

/**
 * @brief 自动检测 — 驻留当前协议，接收 N 帧积累命中后锁定。
 *        失败次数达到 need_hits 则换下一协议，全部耗尽后丢字节重来。
 */
void Remote::HandleDetecting()
{
    // 无候选 → 从第一个协议开始驻留
    if (detect_.probe.entry == nullptr)
    {
        detect_.probe.entry = __remote_start;
        if (detect_.probe.entry >= __remote_end) return;
        SwitchProto(detect_.probe.entry);
        detect_.probe.hits  = 0;
        detect_.probe.retry = 0;
        return;
    }

    const auto *e = detect_.probe.entry;

    // 帧不够 → 继续接收
    if (frame_.frame_pos_ < e->frame_size) return;

    // Validate
    if (e->protocol->Validate(frame_.frame_buf_, e->frame_size))
    {
        detect_.probe.hits++;
        if (detect_.probe.hits >= e->need_hits)
        {
            LOG_INF("%s: hit %d/%d, lock", e->name, detect_.probe.hits, e->need_hits);
            detect_.locked   = e;
            detect_.state    = DetectState::Locked;
            detect_.fail_count = 0;
        }
        else
        {
            LOG_INF("%s: hit %d/%d", e->name, detect_.probe.hits, e->need_hits);
        }
        Consume(e->frame_size);
        return;
    }

    // Validate 失败
    detect_.probe.retry++;
    Consume(e->frame_size);

    // 当前协议失败次数够了 → 换下一个
    if (detect_.probe.retry >= e->need_hits)
    {
        const auto *next = e + 1;
        if (next >= __remote_end)
        {
            // 全部试完 → 从头开始
            detect_.probe = {};
            frame_.frame_pos_ = 0;
            return;
        }
        // 切到下一个协议
        detect_.probe.entry = next;
        detect_.probe.hits  = 0;
        detect_.probe.retry = 0;
        SwitchProto(next);
    }
}

/**
 * @brief 已锁定协议解码 — 直接走 Decode，连续失败后回到探测。
 */
void Remote::HandleLocked()
{
    if (detect_.locked == nullptr) {
        ResetDetect();
        return;
    }
    const auto *entry = detect_.locked;

    while (frame_.frame_pos_ >= entry->frame_size)
    {
        if (entry->protocol->Decode(frame_.frame_buf_, entry->frame_size, pub_))
        {
            if (detect_.last_valid_ms == 0) {
                LOG_INF("reconnect %s", entry->name);
            }
            detect_.last_valid_ms = k_uptime_get_32();
            detect_.fail_count = 0;
            Consume(entry->frame_size);
        } else {
            detect_.fail_count++;
            Consume(entry->frame_size);
        }

        if (detect_.fail_count >= kUnlockFailLimit) {
            ResetDetect();
            break;
        }
    }
}

/**
 * @brief 切换到指定协议的 UART 配置。
 */
void Remote::SwitchProto(const RemoteEntry *e)
{
    LOG_INF("switch to %s", e->name);
    uart_->StopRx();
    frame_.frame_pos_ = 0;                                  // 丢弃旧数据
    k_busy_wait(1000);                        // 等待 FIFO 排空
    uart_->SetLineConfig(e->protocol->GetLineCfg());
    uart_->StartRx();
}

/**
 * @brief 遍历注册表计算帧长范围，用于自动探测帧对齐。
 */
void Remote::InitRange()
{
    detect_.min_frame_size = UINT8_MAX;
    detect_.max_frame_size = 0;

    for (const remote::RemoteEntry *e = __remote_start; e < __remote_end; e++)
    {
        if (e->frame_size < detect_.min_frame_size) detect_.min_frame_size = e->frame_size;
        if (e->frame_size > detect_.max_frame_size) detect_.max_frame_size = e->frame_size;
    }
}

/**
 * @brief 回到协议探测状态，清空锁定和候选计数。
 */
void Remote::ResetDetect()
{
    if (detect_.locked != nullptr) {
        LOG_ERR("lost %s, redetect", detect_.locked->name);
    }
    detect_.locked         = nullptr;
    detect_.state          = DetectState::Detecting;
    detect_.fail_count     = 0;
    detect_.probe          = {};
}

/**
 * @brief 从帧缓冲区头部移除已消费的字节。
 */
void Remote::Consume(uint16_t len)
{
    if (len >= frame_.frame_pos_) {
        frame_.frame_pos_ = 0;
        return;
    }

    uint16_t rem = frame_.frame_pos_ - len;
    memmove(frame_.frame_buf_, frame_.frame_buf_ + len, rem);
    frame_.frame_pos_ = rem;
}

} // namespace remote
