/**
 * @file powermeter.hpp
 * @author qingyu
 * @brief HPM5361 功率计 CAN 解析 — seqlock 保护 ISR/线程数据一致性
 * @version 0.2
 * @date 2026-05-10
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "stdint.h"
#include <zephyr/sys/atomic.h>

class PowerMeter
{
public:

    PowerMeter() = default;
    PowerMeter(uint16_t rx_id) : rx_id_(rx_id) {}

    void Init(uint16_t rx_id) {
        rx_id_ = rx_id;
    };

    void CanCpltRxCallback(uint8_t* buffer);

    struct Snapshot {
        float shunt_volt, bus_volt, current, power;
    };

    /** @brief 批量读——seqlock 保护，一次拿到所有值的一致快照 */
    Snapshot ReadAll() const
    {
        atomic_t seq;
        Snapshot snap;
        do {
            seq = atomic_get(&seq_);
            if (seq & 1) continue;
            snap.shunt_volt = shunt_volt;
            snap.bus_volt   = bus_volt;
            snap.current    = current;
            snap.power      = power;
        } while (atomic_get(&seq_) != seq);
        return snap;
    }

    float GetShuntVolt() const { return shunt_volt; }
    float GetBusVolt()   const { return bus_volt;   }
    float GetCurrent()   const { return current;    }
    float GetPower()     const { return power;      }

private:
    uint16_t rx_id_  = 0x00;

    float shunt_volt = 0.0f;
    float bus_volt   = 0.0f;
    float current    = 0.0f;
    float power      = 0.0f;

    mutable atomic_t seq_ = 0;
};