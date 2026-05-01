/**
 * @file horizon.hpp
 * @author qingyu
 * @brief 
 * @version 0.1
 * @date 2026-04-30
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "stdint.h"

class PowerMeter
{
public:
    PowerMeter() = default;

    PowerMeter(uint16_t rx_id) : rx_id_(rx_id) {}

    void Init(uint16_t rx_id) {
        rx_id_ = rx_id;
    };

    void CanCpltRxCallback(uint8_t* buffer);

    float GetShuntVolt() const { return shunt_volt; }
    float GetBusVolt() const { return bus_volt; }
    float GetCurrent() const { return current; }
    float GetPower() const { return power; }

private:

    uint16_t rx_id_;

    float shunt_volt = 0.0f;
    float bus_volt = 0.0f;
    float current = 0.0f;
    float power = 0.0f;
};