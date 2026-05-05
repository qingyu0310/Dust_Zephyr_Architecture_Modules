/**
 * @file dr16.hpp
 * @author qingyu
 * @brief DR16 遥控器子类
 * @version 0.1
 * @date 2026-05-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#pragma once

#include "remote.hpp"
#include <cstdint>

namespace thread::remote {

struct DR16Keyboard {
    union 
    {
        uint16_t all;
        struct {
            uint16_t w : 1;
            uint16_t s : 1;
            uint16_t a : 1;
            uint16_t d : 1;
            uint16_t q : 1;
            uint16_t e : 1;
            uint16_t shift : 1;
            uint16_t ctrl : 1;
            uint16_t reserved : 8;
        };
    };
};

struct DR16Mouse {
    float x{}, y{}, z{};
    bool left{}, mid{}, right{};
};

struct DR16Output {
    struct {
        float x;            // chassis x
        float y;            // chassis y
        float yaw;
        float pitch;
    } axis;

    struct {
        uint8_t left;
        uint8_t right;
    } sw;

    DR16Keyboard keyboard;
    DR16Mouse mouse;
};

class DR16 final : public Remote
{
public:
    const DR16Output& GetOutput() const { return output_; }

private:
    void ClearData() override;
    void DataProcess(uint8_t* buffer, uint8_t len) override;

    DR16Output output_{};
};

}
