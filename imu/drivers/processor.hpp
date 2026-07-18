/**
 * @file process.hpp
 * @author qingyu
 * @brief IMU 姿态解算模块
 * @version 0.1
 * @date 2026-07-19
 */

#pragma once

#include "imu_device_layer.hpp"
#include "imu_to.hpp"
#include "quaternion.hpp"

namespace attitude {

class Processor final
{
public:
    void Init();
    void Process(const Sample& sample, topic::imu_to::Message& pub);

private:
    alg::attitude::QuaternionEkf ekf_ {};
};

} // namespace attitude
