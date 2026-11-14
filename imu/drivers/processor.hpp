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
    /**
     * @brief 初始化姿态解算器
     * @param cfg EKF 配置
     */
    void Init(const alg::attitude::QuaternionEkf::Config& cfg);

    void Process(const Sample& sample, topic::imu_to::Message& pub);

private:
    alg::attitude::QuaternionEkf ekf_ {};
};

} // namespace attitude
