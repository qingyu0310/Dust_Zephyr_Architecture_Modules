/**
 * @file processor.cpp
 * @author qingyu
 * @brief 姿态解算初始化与 EKF 更新
 * @version 0.1
 * @date 2026-07-19
 */

#include "processor.hpp"
#include <cstring>

namespace attitude {

/**
 * @brief 初始化姿态解算器
 *
 * 配置加速度计低通滤波时间常数后初始化 EKF。
 */
void Processor::Init()
{
    constexpr float kDefaultAccelLpfTimeConstant = 0.02f;
    alg::attitude::QuaternionEkf::Config cfg {};
    cfg.alpha = kDefaultAccelLpfTimeConstant;
    ekf_.Init(cfg);
}

/**
 * @brief 处理一帧 IMU 样本并发布姿态结果
 *
 * 执行 EKF 更新，将解算结果（四元数、角速度、欧拉角、温度）
 * 填入发布消息结构体。
 *
 * @param sample 当前 IMU 工程量样本
 * @param pub    输出消息，post 后发送到 topic
 */
void Processor::Process(const Sample& sample, topic::imu_to::Message& pub)
{
    ekf_.Update(sample);

    const auto& state = ekf_.GetState();

    memcpy(pub.quaternion, state.q,  sizeof(pub.quaternion));
    memcpy(pub.gyro,       state.w,  sizeof(pub.gyro));

    pub.roll        = state.roll;
    pub.pitch       = state.pitch;
    pub.yaw         = state.yaw;
    pub.yaw_total   = state.yaw_sum;
    pub.temperature = sample.temp;
}

} // namespace attitude
