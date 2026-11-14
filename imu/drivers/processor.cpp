/**
 * @file processor.cpp
 * @author qingyu
 * @brief 姿态解算初始化与 EKF 更新
 * @version 0.1
 * @date 2026-07-19
 */

#include "processor.hpp"
#include <cstring>

#pragma message "Compiling Modules/Imu/Drivers/Processor"
namespace attitude {

/**
 * @brief 初始化姿态解算器
 * @param cfg EKF 配置
 */
void Processor::Init(const alg::attitude::QuaternionEkf::Config& cfg)
{
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

    memcpy(pub.quaternion, state.q,  	 sizeof(pub.quaternion));
    memcpy(pub.gyro,       state.w,  	 sizeof(pub.gyro));
    memcpy(pub.accel,      state.a,  	 sizeof(pub.accel));

    pub.roll        = state.roll;
    pub.pitch       = state.pitch;
    pub.yaw         = state.yaw;
    pub.yaw_total   = state.yaw_sum;
    pub.temperature = sample.temp;

    pub.bg[0]       = state.bg[2];	// yaw 零偏（x/y 零偏由驱动层静态标定，不进状态）
    pub.chi2        = state.chi2;
    pub.k_scale     = state.k_scale;
    pub.w_norm      = state.w_norm;
    pub.a_norm      = state.a_norm;
    pub.stable      = state.stable;
}

} // namespace attitude
