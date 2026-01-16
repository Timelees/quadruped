#pragma once

#include <Eigen/Dense>
#include <iostream>

namespace ocs2::legged_robot
{
    class STAObserver
    {
    public:
        STAObserver() = default;
        ~STAObserver() = default;

        /**
         * @brief 初始化观测器
         * @param k1 增益参数1 (对应论文中的增益系数)
         * @param k2 增益参数2 (对应论文中的增益系数)
         */
        void init(double k1, double k2);

        /**
         * @brief 更新观测器状态
         * @param dt 时间步长
         * @param acc_body 机体坐标系下的加速度 (IMU测量值)
         * @param q_wb 机体到世界的旋转四元数 (ESKF估计值)
         * @param vel_meas 观测到的速度 (通常来自足端里程计)
         */
        void update(double dt, const Eigen::Vector3d &acc_body, const Eigen::Quaterniond &q_wb, const Eigen::Vector3d &vel_meas);

        /**
         * @brief 获取估计的速度
         * @return 世界坐标系下的速度
         */
        Eigen::Vector3d getVelocity() const { return velocity_; }

        /**
         * @brief 获取估计的扰动 (外部力/加速度偏差)
         * @return 世界坐标系下的扰动
         */
        Eigen::Vector3d getDisturbance() const { return disturbance_; }

    private:
        Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();    // 估计速度
        Eigen::Vector3d disturbance_ = Eigen::Vector3d::Zero(); // 估计扰动
        
        double k1_ = 0.0;
        double k2_ = 0.0;
        
        const Eigen::Vector3d gravity_{0.0, 0.0, -9.81};
    };
}
