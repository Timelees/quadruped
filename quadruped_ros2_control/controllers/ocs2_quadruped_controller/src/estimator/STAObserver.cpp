#include "ocs2_quadruped_controller/estimator/STAObserver.h"
#include <cmath>
#include <algorithm> // for clamp

namespace ocs2::legged_robot
{
    void STAObserver::init(double k1, double k2)
    {
        k1_ = k1;
        k2_ = k2;
        velocity_.setZero();
        disturbance_.setZero();
    }

    void STAObserver::update(double dt, const Eigen::Vector3d &acc_body, const Eigen::Quaterniond &q_wb, const Eigen::Vector3d &vel_meas)
    {
        // 1. 将机体加速度转换到世界系，并移除重力
        // 假设 acc_body 包含重力分量 (静止时 z ~ 9.8)
        // a_world_real = q * a_body + g_vec(0,0,-9.8)
        Eigen::Vector3d acc_world = q_wb * acc_body + gravity_;

        // 2. 计算观测误差 (测量速度 - 估计速度)
        Eigen::Vector3d error = vel_meas - velocity_;

        // 3. Super-Twisting Algorithm (STA)
        // 对应论文公式 (24) 和 (25) 的简化形式
        Eigen::Vector3d correction_v;
        Eigen::Vector3d correction_d;

        for (int i = 0; i < 3; ++i)
        {
            double e = error(i);
            double sgn_e = (e > 0) ? 1.0 : ((e < 0) ? -1.0 : 0.0);
            double sqrt_e = std::sqrt(std::abs(e));

            // STA 连续项
            correction_v(i) = k1_ * sqrt_e * sgn_e;
            // STA 积分项 (扰动导数)
            correction_d(i) = k2_ * sgn_e;
        }

        // 4. 更新估计值 (Euler积分)
        // dot(v) = a + d + correction_v
        velocity_ += (acc_world + disturbance_ + correction_v) * dt;
        
        // dot(d) = correction_d
        disturbance_ += correction_d * dt;

        // [优化] 添加扰动限幅，防止发散 (假设最大外力产生的加速度不超过 20 m/s^2)
        double max_disturbance = 20.0;
        disturbance_ = disturbance_.cwiseMax(-max_disturbance).cwiseMin(max_disturbance);
    }
}