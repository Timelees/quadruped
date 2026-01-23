#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

// 使用 Sophus 库进行李群李代数计算
#include <sophus/so3.hpp>

namespace ocs2::legged_robot::lie_group
{
    /**
     * @brief SO(3) Lie Group 工具函数
     * 基于 Sophus 库实现
     * 参考论文 "Hybrid Real-Time State Estimation for Quadruped Robots With Proprioceptive Sensors"
     */

    // 小角度阈值，用于数值稳定性
    constexpr double SMALL_ANGLE_THRESHOLD = 1e-8;

    /**
     * @brief 将向量转换为反对称矩阵 (skew-symmetric matrix / hat operator)
     * 使用 Sophus::SO3::hat() 替代
     * @param v 3D向量
     * @return 3x3反对称矩阵 [v]_x
     */
    inline Eigen::Matrix3d skew(const Eigen::Vector3d &v)
    {
        return Sophus::SO3d::hat(v);
    }

    inline Eigen::Matrix3d Gamma(const Eigen::Vector3d &omega)
    {
        // 暂未实现
        return Eigen::Matrix3d::Identity();
    }

    /**
     * @brief SO(3) 指数映射: so(3) -> SO(3)
     * 使用 Sophus::SO3::exp() 替代 Rodrigues 公式
     * @param omega 旋转向量 (轴角: omega = theta * axis)
     * @return 旋转矩阵 R ∈ SO(3)
     */
    inline Eigen::Matrix3d ExpSO3(const Eigen::Vector3d &omega)
    {
        return Sophus::SO3d::exp(omega).matrix();
    }

    /**
     * @brief SO(3) 指数映射，返回四元数
     * 使用 Sophus::SO3::exp() 并转换为四元数
     * @param omega 旋转向量 (轴角表示)
     * @return 单位四元数 q ∈ S^3
     */
    inline Eigen::Quaterniond ExpSO3Quat(const Eigen::Vector3d &omega)
    {
        return Sophus::SO3d::exp(omega).unit_quaternion();
    }

    
    /**
     * @brief SO(3) 对数映射: SO(3) -> so(3)
     * 使用 Sophus::SO3::log() 替代
     * @param R 旋转矩阵 R ∈ SO(3)
     * @return 旋转向量 omega ∈ R^3
     */
    inline Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R)
    {
        return Sophus::SO3d(R).log();
    }

    /**
     * @brief SO(3) 对数映射 (四元数版本)
     * 使用 Sophus::SO3::log() 替代
     * @param q 单位四元数
     * @return 旋转向量 omega ∈ R^3
     */
    inline Eigen::Vector3d LogSO3(const Eigen::Quaterniond &q)
    {
        return Sophus::SO3d(q).log();
    }

    /**
     * @brief SO(3) 右雅可比矩阵 Jr(omega)
     *
     * 注意：Sophus::SO3d::Dx_exp_x() 返回的是 exp(x) 对 x 的导数（4x3，四元数参数化），
     * 不是 ESKF 推导里常用的 3x3 右雅可比矩阵。
     * 这里保留标准解析形式：
     *   Jr(ω) = I - (1-cosθ)/θ^2 [ω]_x + (θ-sinθ)/θ^3 [ω]_x^2
     */
    inline Eigen::Matrix3d JrSO3(const Eigen::Vector3d &omega)
    {
        const double theta = omega.norm();
        if (theta < SMALL_ANGLE_THRESHOLD)
        {
            return Eigen::Matrix3d::Identity() - 0.5 * skew(omega);
        }

        const double theta2 = theta * theta;
        const double theta3 = theta2 * theta;
        const Eigen::Matrix3d K = skew(omega);

        return Eigen::Matrix3d::Identity() - ((1.0 - std::cos(theta)) / theta2) * K + ((theta - std::sin(theta)) / theta3) * K * K;
    }

    /**
     * @brief SO(3) 右雅可比矩阵的逆 Jr^{-1}(omega)
     * 使用 Sophus::SO3::Dx_exp_x_inverse_at() 替代 (若可用)
     * 或通过求逆 JrSO3(omega).inverse()
     * @param omega 旋转向量
     * @return 3x3 右雅可比矩阵的逆
     */
    inline Eigen::Matrix3d JrInvSO3(const Eigen::Vector3d &omega)
    {
        // 直接对右雅可比矩阵求逆
        return JrSO3(omega).inverse();
    }

    /**
     * @brief SO(3) 左雅可比矩阵 Jl(omega) = Jr(-omega)
     * @param omega 旋转向量
     * @return 3x3 左雅可比矩阵
     */
    inline Eigen::Matrix3d JlSO3(const Eigen::Vector3d &omega)
    {
        return JrSO3(-omega);
    }

    /**
     * @brief 四元数乘法的导数矩阵 (用于协方差传播)
     * ∂(q ⊗ δq) / ∂δθ 其中 δq = [1, δθ/2]
     * @param q 参考四元数
     * @return 4x3 矩阵
     */
    inline Eigen::Matrix<double, 4, 3> dQuatProduct_dDeltaTheta(const Eigen::Quaterniond &q)
    {
        Eigen::Matrix<double, 4, 3> M;
        // q ⊗ δq, δq = [1, δθ/2] 时的线性化
        // 结果 ≈ q + 0.5 * q ⊗ [0, δθ]
        M << -q.x(), -q.y(), -q.z(),
            q.w(), -q.z(), q.y(),
            q.z(), q.w(), -q.x(),
            -q.y(), q.x(), q.w();
        return 0.5 * M;
    }

    /**
     * @brief 计算两个四元数之间的误差 (切空间表示)
     * 使用 Sophus 进行李群计算
     * delta_theta = Log(q_measured^{-1} * q_estimated)
     * @param q_est 估计的四元数
     * @param q_meas 测量的四元数
     * @return 误差旋转向量
     */
    inline Eigen::Vector3d quaternionError(const Eigen::Quaterniond &q_est, const Eigen::Quaterniond &q_meas)
    {
        // 使用 Sophus 计算四元数误差
        Sophus::SO3d so3_est(q_est);
        Sophus::SO3d so3_meas(q_meas);
        return (so3_meas.inverse() * so3_est).log();
    }

} // namespace ocs2::legged_robot::lie_group
