#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

namespace ocs2::legged_robot::lie_group
{
    /**
     * @brief SO(3) Lie Group 工具函数
     * 基于论文 "Hybrid Real-Time State Estimation for Quadruped Robots With Proprioceptive Sensors"
     * 以及标准李群李代数理论
     */

    // 小角度阈值，用于数值稳定性
    constexpr double SMALL_ANGLE_THRESHOLD = 1e-8;

    /**
     * @brief 将向量转换为反对称矩阵 (skew-symmetric matrix / hat operator)
     * @param v 3D向量
     * @return 3x3反对称矩阵 [v]_x
     */
    inline Eigen::Matrix3d skew(const Eigen::Vector3d &v)
    {
        Eigen::Matrix3d m;
        m << 0.0, -v.z(), v.y(),
            v.z(), 0.0, -v.x(),
            -v.y(), v.x(), 0.0;
        return m;
    }

    inline Eigen::Matrix3d Gamma(const Eigen::Vector3d &omega)
    {
    }

    /**
     * @brief SO(3) 指数映射: so(3) -> SO(3)
     * 将旋转向量 (轴角表示) 映射到旋转矩阵
     * Rodrigues 公式(基于李代数生成元): exp(theta * axis) = I + sin(theta) * [axis]_x + (1 - cos(theta)) * [axis]_x^2
     * @param omega 旋转向量 (轴角: omega = theta * axis)
     * @return 旋转矩阵 R ∈ SO(3)
     */
    inline Eigen::Matrix3d ExpSO3(const Eigen::Vector3d &omega)
    {
        double theta = omega.norm();
        if (theta < SMALL_ANGLE_THRESHOLD)
        {
            // 一阶近似: exp(omega) ≈ I + [omega]_x
            return Eigen::Matrix3d::Identity() + skew(omega);
        }

        Eigen::Vector3d axis = omega / theta;
        Eigen::Matrix3d K = skew(axis);

        // Rodrigues 公式
        return Eigen::Matrix3d::Identity() + std::sin(theta) * K + (1.0 - std::cos(theta)) * K * K;
    }

    /**
     * @brief SO(3) 指数映射，返回四元数
     * @param omega 旋转向量 (轴角表示)
     * @return 单位四元数 q ∈ S^3
     */
    inline Eigen::Quaterniond ExpSO3Quat(const Eigen::Vector3d &omega)
    {
        const double theta_sq = omega.squaredNorm();

        if (theta_sq < SMALL_ANGLE_THRESHOLD * SMALL_ANGLE_THRESHOLD)
        {
            // 使用更精确的小角度近似（保留二阶项）
            const double theta = std::sqrt(theta_sq);
            const double half_theta = 0.5 * theta;

            // 二阶近似：sin(x) ≈ x - x³/6, cos(x) ≈ 1 - x²/2
            // 但为简化，通常使用 sinc 函数
            double imag_factor, real_factor;

            if (theta < 1e-8)
            {
                // 极小角度，避免除零
                imag_factor = 0.5;
                real_factor = 1.0;
            }
            else
            {
                const double sin_half_theta_over_theta = std::sin(half_theta) / theta;
                imag_factor = sin_half_theta_over_theta;
                real_factor = std::cos(half_theta);
            }

            return Eigen::Quaterniond(real_factor,
                                      imag_factor * omega.x(),
                                      imag_factor * omega.y(),
                                      imag_factor * omega.z());
        }

        // 正常角度计算
        const double theta = std::sqrt(theta_sq);
        const double half_theta = 0.5 * theta;
        const double sin_half_theta = std::sin(half_theta);
        const double cos_half_theta = std::cos(half_theta);

        const Eigen::Vector3d axis = omega / theta;

        return Eigen::Quaterniond(cos_half_theta,
                                  sin_half_theta * axis.x(),
                                  sin_half_theta * axis.y(),
                                  sin_half_theta * axis.z());
    }

    
    /**
     * @brief SO(3) 对数映射: SO(3) -> so(3)
     * 将旋转矩阵映射到旋转向量
     * @param R 旋转矩阵 R ∈ SO(3)
     * @return 旋转向量 omega ∈ R^3
     */
    inline Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R)
    {
        double trace = R.trace();
        double cos_theta = (trace - 1.0) / 2.0;

        // 数值稳定性处理
        cos_theta = std::max(-1.0, std::min(1.0, cos_theta));

        double theta = std::acos(cos_theta);

        if (theta < SMALL_ANGLE_THRESHOLD)
        {
            // 小角度近似: omega ≈ vee(R - I)
            return Eigen::Vector3d(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1)) / 2.0;
        }

        if (std::abs(theta - M_PI) < SMALL_ANGLE_THRESHOLD)
        {
            // theta ≈ π 的特殊处理
            // 找到 R 对角线最大元素对应的轴
            Eigen::Vector3d axis;
            if (R(0, 0) >= R(1, 1) && R(0, 0) >= R(2, 2))
            {
                axis.x() = std::sqrt((R(0, 0) + 1.0) / 2.0);
                axis.y() = R(0, 1) / (2.0 * axis.x());
                axis.z() = R(0, 2) / (2.0 * axis.x());
            }
            else if (R(1, 1) >= R(0, 0) && R(1, 1) >= R(2, 2))
            {
                axis.y() = std::sqrt((R(1, 1) + 1.0) / 2.0);
                axis.x() = R(0, 1) / (2.0 * axis.y());
                axis.z() = R(1, 2) / (2.0 * axis.y());
            }
            else
            {
                axis.z() = std::sqrt((R(2, 2) + 1.0) / 2.0);
                axis.x() = R(0, 2) / (2.0 * axis.z());
                axis.y() = R(1, 2) / (2.0 * axis.z());
            }
            return theta * axis;
        }

        // 一般情况
        Eigen::Vector3d omega(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
        return omega * (theta / (2.0 * std::sin(theta)));
    }

    /**
     * @brief SO(3) 对数映射 (四元数版本)
     * @param q 单位四元数
     * @return 旋转向量 omega ∈ R^3
     */
    inline Eigen::Vector3d LogSO3(const Eigen::Quaterniond &q)
    {
        // 确保 w > 0 以避免符号歧义
        Eigen::Quaterniond q_pos = (q.w() < 0) ? Eigen::Quaterniond(-q.coeffs()) : q;

        Eigen::Vector3d v(q_pos.x(), q_pos.y(), q_pos.z());
        double v_norm = v.norm();

        if (v_norm < SMALL_ANGLE_THRESHOLD)
        {
            // 小角度近似: omega ≈ 2 * v
            return 2.0 * v;
        }

        double theta = 2.0 * std::atan2(v_norm, q_pos.w());
        return theta * (v / v_norm);
    }

    /**
     * @brief SO(3) 右雅可比矩阵 Jr(omega)
     * 用于误差状态传播
     * Jr(omega) = I - (1-cos(theta))/theta^2 * [omega]_x + (theta - sin(theta))/theta^3 * [omega]_x^2
     * @param omega 旋转向量
     * @return 3x3 右雅可比矩阵
     */
    inline Eigen::Matrix3d JrSO3(const Eigen::Vector3d &omega)
    {
        double theta = omega.norm();

        if (theta < SMALL_ANGLE_THRESHOLD)
        {
            // 小角度近似: Jr ≈ I - 0.5 * [omega]_x
            return Eigen::Matrix3d::Identity() - 0.5 * skew(omega);
        }

        double theta2 = theta * theta;
        double theta3 = theta2 * theta;

        Eigen::Matrix3d K = skew(omega);

        return Eigen::Matrix3d::Identity() - ((1.0 - std::cos(theta)) / theta2) * K + ((theta - std::sin(theta)) / theta3) * K * K;
    }

    /**
     * @brief SO(3) 右雅可比矩阵的逆 Jr^{-1}(omega)
     * @param omega 旋转向量
     * @return 3x3 右雅可比矩阵的逆
     */
    inline Eigen::Matrix3d JrInvSO3(const Eigen::Vector3d &omega)
    {
        double theta = omega.norm();

        if (theta < SMALL_ANGLE_THRESHOLD)
        {
            // 小角度近似: Jr^{-1} ≈ I + 0.5 * [omega]_x
            return Eigen::Matrix3d::Identity() + 0.5 * skew(omega);
        }

        double half_theta = theta / 2.0;
        double cot_half_theta = std::cos(half_theta) / std::sin(half_theta);

        Eigen::Matrix3d K = skew(omega / theta);

        return (half_theta * cot_half_theta) * Eigen::Matrix3d::Identity() + (1.0 - half_theta * cot_half_theta) * K * K + half_theta * K;
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
     * delta_theta = Log(q_measured^{-1} * q_estimated)
     * @param q_est 估计的四元数
     * @param q_meas 测量的四元数
     * @return 误差旋转向量
     */
    inline Eigen::Vector3d quaternionError(const Eigen::Quaterniond &q_est, const Eigen::Quaterniond &q_meas)
    {
        Eigen::Quaterniond q_err = q_meas.inverse() * q_est;
        return LogSO3(q_err);
    }

} // namespace ocs2::legged_robot::lie_group
