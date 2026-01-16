#pragma once

#include "LieGroupUtils.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <iostream>
#include <limits>

namespace ocs2::legged_robot
{
    using namespace lie_group;

    /**
     * @brief ESKF 名义状态结构
     * 状态布局: [p(3), v(3), q(4), f1(3), f2(3), f3(3), f4(3), ba(3), bg(3)]

     */
    struct ESKFNominalState
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d p;    // body位置 (世界系)
        Eigen::Vector3d v;    // body速度 (世界系)
        Eigen::Quaterniond q; // body姿态 (机体系 -> 世界系 )
        Eigen::Vector3d f[4]; // 4个足端位置 (世界系)
        Eigen::Vector3d ba;   // 加速度计偏置
        Eigen::Vector3d bg;   // 陀螺仪偏置

        ESKFNominalState()
        {
            p.setZero();
            v.setZero();
            q = Eigen::Quaterniond::Identity();
            for (int i = 0; i < 4; ++i)
                f[i].setZero();
            ba.setZero();
            bg.setZero();
        }

        // 状态维度定义
        static constexpr int POS_IDX = 0;
        static constexpr int VEL_IDX = 3;
        static constexpr int ROT_IDX = 6; // 误差状态中姿态是 3 维
        static constexpr int F1_IDX = 9;
        static constexpr int F2_IDX = 12;
        static constexpr int F3_IDX = 15;
        static constexpr int F4_IDX = 18;
        static constexpr int BA_IDX = 21;
        static constexpr int BG_IDX = 24;

        static constexpr int STATE_DIM = 27;       // 误差状态维度
        static constexpr int NOMINAL_DIM = 28;     // 名义状态维度 (四元数占4维)
        static constexpr int PROCESS_NOISE_DIM = 24; // 过程噪声维度 [n_g(3), n_a(3), n_f(12), n_ba(3), n_bg(3)]
        
        // 观测维度定义
        // 观测向量布局: [foot_pos(12), foot_vel(12), foot_height(4)]
        // - foot_pos: 4足端在body系下的位置 (4×3=12)
        // - foot_vel: 4足端在world系下的速度 (4×3=12), 支撑相应为0
        // - foot_height: 4足端在world系下的高度 (4×1=4), 支撑相应为footRadius
        static constexpr int MEAS_FOOT_POS_DIM = 12;    // 足端位置观测维度
        static constexpr int MEAS_FOOT_VEL_DIM = 12;    // 足端速度观测维度
        static constexpr int MEAS_FOOT_HEIGHT_DIM = 4;  // 足端高度观测维度
        static constexpr int MEAS_DIM = 28;             // 总观测维度 (12 + 12 + 4 = 28)
        
        // 观测向量索引
        static constexpr int MEAS_FPOS_IDX = 0;         // 足端位置起始索引
        static constexpr int MEAS_FVEL_IDX = 12;        // 足端速度起始索引
        static constexpr int MEAS_FHEIGHT_IDX = 24;     // 足端高度起始索引
    };

    /**
     * @brief ESKF 输入结构 (IMU 测量值 + STA 扰动)
     */
    struct ESKFInput
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d acc;      // 加速度计测量 (机体系)
        Eigen::Vector3d gyro;     // 陀螺仪测量 (机体系)
        Eigen::Vector3d dist_vel; // 速度扰动 (STA)
        Eigen::Vector3d dist_acc; // 加速度扰动 (STA)
        Eigen::Vector3d dist_rot; // 姿态扰动 (STA)

        ESKFInput()
        {
            acc.setZero();
            gyro.setZero();
            dist_vel.setZero();
            dist_acc.setZero();
            dist_rot.setZero();
        }
    };

    /**
     * @brief 基于李群李代数的 Error-State Kalman Filter
     *
     * 实现论文中的离散时间 ESKF 公式：
     * - 名义状态传播 (Nominal State Propagation)
     * - 误差状态传播 (Error State Propagation)
     * - 量测更新 (Measurement Update)
     * - 误差状态注入与重置 (Error Injection and Reset)
     */
    class LieGroupESKF
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        using StateVec = Eigen::Matrix<double, ESKFNominalState::STATE_DIM, 1>;
        using StateCov = Eigen::Matrix<double, ESKFNominalState::STATE_DIM, ESKFNominalState::STATE_DIM>;
        using ProcessNoiseCov = Eigen::Matrix<double, ESKFNominalState::PROCESS_NOISE_DIM, ESKFNominalState::PROCESS_NOISE_DIM>;
        using MeasVec = Eigen::Matrix<double, ESKFNominalState::MEAS_DIM, 1>;
        using MeasCov = Eigen::Matrix<double, ESKFNominalState::MEAS_DIM, ESKFNominalState::MEAS_DIM>;
        using MeasJacobian = Eigen::Matrix<double, ESKFNominalState::MEAS_DIM, ESKFNominalState::STATE_DIM>;

        LieGroupESKF()
            : P_(StateCov::Identity())
        {
        }

        /**
         * @brief 初始化滤波器
         * @param x0 初始名义状态
         * @param P0 初始误差协方差
         */
        void init(const ESKFNominalState &x0, const StateCov &P0)
        {
            x_ = x0;
            P_ = P0;
        }

        /**
         * @brief 设置名义状态
         */
        void setState(const ESKFNominalState &x)
        {
            x_ = x;
        }

        /**
         * @brief 设置误差协方差
         */
        void setCovariance(const StateCov &P)
        {
            P_ = P;
        }

        /**
         * @brief 获取当前名义状态
         */
        const ESKFNominalState &getState() const { return x_; }

        /**
         * @brief 获取当前误差协方差
         */
        const StateCov &getCovariance() const { return P_; }

        /**
         * @brief 获取当前观测噪声协方差
         */
        const MeasCov &getMeasurementCovariance() const { return R_; }

        /**
         * @brief ESKF 预测步骤 (Prediction Step)
         *
         * 论文公式:
         * 名义状态传播:
         *   p_k = p_{k-1} + v_{k-1} * dt + 0.5 * (R_{k-1} * (a_m - b_a) + g) * dt^2
         *   v_k = v_{k-1} + (R_{k-1} * (a_m - b_a) + g) * dt
         *   R_k = R_{k-1} * Exp((ω_m - b_g) * dt)
         *   f_i = f_i (不变, 由过程噪声建模)
         *   b_a = b_a (不变, 由过程噪声建模)
         *   b_g = b_g (不变, 由过程噪声建模)
         *
         * 误差状态传播:
         *   δx_k = A_k * δx_{k-1} + B_k * w
         *   P_k = A_k * P_{k-1} * A_k^T + B_k * Q * B_k^T
         *
         * @param dt 时间步长
         * @param Q 过程噪声协方差
         * @param u 输入 (IMU 测量值)
         */
        void predict(double dt, const ProcessNoiseCov &Q, const ESKFInput &u)
        {
            if (dt <= 0 || !std::isfinite(dt))
            {
                std::cerr << "[LieGroupESKF] Invalid dt: " << dt << std::endl;
                return;
            }

            // 1. 提取去偏置后的 IMU 测量值
            Eigen::Vector3d a_unbias = u.acc - x_.ba;
            Eigen::Vector3d omega_unbias = u.gyro - x_.bg;

            // 2. 当前姿态旋转矩阵
            Eigen::Matrix3d R = x_.q.toRotationMatrix();

            // 3. 重力向量 (世界系, Z轴向上)
            Eigen::Vector3d g(0.0, 0.0, -9.81);

            // 4. 计算世界系下的加速度
            Eigen::Vector3d a_world = R * a_unbias + g;

            // =================== 名义状态传播 ===================
            // 位置更新: p_k = p_{k-1} + v * dt 
            x_.p = x_.p + x_.v * dt + 0.5 * a_world * dt * dt;

            // 速度更新: v_k = v_{k-1} + a * dt
            x_.v = x_.v + a_world * dt;

            // 姿态更新: R_k = R_{k-1} * Exp(ω * dt)
            Eigen::Vector3d delta_theta = omega_unbias * dt;    // 旋转角度
            Eigen::Quaterniond dq = ExpSO3Quat(delta_theta);
            x_.q = x_.q * dq;
            x_.q.normalize(); // 保持单位四元数

            // 足端位置、偏置保持不变 (由过程噪声建模)
            for(int i = 0; i<4; i++)
            {
                x_.f[i] = x_.f[i];
            }
            x_.ba = x_.ba;
            x_.bg = x_.bg;

            // =================== 误差状态协方差传播 ===================
            // 计算离散化的状态转移矩阵 A 和噪声雅可比 B

            // 状态转移矩阵 A (连续时间)
            // 误差状态布局: [δp, δv, δθ, δf1, δf2, δf3, δf4, δba, δbg]
            StateCov A = StateCov::Identity();

            // ∂δp_dot/∂δv = I
            A.block<3, 3>(ESKFNominalState::POS_IDX, ESKFNominalState::VEL_IDX) = Eigen::Matrix3d::Identity() * dt;

            // ∂δp_dot/∂δba = -R * dt * dt * 0.5
            A.block<3, 3>(ESKFNominalState::POS_IDX, ESKFNominalState::BA_IDX) = -R * (dt * dt * 0.5);

            // ∂δv_dot/∂δθ = -R * [a_unbias]_x * dt
            A.block<3, 3>(ESKFNominalState::VEL_IDX, ESKFNominalState::ROT_IDX) = -R * skew(a_unbias) * dt;

            // ∂δv_dot/∂δba = -R * dt
            A.block<3, 3>(ESKFNominalState::VEL_IDX, ESKFNominalState::BA_IDX) = -R * dt;

            // ∂δθ_dot/∂δθ = I - [ω_unbias]_x * dt ≈ Exp(-ω * dt)
            // 使用右雅可比近似
            A.block<3, 3>(ESKFNominalState::ROT_IDX, ESKFNominalState::ROT_IDX) = ExpSO3(-delta_theta);

            // ∂δθ_dot/∂δbg = -I * dt
            A.block<3, 3>(ESKFNominalState::ROT_IDX, ESKFNominalState::BG_IDX) = -Eigen::Matrix3d::Identity() * dt;

            // 足端位置和偏置的自回归项已经在 Identity 中

            // 噪声雅可比 B
            // 噪声布局: [n_g(3), n_a(3), n_f1(3), n_f2(3), n_f3(3), n_f4(3), n_ba(3), n_bg(3)]
            Eigen::Matrix<double, ESKFNominalState::STATE_DIM, ESKFNominalState::PROCESS_NOISE_DIM> B;
            B.setZero();

            // 陀螺仪噪声 -> 姿态误差
            B.block<3, 3>(ESKFNominalState::ROT_IDX, 0) = -ExpSO3(delta_theta) * dt;

            // 加速度噪声 -> 速度误差
            B.block<3, 3>(ESKFNominalState::VEL_IDX, 3) = -R * dt;

            // 足端过程噪声 -> 足端位置误差 (世界系)
            B.block<3, 3>(ESKFNominalState::F1_IDX, 6) = R * dt;
            B.block<3, 3>(ESKFNominalState::F2_IDX, 9) = R * dt;
            B.block<3, 3>(ESKFNominalState::F3_IDX, 12) = R * dt;
            B.block<3, 3>(ESKFNominalState::F4_IDX, 15) = R * dt;

            // 偏置随机游走噪声
            B.block<3, 3>(ESKFNominalState::BA_IDX, 18) = Eigen::Matrix3d::Identity() * dt;
            B.block<3, 3>(ESKFNominalState::BG_IDX, 21) = Eigen::Matrix3d::Identity() * dt;

            // 协方差传播: P_k = A * P_{k-1} * A^T + B * Q * B^T
            P_ = A * P_ * A.transpose() + B * Q * B.transpose();

            // 确保对称性
            P_ = 0.5 * (P_ + P_.transpose());
        }

        /**
         * @brief ESKF 更新步骤 (Update Step)
         * 用于处理线性模型
         * 论文公式:
         *   K = P * C^T * S^{-1}
         *   S = C * P * C^T + R
         *   δx = K * (z - h(x))
         *   P = (I - K * C) * P
         *
         * 然后将误差状态注入名义状态:
         *   p ← p + δp
         *   v ← v + δv
         *   q ← q ⊗ Exp(δθ)
         *   f_i ← f_i + δf_i
         *   ba ← ba + δba
         *   bg ← bg + δbg
         *
         * 最后重置误差状态 (ESKF Reset):
         *   P ← G_reset * P * G_reset^T
         *   其中 G_reset 在姿态部分为 I - [δθ/2]_x
         *
         * @param y 观测向量 (残差: z = z_meas - h(x))
         * @param C 观测雅可比矩阵 ∂h/∂δx
         * @param R 观测噪声协方差
         */
        void update(const MeasVec &y, const MeasJacobian &C, const MeasCov &R)
        {
            // 1. 计算卡尔曼增益
            MeasCov S = C * P_ * C.transpose() + R;         // 计算残差协方差

            // 数值防护：对称化 + 对角 jitter，避免由于数值误差导致的非正定/奇异
            S = 0.5 * (S + S.transpose());
            double jitter = 1e-12;
            const double max_diag = S.diagonal().cwiseAbs().maxCoeff();
            if (std::isfinite(max_diag) && max_diag > 0.0) {
                jitter = std::max(jitter, 1e-12 * max_diag);
            }
            S.diagonal().array() += jitter;

            if (!S.allFinite() || !C.allFinite() || !P_.allFinite() || !R.allFinite() || !y.allFinite()) {
                std::cerr << "[LieGroupESKF] Non-finite detected before update. "
                          << "yFinite=" << y.allFinite() << ", CFinite=" << C.allFinite()
                          << ", PFinite=" << P_.allFinite() << ", RFinite=" << R.allFinite()
                          << ", SFinite=" << S.allFinite() << std::endl;
                return;
            }

            // 使用 LDLT（对称）分解，比 LU 更适合协方差矩阵
            Eigen::LDLT<MeasCov> ldlt(S);
            if (ldlt.info() != Eigen::Success) {
                std::cerr << "[LieGroupESKF] LDLT failed in update(). Skipping update. jitter=" << jitter << std::endl;
                return;
            }

            Eigen::Matrix<double, ESKFNominalState::MEAS_DIM, ESKFNominalState::STATE_DIM> S_inv_C = ldlt.solve(C);
            Eigen::Matrix<double, ESKFNominalState::STATE_DIM, ESKFNominalState::MEAS_DIM> K = P_ * S_inv_C.transpose();

            // 2. 计算误差状态修正
            StateVec dx = K * y;
            
            // NaN 检查
            if (!dx.allFinite())
            {
                std::cerr << "[LieGroupESKF] NaN detected in update dx, skipping update" << std::endl;
                return;
            }

            // 3. 将误差状态注入名义状态
            injectErrorState(dx);

            // 4. 更新协方差 (约瑟夫形式保证矩阵正定性)
            StateCov I_KH = StateCov::Identity() - K * C;
            P_ = I_KH * P_ * I_KH.transpose() + K * R * K.transpose();      // 非最优卡尔曼增益下会引入观测噪声项

            // 5. 误差状态重置 (姿态部分)
            resetErrorState(dx);

            // 确保对称性
            P_ = 0.5 * (P_ + P_.transpose());
        }

        /**
         * @brief 迭代 ESKF 更新 (Iterated EKF Update)
         * 用于处理非线性观测模型
         *
         * @param maxIter 最大迭代次数
         * @param convergenceThreshold 收敛阈值
         * @param measurementFunc 观测函数，返回 (残差, 雅可比H, 噪声R)
         */
        template <typename MeasurementFunc>
        void updateIterated(int maxIter, double convergenceThreshold, MeasurementFunc measurementFunc)
        {
            ESKFNominalState x_prior = x_;
            StateCov P_prior = P_;

            for (int iter = 0; iter < maxIter; ++iter)
            {
                // 计算当前状态下的观测残差和雅可比
                MeasVec y;
                MeasJacobian C;
                MeasCov R;
                bool valid = measurementFunc(x_, y, C, R);

                R_ = R; // 更新当前观测噪声协方差

                if (!valid)
                    continue;

                // 计算与先验状态的误差
                StateVec dx_prior = computeStateDifference(x_, x_prior);

                // 迭代 EKF 更新公式
                MeasCov S = C * P_prior * C.transpose() + R;
                S = 0.5 * (S + S.transpose());
                double jitter = 1e-12;
                const double max_diag = S.diagonal().cwiseAbs().maxCoeff();
                if (std::isfinite(max_diag) && max_diag > 0.0) {
                    jitter = std::max(jitter, 1e-12 * max_diag);
                }
                S.diagonal().array() += jitter;

                if (!S.allFinite() || !C.allFinite() || !P_prior.allFinite() || !R.allFinite() || !y.allFinite()) {
                    std::cerr << "[LieGroupESKF] Non-finite detected before updateIterated(). iter=" << iter
                              << " yFinite=" << y.allFinite() << ", CFinite=" << C.allFinite()
                              << ", PFinite=" << P_prior.allFinite() << ", RFinite=" << R.allFinite()
                              << ", SFinite=" << S.allFinite() << std::endl;
                    break;
                }

                Eigen::LDLT<MeasCov> ldlt(S);
                if (ldlt.info() != Eigen::Success) {
                    std::cerr << "[LieGroupESKF] LDLT failed in updateIterated(). iter=" << iter
                              << " jitter=" << jitter << ". Skipping." << std::endl;
                    break;
                }

                Eigen::Matrix<double, ESKFNominalState::MEAS_DIM, ESKFNominalState::STATE_DIM> S_inv_C = ldlt.solve(C);
                Eigen::Matrix<double, ESKFNominalState::STATE_DIM, ESKFNominalState::MEAS_DIM> K =
                    P_prior * S_inv_C.transpose();

                StateCov I_KH = StateCov::Identity() - K * C;
                StateVec dx = K * y + I_KH * dx_prior;

                // NaN 检查
                if (!dx.allFinite())
                {
                    std::cerr << "[LieGroupESKF] NaN detected in updateIterated dx at iter " << iter << ", skipping" << std::endl;
                    break;
                }

                // 检查收敛
                bool converged = true;
                for (int i = 0; i < ESKFNominalState::STATE_DIM; ++i)
                {
                    if (std::fabs(dx[i] - dx_prior[i]) > convergenceThreshold)
                    {
                        converged = false;
                        break;
                    }
                }

                // 更新状态
                x_ = x_prior;
                injectErrorState(dx);

                if (converged)
                {
                    P_ = I_KH * P_prior * I_KH.transpose() + K * R * K.transpose();
                    resetErrorState(dx);
                    P_ = 0.5 * (P_ + P_.transpose());
                    break;
                }

                if (iter == maxIter - 1)
                {
                    P_ = I_KH * P_prior * I_KH.transpose() + K * R * K.transpose();
                    resetErrorState(dx);
                    P_ = 0.5 * (P_ + P_.transpose());
                }
            }
        }

        /**
         * @brief 带日志功能的迭代 ESKF 更新
         * 
         * @param maxIter 最大迭代次数
         * @param convergenceThreshold 收敛阈值
         * @param measurementFunc 观测函数
         * @param logCallback 日志回调函数，参数：(迭代次数, 残差y, 雅可比C, 卡尔曼增益K, 误差状态dx, 是否收敛)
         */
        template <typename MeasurementFunc, typename LogCallback>
        void updateIteratedWithLog(int maxIter, double convergenceThreshold, 
                                   MeasurementFunc measurementFunc, LogCallback logCallback)
        {
            ESKFNominalState x_prior = x_;
            StateCov P_prior = P_;
            
            int final_iter = 0;
            bool final_converged = false;

            for (int iter = 0; iter < maxIter; ++iter)
            {
                final_iter = iter;
                
                // 计算当前状态下的观测残差和雅可比
                MeasVec y;
                MeasJacobian C;
                MeasCov R;
                bool valid = measurementFunc(x_, y, C, R);
                R_ = R;
                
                if (!valid)
                    continue;

                // 计算与先验状态的误差
                StateVec dx_prior = computeStateDifference(x_, x_prior);

                // 迭代 EKF 更新公式
                MeasCov S = C * P_prior * C.transpose() + R;

                // 数值防护：对称化 + jitter
                S = 0.5 * (S + S.transpose());
                double jitter = 1e-12;
                const double max_diag = S.diagonal().cwiseAbs().maxCoeff();
                if (std::isfinite(max_diag) && max_diag > 0.0) {
                    jitter = std::max(jitter, 1e-12 * max_diag);
                }
                S.diagonal().array() += jitter;

                if (!S.allFinite() || !C.allFinite() || !P_prior.allFinite() || !R.allFinite() || !y.allFinite())
                {
                    std::cerr << "[LieGroupESKF] Non-finite detected before updateIteratedWithLog(). iter=" << iter
                              << " yFinite=" << y.allFinite() << ", CFinite=" << C.allFinite()
                              << ", PFinite=" << P_prior.allFinite() << ", RFinite=" << R.allFinite()
                              << ", SFinite=" << S.allFinite() << std::endl;

                    // 记录一次，方便离线定位
                    Eigen::Matrix<double, ESKFNominalState::STATE_DIM, ESKFNominalState::MEAS_DIM> K_nan;
                    K_nan.setZero();
                    StateVec dx_nan;
                    dx_nan.setConstant(std::numeric_limits<double>::quiet_NaN());
                    logCallback(iter, y, C, K_nan, dx_nan, S, false, true);
                    break;
                }

                Eigen::LDLT<MeasCov> ldlt(S);
                if (ldlt.info() != Eigen::Success)
                {
                    std::cerr << "[LieGroupESKF] LDLT failed in updateIteratedWithLog(). iter=" << iter
                              << " jitter=" << jitter << ". Skipping." << std::endl;

                    Eigen::Matrix<double, ESKFNominalState::STATE_DIM, ESKFNominalState::MEAS_DIM> K_nan;
                    K_nan.setZero();
                    StateVec dx_nan;
                    dx_nan.setConstant(std::numeric_limits<double>::quiet_NaN());
                    logCallback(iter, y, C, K_nan, dx_nan, S, false, true);
                    break;
                }

                Eigen::Matrix<double, ESKFNominalState::MEAS_DIM, ESKFNominalState::STATE_DIM> S_inv_C = ldlt.solve(C);
                Eigen::Matrix<double, ESKFNominalState::STATE_DIM, ESKFNominalState::MEAS_DIM> K =
                    P_prior * S_inv_C.transpose();          // 计算卡尔曼增益

                StateCov I_KH = StateCov::Identity() - K * C;
                StateVec dx = K * y + I_KH * dx_prior;

                // NaN 检查
                if (!dx.allFinite())
                {
                    std::cerr << "[LieGroupESKF] NaN detected in updateIterated dx at iter " << iter << ", skipping" << std::endl;
                    logCallback(iter, y, C, K, dx, S, false, true);  // NaN detected
                    break;
                }

                // 检查收敛
                bool converged = true;
                double max_change = 0.0;
                for (int i = 0; i < ESKFNominalState::STATE_DIM; ++i)
                {
                    double change = std::fabs(dx[i] - dx_prior[i]);
                    max_change = std::max(max_change, change);
                    if (change > convergenceThreshold)
                    {
                        converged = false;
                    }
                }
                
                // 调用日志回调
                logCallback(iter, y, C, K, dx, S, converged, false);

                // 更新状态
                x_ = x_prior;
                injectErrorState(dx);

                if (converged)
                {
                    final_converged = true;
                    // 更新协方差
                    P_ = I_KH * P_prior * I_KH.transpose() + K * R * K.transpose();
                    resetErrorState(dx);
                    P_ = 0.5 * (P_ + P_.transpose());
                    break;
                }

                if (iter == maxIter - 1)
                {
                    // 最后一次迭代，更新协方差
                    P_ = I_KH * P_prior * I_KH.transpose() + K * R * K.transpose();
                    resetErrorState(dx);
                    P_ = 0.5 * (P_ + P_.transpose());
                }
            }
        }

    private:
        ESKFNominalState x_; // 名义状态
        StateCov P_;         // 误差状态协方差
        MeasCov R_;         // 观测噪声协方差
        /**
         * @brief 将误差状态注入名义状态
         */
        void injectErrorState(const StateVec &dx)
        {
            // 位置
            x_.p += dx.segment<3>(ESKFNominalState::POS_IDX);

            // 速度
            x_.v += dx.segment<3>(ESKFNominalState::VEL_IDX);

            // 姿态 (使用李群更新: q ← q ⊗ Exp(δθ))
            Eigen::Vector3d delta_theta = dx.segment<3>(ESKFNominalState::ROT_IDX);
            Eigen::Quaterniond dq = ExpSO3Quat(delta_theta);
            x_.q = x_.q * dq;
            x_.q.normalize();

            // 足端位置
            x_.f[0] += dx.segment<3>(ESKFNominalState::F1_IDX);
            x_.f[1] += dx.segment<3>(ESKFNominalState::F2_IDX);
            x_.f[2] += dx.segment<3>(ESKFNominalState::F3_IDX);
            x_.f[3] += dx.segment<3>(ESKFNominalState::F4_IDX);

            // 偏置
            x_.ba += dx.segment<3>(ESKFNominalState::BA_IDX);
            x_.bg += dx.segment<3>(ESKFNominalState::BG_IDX);
        }

        /**
         * @brief 误差状态重置 (修正协方差)
         * 旋转空间的“加法”（乘法）是非线性的，把误差注入名义状态这个动作，会轻微改变误差分布的形状（协方差）。所以必须用 G 矩阵对 P 进行“重置/投影”。
         * 对于姿态部分，使用:
         *   P_θθ ← G_reset * P_θθ * G_reset^T
         *   其中 G_reset = I - 0.5 * [δθ]_x
         */
        void resetErrorState(const StateVec &dx)
        {
            Eigen::Vector3d delta_theta = dx.segment<3>(ESKFNominalState::ROT_IDX);

            // 构造重置雅可比 (只有姿态部分需要修正)
            StateCov G_reset = StateCov::Identity();
            G_reset.block<3, 3>(ESKFNominalState::ROT_IDX, ESKFNominalState::ROT_IDX) =
                Eigen::Matrix3d::Identity() - 0.5 * skew(delta_theta);

            // 更新协方差
            P_ = G_reset * P_ * G_reset.transpose();
        }

        /**
         * @brief 计算两个状态之间的误差 (切空间表示)
         */
        StateVec computeStateDifference(const ESKFNominalState &x, const ESKFNominalState &x0) const
        {
            StateVec dx = StateVec::Zero();

            dx.segment<3>(ESKFNominalState::POS_IDX) = x.p - x0.p;
            dx.segment<3>(ESKFNominalState::VEL_IDX) = x.v - x0.v;
            dx.segment<3>(ESKFNominalState::ROT_IDX) = LogSO3(x0.q.inverse() * x.q);
            dx.segment<3>(ESKFNominalState::F1_IDX) = x.f[0] - x0.f[0];
            dx.segment<3>(ESKFNominalState::F2_IDX) = x.f[1] - x0.f[1];
            dx.segment<3>(ESKFNominalState::F3_IDX) = x.f[2] - x0.f[2];
            dx.segment<3>(ESKFNominalState::F4_IDX) = x.f[3] - x0.f[3];
            dx.segment<3>(ESKFNominalState::BA_IDX) = x.ba - x0.ba;
            dx.segment<3>(ESKFNominalState::BG_IDX) = x.bg - x0.bg;

            return dx;
        }
    };

} // namespace ocs2::legged_robot
