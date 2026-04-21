#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <sophus/so3.hpp>

namespace ocs2::legged_robot
{
    // 前向声明，避免与 ErrorStateKalmanFilter.h 循环包含
    struct ESKFNominalState;
    struct STAState
    {
        Eigen::Vector3d p;    // body位置 (世界系)
        Eigen::Vector3d v;    // body速度 (世界系)
        Eigen::Quaterniond q; // body姿态 (机体系 -> 世界系)

        STAState()
        {
            p.setZero();
            v.setZero();
            q.setIdentity();
        }

        // 状态维度定义
        static constexpr int POS_IDX = 0;
        static constexpr int VEL_IDX = 3;
        static constexpr int ROT_IDX = 6;

        static constexpr int STATE_DIM = 9;       // 状态总维度（p(3)+v(3)+旋转矩阵G(3)）
        static constexpr int ESKF_STATE_DIM = 27; // ESKF状态维度
    };

    struct STADisturbance
    {
        Eigen::Vector3d dp;       // 位置扰动
        Eigen::Vector3d dv;       // 速度扰动
        Eigen::Quaterniond dquat; // 姿态扰动

        STADisturbance()
        {
            dp.setZero();
            dv.setZero();
            dquat.setIdentity();
        }

        // 扰动维度定义
        static constexpr int POS_IDX = 0;
        static constexpr int VEL_IDX = 3;
        static constexpr int ROT_IDX = 6;

        static constexpr int DISTURBANCE_DIM = 9; // 扰动总维度
    };

    using STAStateVec = Eigen::Matrix<double, STAState::STATE_DIM, 1>;
    using STADisturbanceVec = Eigen::Matrix<double, STADisturbance::DISTURBANCE_DIM, 1>;
    using STAStateMatrix = Eigen::Matrix<double, STAState::STATE_DIM, STAState::STATE_DIM>;
    using STADisturbanceMatrix = Eigen::Matrix<double, STADisturbance::DISTURBANCE_DIM, STADisturbance::DISTURBANCE_DIM>;
    using STAGainMatrix = Eigen::Matrix<double, STAState::STATE_DIM, STAState::STATE_DIM>;
    using STAInputMatrix = Eigen::Matrix<double, STAState::STATE_DIM, STAState::ESKF_STATE_DIM>;
    using STADisturbanceInputMatrix = Eigen::Matrix<double, STADisturbance::DISTURBANCE_DIM, STAState::STATE_DIM>;
    using STADisturbanceCov = Eigen::Matrix<double, STADisturbance::DISTURBANCE_DIM, STADisturbance::DISTURBANCE_DIM>;

    class STAObserver
    {
    public:
        STAObserver() = default;
        ~STAObserver() = default;

        /**
         * @brief 初始化观测器
         * @param k1 增益参数1
         * @param k2 增益参数2
         */
        void init(double k1, double k2);

        /**
         * @brief 更新观测器状态
         * @param dt 时间步长
         * @param acc_world 世界坐标系下的加速度
         * @param q_wb 机体到世界的旋转四元数 (ESKF估计值)
         * @param vel_meas 观测到的速度 （运动学计算结果）
         */
        void update(const double dt, ESKFNominalState &state, Eigen::Vector3d &acc_world);

        STADisturbanceCov computeDisturbanceCovariance(int window_size);

    private:
        // 状态和扰动定义
        STAStateVec z_;      // 状态z_k
        STAStateVec z_prev_; // 状态z_{k-1}

        STADisturbanceVec z1_;      // 扰动z1_k
        STADisturbanceVec z1_prev_; // 扰动z1_{k-1}

        // 历史扰动记录
        std::vector<STADisturbanceVec> disturbance_history_;
        std::size_t max_history_size_ = 20;  

        // STA增益矩阵（方阵）
        STAGainMatrix K1_;
        STAGainMatrix K2_;

        // （历史扰动记录已在上方声明）

        // 状态转移矩阵F
        STAStateMatrix F_;

    // 输出矩阵 H (选择矩阵) - 将 H 设为 9x9 的 STAStateMatrix，使其能与 STAStateVec 相乘
    STAStateMatrix H_;

        // 扰动输入矩阵B_d
        STADisturbanceInputMatrix Bd_;

        // 扰动协方差矩阵
        STADisturbanceCov Qd_;

        void setGains(const Eigen::MatrixXd &L);
    };
}
