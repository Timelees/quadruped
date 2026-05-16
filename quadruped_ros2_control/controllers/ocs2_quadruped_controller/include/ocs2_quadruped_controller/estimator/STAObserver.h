#pragma once

#include <Eigen/Dense>
#include <vector>
#include <sophus/so3.hpp>

namespace ocs2::legged_robot
{
    // 前向声明，避免与 ErrorStateKalmanFilter.h 循环包含
    struct ESKFNominalState;
    struct STAState
    {
        // 状态维度定义
        static constexpr int POS_IDX = 0;
        static constexpr int VEL_IDX = 3;
        static constexpr int ROT_IDX = 6;

        static constexpr int STATE_DIM = 9;       // 状态总维度：p(3)+v(3)+theta(3)
        static constexpr int ESKF_STATE_DIM = 27; // ESKF状态维度
    };

    struct STADisturbance
    {
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
         * @param k1 STA 一阶注入增益
         * @param k2 STA 二阶扰动增益
         */
        void init(double k1, double k2);

        /**
         * @brief 更新观测器状态
         * @param dt 时间步长
         * @param state ESKF 后验名义状态
         * @param omega_body 去偏置后的机体系角速度
         */
        void update(double dt, const ESKFNominalState &state, const Eigen::Vector3d &omega_body);

        void reset(const ESKFNominalState &state);

        void setBandwidth(double bandwidth);

        void setParameters(int covariance_window, double epsilon, double covariance_regularization,
                           double max_covariance, double max_disturbance);

        [[nodiscard]] const STADisturbanceVec &getDisturbance() const { return z1_; }

        [[nodiscard]] STADisturbanceCov getDisturbanceCovariance() const;

        [[nodiscard]] STADisturbanceCov computeDisturbanceCovariance(int window_size) const;

    private:
        // 状态和扰动定义
        STAStateVec z_;      // 状态z_k
        STAStateVec z_prev_; // 状态z_{k-1}

        STADisturbanceVec z1_;      // 扰动z1_k
        STADisturbanceVec z1_prev_; // 扰动z1_{k-1}

        // 历史扰动记录
        std::vector<STADisturbanceVec> disturbance_history_;
        std::size_t max_history_size_ = 20;
        int covariance_window_ = 20;
        double epsilon_ = 1e-6;
        double covariance_regularization_ = 1e-6;
        double max_covariance_ = 10.0;
        double max_disturbance_ = 50.0;
        bool initialized_ = false;

        // STA增益矩阵（方阵）
        STAGainMatrix K1_;
        STAGainMatrix K2_;

        // 状态转移矩阵F
        STAStateMatrix F_;

        // 输出矩阵 H (选择矩阵)
        STAStateMatrix H_;

        // 扰动输入矩阵B_d
        STADisturbanceInputMatrix Bd_;

        // 扰动协方差矩阵
        STADisturbanceCov Qd_;

        void setGains(const Eigen::MatrixXd &L);

        [[nodiscard]] static STAStateVec stateToVector(const ESKFNominalState &state);
    };
}
