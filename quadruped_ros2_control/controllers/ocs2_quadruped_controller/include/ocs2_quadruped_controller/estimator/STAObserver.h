#pragma once

#include <Eigen/Dense>
#include <vector>
#include <sophus/so3.hpp>

namespace ocs2::legged_robot
{
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
    using STADisturbanceCov = Eigen::Matrix<double, STADisturbance::DISTURBANCE_DIM, STADisturbance::DISTURBANCE_DIM>;

    /** 质心动力学单步传播：描述“模型认为机器人应如何运动” */
    struct STADynamicsInput
    {
        Eigen::Vector3d accel_world{Eigen::Vector3d::Zero()};
        Eigen::Vector3d delta_theta_body{Eigen::Vector3d::Zero()};
        Eigen::Vector3d omega_body_fallback{Eigen::Vector3d::Zero()};
        bool valid{false};
    };

    /** 决定是否允许 STA 学习扰动（避免初始化/低接触置信阶段积分噪声） */
    struct STAGatingInput
    {
        bool allow_update{false};
        bool dynamics_valid{false};
        double contact_trust{0.0};
        int stance_leg_count{0};
    };

    /** 分通道 STA 参数：位置 / 速度 / 姿态 独立调参 */
    struct STAChannelParams
    {
        double bandwidth{5.0};
        double error_scale{1.0};
        double max_disturbance{10.0};
        double output_gain{0.35};
        double innovation_gate{1.0};
        bool enable_learning{true};
    };

    class STAObserver
    {
    public:
        STAObserver() = default;
        ~STAObserver() = default;

        void init(double k1, double k2);

        void update(double dt, const ESKFNominalState &state, const STADynamicsInput &dynamics,
                    const STAGatingInput &gating);

        void reset(const ESKFNominalState &state);

        void setChannelParameters(const STAChannelParams &pos, const STAChannelParams &vel,
                                  const STAChannelParams &rot, int covariance_window, double epsilon,
                                  double covariance_regularization, double max_covariance,
                                  double disturbance_lpf, double gated_decay);

        [[nodiscard]] const STADisturbanceVec &getDisturbance() const { return z1_filtered_; }

        [[nodiscard]] STADisturbanceCov getDisturbanceCovariance() const;

        [[nodiscard]] STADisturbanceCov computeDisturbanceCovariance(int window_size) const;

    private:
        STAStateVec z_pred_;
        STAStateVec z_;
        STAStateVec z_prev_;

        STADisturbanceVec z1_;
        STADisturbanceVec z1_prev_;
        STADisturbanceVec z1_smoothed_;
        STADisturbanceVec z1_filtered_;

        Eigen::Quaterniond z_q_{Eigen::Quaterniond::Identity()};
        Eigen::Quaterniond z_q_prev_{Eigen::Quaterniond::Identity()};
        Eigen::Quaterniond z_q_pred_{Eigen::Quaterniond::Identity()};

        STAChannelParams channel_pos_;
        STAChannelParams channel_vel_;
        STAChannelParams channel_rot_;

        std::vector<STADisturbanceVec> disturbance_history_;
        std::size_t max_history_size_ = 20;
        int covariance_window_ = 20;
        double epsilon_ = 1e-6;
        double covariance_regularization_ = 1e-6;
        double max_covariance_ = 10.0;
        double disturbance_lpf_ = 0.85;
        double gated_decay_ = 0.92;
        bool initialized_ = false;

        void syncVectorFromNominalState();

        [[nodiscard]] static Eigen::Quaterniond normalizeQuaternion(const Eigen::Quaterniond &q);

        [[nodiscard]] static Eigen::Vector3d rotationError(
            const Eigen::Quaterniond &q_meas, const Eigen::Quaterniond &q_pred);

        void predictNominalState(double dt, const STADynamicsInput &dynamics);

        /** 式 (21) 线性子块：z = z_pred + K1·e/||e||^{1/2}·dt + z1·dt，z1 += K2·e/||e||·dt */
        void updateStaLinearChannel(int state_idx, int dist_idx, const Eigen::Vector3d &z_pred_block,
                                    const Eigen::Vector3d &e_block, const STAChannelParams &params,
                                    double dt, const STAGatingInput &gating);

        /** 式 (21) 姿态子块（SO(3)）：乘法形式注入 K1 与 z1 */
        void updateStaRotationChannel(const Eigen::Vector3d &e_rot, const STAChannelParams &params,
                                        double dt, const STAGatingInput &gating);

        void filterDisturbanceOutput();

        [[nodiscard]] static double computeK1FromBandwidth(double bandwidth);
        [[nodiscard]] static double computeK2FromBandwidth(double bandwidth);
    };
}
