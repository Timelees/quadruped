#include "ocs2_quadruped_controller/estimator/STAObserver.h"
#include "ocs2_quadruped_controller/estimator/ErrorStateKalmanFilter.h"
#include <algorithm>
#include <cmath>

namespace ocs2::legged_robot
{
    double STAObserver::computeK1FromBandwidth(const double bandwidth)
    {
        return 1.1 * std::max(bandwidth, 0.0);
    }

    double STAObserver::computeK2FromBandwidth(const double bandwidth)
    {
        return 1.5 * std::sqrt(std::max(bandwidth, 0.0));
    }

    Eigen::Quaterniond STAObserver::normalizeQuaternion(const Eigen::Quaterniond &q)
    {
        Eigen::Quaterniond q_out = q;
        if (!q_out.coeffs().allFinite() || q_out.norm() < 1e-12)
        {
            return Eigen::Quaterniond::Identity();
        }
        q_out.normalize();
        return q_out;
    }

    Eigen::Vector3d STAObserver::rotationError(const Eigen::Quaterniond &q_meas,
                                               const Eigen::Quaterniond &q_pred)
    {
        // 右不变误差：与后续 z_q * Exp(delta) 的右乘注入保持一致。
        const Eigen::Quaterniond q_err = normalizeQuaternion(q_pred.inverse() * q_meas);
        return Sophus::SO3d(q_err).log();
    }

    void STAObserver::syncVectorFromNominalState()
    {
        z_.segment<3>(STAState::ROT_IDX) = Sophus::SO3d(z_q_).log();
        z_prev_.segment<3>(STAState::ROT_IDX) = Sophus::SO3d(z_q_prev_).log();
        z_pred_.segment<3>(STAState::ROT_IDX) = Sophus::SO3d(z_q_pred_).log();
    }

    void STAObserver::init(const double k1, const double k2)
    {
        (void)k1;
        (void)k2;
        z_ = STAStateVec::Zero();
        z1_ = STADisturbanceVec::Zero();
        z1_smoothed_ = STADisturbanceVec::Zero();
        z1_filtered_ = STADisturbanceVec::Zero();

        z_q_.setIdentity();
        z_q_prev_.setIdentity();
        z_q_pred_.setIdentity();

        disturbance_history_.reserve(max_history_size_);
        disturbance_history_.clear();
        disturbance_history_.push_back(z1_filtered_);
        z_prev_ = z_;
        z1_prev_ = z1_;
        initialized_ = false;
    }

    void STAObserver::setChannelParameters(const STAChannelParams &pos, const STAChannelParams &vel,
                                           const STAChannelParams &rot, const int covariance_window,
                                           const double epsilon, const double covariance_regularization,
                                           const double max_covariance, const double disturbance_lpf,
                                           const double gated_decay)
    {
        channel_pos_ = pos;
        channel_vel_ = vel;
        channel_rot_ = rot;

        covariance_window_ = std::max(2, covariance_window);
        max_history_size_ = static_cast<std::size_t>(covariance_window_);
        epsilon_ = std::max(epsilon, 1e-12);
        covariance_regularization_ = std::max(covariance_regularization, 0.0);
        max_covariance_ = std::max(max_covariance, covariance_regularization_);
        disturbance_lpf_ = std::clamp(disturbance_lpf, 0.0, 0.99);
        gated_decay_ = std::clamp(gated_decay, 0.0, 1.0);

        if (disturbance_history_.size() > max_history_size_)
        {
            const auto overflow = disturbance_history_.size() - max_history_size_;
            disturbance_history_.erase(disturbance_history_.begin(),
                                     disturbance_history_.begin() + static_cast<long>(overflow));
        }
    }

    void STAObserver::predictNominalState(const double dt, const STADynamicsInput &dynamics)
    {
        const Eigen::Vector3d p_prev = z_prev_.segment<3>(STAState::POS_IDX);
        const Eigen::Vector3d v_prev = z_prev_.segment<3>(STAState::VEL_IDX);

        if (dynamics.valid)
        {
            // dynamics 已由 OCS2 质心动力学给出 base 加速度和机体系姿态增量。
            z_pred_.segment<3>(STAState::POS_IDX) =
                p_prev + v_prev * dt + 0.5 * dynamics.accel_world * dt * dt;
            z_pred_.segment<3>(STAState::VEL_IDX) = v_prev + dynamics.accel_world * dt;
            z_q_pred_ = normalizeQuaternion(
                z_q_prev_ * Sophus::SO3d::exp(dynamics.delta_theta_body).unit_quaternion());
        }
        else
        {
            z_pred_.segment<3>(STAState::POS_IDX) = p_prev + v_prev * dt;
            z_pred_.segment<3>(STAState::VEL_IDX) = v_prev;
            z_q_pred_ = normalizeQuaternion(
                z_q_prev_ * Sophus::SO3d::exp(dynamics.omega_body_fallback * dt).unit_quaternion());
        }
    }

    void STAObserver::reset(const ESKFNominalState &state)
    {
        z_.segment<3>(STAState::POS_IDX) = state.p;
        z_.segment<3>(STAState::VEL_IDX) = state.v;
        z_q_ = normalizeQuaternion(state.q);

        z_prev_ = z_;
        z_q_prev_ = z_q_;
        z_q_pred_ = z_q_;
        syncVectorFromNominalState();

        z1_.setZero();
        z1_prev_.setZero();
        z1_smoothed_.setZero();
        z1_filtered_.setZero();
        disturbance_history_.clear();
        disturbance_history_.push_back(z1_filtered_);
        initialized_ = true;
    }

    void STAObserver::updateStaLinearChannel(const int state_idx, const int dist_idx,
                                               const Eigen::Vector3d &z_pred_block,
                                               const Eigen::Vector3d &e_block,
                                               const STAChannelParams &params, const double dt,
                                               const STAGatingInput &gating)
    {
        Eigen::Vector3d z_block = z_.segment<3>(state_idx);
        Eigen::Vector3d z1_block = z1_.segment<3>(dist_idx);
        const Eigen::Vector3d z1_prev_block = z1_prev_.segment<3>(dist_idx);

        const double scale = std::max(params.error_scale, 1e-6);
        const double innov_norm = (e_block / scale).norm();

        const bool gate_open = gating.allow_update && gating.dynamics_valid && params.enable_learning &&
                               innov_norm <= params.innovation_gate;
        const double trust = std::clamp(gating.contact_trust, 0.0, 1.0);

        if (!gate_open)
        {
            // 门控关闭：保留 z1 前馈，停止 K1/K2 滑模学习。
            z_block = z_pred_block + z1_prev_block * dt;
            z1_block = gated_decay_ * z1_prev_block;
        }
        else
        {
            const double norm = std::max(innov_norm, epsilon_);
            const double sqrt_norm = std::sqrt(norm);
            const double k1 = computeK1FromBandwidth(params.bandwidth);
            const double k2 = computeK2FromBandwidth(params.bandwidth);

            // Super-Twisting 式 (21)
            z_block = z_pred_block + k1 * (e_block / sqrt_norm) * dt * trust + z1_prev_block * dt;
            z1_block = z1_prev_block + k2 * (e_block / norm) * dt * trust;
        }

        const double block_norm = z1_block.norm();
        if (block_norm > params.max_disturbance)
        {
            z1_block *= params.max_disturbance / block_norm;
        }
        z_.segment<3>(state_idx) = z_block;
        z1_.segment<3>(dist_idx) = z1_block;
    }

    void STAObserver::updateStaRotationChannel(const Eigen::Vector3d &e_rot,
                                                   const STAChannelParams &params, const double dt,
                                                   const STAGatingInput &gating)
    {
        Eigen::Vector3d z1_rot = z1_.segment<3>(STADisturbance::ROT_IDX);
        const Eigen::Vector3d z1_prev_rot = z1_prev_.segment<3>(STADisturbance::ROT_IDX);

        const double scale = std::max(params.error_scale, 1e-6);
        const double innov_norm = (e_rot / scale).norm();

        const bool gate_open = gating.allow_update && gating.dynamics_valid && params.enable_learning &&
                               innov_norm <= params.innovation_gate;
        const double trust = std::clamp(gating.contact_trust, 0.0, 1.0);
        const Eigen::Quaterniond z_q_ff = normalizeQuaternion(
            z_q_pred_ * Sophus::SO3d::exp(z1_prev_rot * dt).unit_quaternion());

        if (!gate_open)
        {
            z_q_ = z_q_ff;
            z1_rot = gated_decay_ * z1_prev_rot;
        }
        else
        {
            const double norm = std::max(innov_norm, epsilon_);
            const double sqrt_norm = std::sqrt(norm);
            const double k1 = computeK1FromBandwidth(params.bandwidth);
            const double k2 = computeK2FromBandwidth(params.bandwidth);

            const Eigen::Vector3d twist = k1 * (e_rot / sqrt_norm) * dt * trust;
            z_q_ = normalizeQuaternion(z_q_ff * Sophus::SO3d::exp(twist).unit_quaternion());
            z1_rot = z1_prev_rot + k2 * (e_rot / norm) * dt * trust;
        }

        const double block_norm = z1_rot.norm();
        if (block_norm > params.max_disturbance)
        {
            z1_rot *= params.max_disturbance / block_norm;
        }
        z1_.segment<3>(STADisturbance::ROT_IDX) = z1_rot;
        z_.segment<3>(STAState::ROT_IDX) = Sophus::SO3d(z_q_).log();
    }

    void STAObserver::update(const double dt, const ESKFNominalState &state,
                             const STADynamicsInput &dynamics, const STAGatingInput &gating)
    {
        if (dt <= 0.0 || !std::isfinite(dt))
        {
            return;
        }
        if (!initialized_)
        {
            reset(state);
            return;
        }

        // 测量：ESKF 后验；预测：由上一时刻后验经 MPC 动力学传播得到的名义轨迹
        const Eigen::Vector3d p_meas = state.p;
        const Eigen::Vector3d v_meas = state.v;
        const Eigen::Quaterniond q_meas = normalizeQuaternion(state.q);

        predictNominalState(dt, dynamics);

        // 残差对齐到“动力学预测 + 上一时刻扰动前馈”后的 observer 预测状态，
        // 避免已由 z1_prev 解释的扰动在下一步被重复学习。
        const Eigen::Vector3d z_pos_ff =
            z_pred_.segment<3>(STAState::POS_IDX) + z1_prev_.segment<3>(STADisturbance::POS_IDX) * dt;
        const Eigen::Vector3d z_vel_ff =
            z_pred_.segment<3>(STAState::VEL_IDX) + z1_prev_.segment<3>(STADisturbance::VEL_IDX) * dt;
        const Eigen::Quaterniond z_q_ff = normalizeQuaternion(
            z_q_pred_ *
            Sophus::SO3d::exp(z1_prev_.segment<3>(STADisturbance::ROT_IDX) * dt).unit_quaternion());

        const Eigen::Vector3d e_pos = p_meas - z_pos_ff;
        const Eigen::Vector3d e_vel = v_meas - z_vel_ff;
        const Eigen::Vector3d e_rot = rotationError(q_meas, z_q_ff);

        // 式 (21) 双层 Super-Twisting：z 由 K1 项 + z1 前馈更新，z1 由 K2 积分

        updateStaLinearChannel(STAState::POS_IDX, STADisturbance::POS_IDX,
                               z_pred_.segment<3>(STAState::POS_IDX), e_pos, channel_pos_, dt, gating);
        updateStaLinearChannel(STAState::VEL_IDX, STADisturbance::VEL_IDX,
                               z_pred_.segment<3>(STAState::VEL_IDX), e_vel, channel_vel_, dt, gating);
        updateStaRotationChannel(e_rot, channel_rot_, dt, gating);

        filterDisturbanceOutput();

        z_prev_ = z_;
        z_q_prev_ = z_q_;
        syncVectorFromNominalState();
        z1_prev_ = z1_;

        disturbance_history_.push_back(z1_filtered_);
        if (disturbance_history_.size() > max_history_size_)
        {
            disturbance_history_.erase(disturbance_history_.begin());
        }
    }

    void STAObserver::filterDisturbanceOutput()
    {
        const double alpha = std::clamp(disturbance_lpf_, 0.0, 0.99);
        const auto blend = [&](const int idx, const double gain)
        {
            z1_smoothed_.segment<3>(idx) =
                alpha * z1_smoothed_.segment<3>(idx) + (1.0 - alpha) * z1_.segment<3>(idx);
            z1_filtered_.segment<3>(idx) = z1_smoothed_.segment<3>(idx) * gain;
        };

        blend(STADisturbance::POS_IDX, channel_pos_.output_gain);
        blend(STADisturbance::VEL_IDX, channel_vel_.output_gain);
        blend(STADisturbance::ROT_IDX, channel_rot_.output_gain);
    }

    STADisturbanceCov STAObserver::getDisturbanceCovariance() const
    {
        return computeDisturbanceCovariance(covariance_window_);
    }

    STADisturbanceCov STAObserver::computeDisturbanceCovariance(const int window_size) const
    {
        if (disturbance_history_.size() < 2)
        {
            return STADisturbanceCov::Identity() * covariance_regularization_;
        }

        const int n = std::max(2, std::min(static_cast<int>(disturbance_history_.size()), window_size));
        const int start_idx = static_cast<int>(disturbance_history_.size()) - n;

        STADisturbanceVec mean = STADisturbanceVec::Zero();
        for (int i = start_idx; i < static_cast<int>(disturbance_history_.size()); ++i)
        {
            mean += disturbance_history_[i];
        }
        mean /= static_cast<double>(n);

        STADisturbanceCov Q_d = STADisturbanceCov::Zero();
        for (int i = start_idx; i < static_cast<int>(disturbance_history_.size()); ++i)
        {
            const STADisturbanceVec err = disturbance_history_[i] - mean;
            Q_d += err * err.transpose();
        }
        Q_d /= static_cast<double>(n);

        Q_d += STADisturbanceCov::Identity() * covariance_regularization_;
        for (int i = 0; i < STADisturbance::DISTURBANCE_DIM; ++i)
        {
            Q_d(i, i) = std::clamp(Q_d(i, i), covariance_regularization_, max_covariance_);
        }
        return 0.5 * (Q_d + Q_d.transpose());
    }

} // namespace ocs2::legged_robot
