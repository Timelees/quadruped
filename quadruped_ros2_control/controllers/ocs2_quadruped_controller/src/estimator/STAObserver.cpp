#include "ocs2_quadruped_controller/estimator/STAObserver.h"
#include "ocs2_quadruped_controller/estimator/ErrorStateKalmanFilter.h" // for ESKFNominalState definition
#include <algorithm>
#include <cmath>

namespace ocs2::legged_robot
{
    void STAObserver::init(double k1, double k2)
    {
        // 初始化状态和扰动
        z_ = STAStateVec::Zero();
        z1_ = STADisturbanceVec::Zero();

        // 初始化增益矩阵
        K1_ = STAGainMatrix::Identity() * k1;
        K2_ = STAGainMatrix::Identity() * k2;

        // 初始化状态转移矩阵F和输出矩阵H
        F_.setIdentity();
        H_.setIdentity();
        // 初始化扰动输入矩阵B_d和扰动协方差矩阵Q_d
        Bd_.setIdentity();
        Qd_.setIdentity();
        Qd_ *= covariance_regularization_;

        disturbance_history_.reserve(max_history_size_);
        disturbance_history_.clear();
        disturbance_history_.push_back(z1_);
        z_prev_ = z_;
        z1_prev_ = z1_;
        initialized_ = false;
    }

    STAStateVec STAObserver::stateToVector(const ESKFNominalState &state)
    {
        STAStateVec x = STAStateVec::Zero();
        x.segment<3>(STAState::POS_IDX) = state.p;
        x.segment<3>(STAState::VEL_IDX) = state.v;
        x.segment<3>(STAState::ROT_IDX) = Sophus::SO3d(state.q.normalized()).log();
        return x;
    }

    void STAObserver::reset(const ESKFNominalState &state)
    {
        z_ = stateToVector(state);
        z_prev_ = z_;
        z1_.setZero();
        z1_prev_.setZero();
        disturbance_history_.clear();
        disturbance_history_.push_back(z1_);
        initialized_ = true;
    }

    // STA更新过程
    void STAObserver::update(double dt, const ESKFNominalState &state, const Eigen::Vector3d &omega_body)
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

        const STAStateVec x_vec = H_ * stateToVector(state);
        const Eigen::Vector3d delta_theta = omega_body * dt;

        // 构建状态转移矩阵F
        F_.setIdentity();
        F_.block<3, 3>(STAState::POS_IDX, STAState::VEL_IDX) = dt * Eigen::Matrix3d::Identity();
        F_.block<3, 3>(STAState::ROT_IDX, STAState::ROT_IDX) = Sophus::SO3d::exp(delta_theta).matrix();

        const STAStateVec e = x_vec - z_prev_;
        const double norm = std::max(e.norm(), epsilon_);
        const STAStateVec e_sqrt_norm = e / std::sqrt(norm);
        const STAStateVec e_norm = e / norm;

        // 论文式 (21)：先用上一时刻扰动推进 z，再积分得到新的 z1。
        z_ = F_ * z_prev_ + K1_ * e_sqrt_norm * dt + z1_prev_ * dt;
        z1_ = z1_prev_ + K2_ * e_norm * dt;
        z1_ = z1_.cwiseMax(-max_disturbance_).cwiseMin(max_disturbance_);

        // 更新历史状态和扰动
        z_prev_ = z_;
        z1_prev_ = z1_;

        // 保存扰动历史（用于自适应协方差计算）
        disturbance_history_.push_back(z1_);
        if (disturbance_history_.size() > max_history_size_)
        {
            disturbance_history_.erase(disturbance_history_.begin());
        }
    }

    // 设置增益矩阵
    void STAObserver::setGains(const Eigen::MatrixXd &L)
    {
        // 论文中的增益选择准则：K1 = 1.1 * L, K2 = 1.5 * sqrt(L)
        // 其中 L 是正定对角矩阵，代表观测器带宽

        if (L.rows() != STAState::STATE_DIM || L.cols() != STAState::STATE_DIM)
        {
            // 如果维度不匹配，使用标量L
            const double L_scalar = std::max(L(0, 0), 0.0);
            K1_ = STAGainMatrix::Identity() * (1.1 * L_scalar);
            K2_ = STAGainMatrix::Identity() * (1.5 * std::sqrt(L_scalar));
        }
        else
        {
            // 矩阵形式的带宽
            K1_ = 1.1 * L;

            // 计算sqrt(L)（对角矩阵的平方根）
            Eigen::MatrixXd sqrt_L = Eigen::MatrixXd::Zero(STAState::STATE_DIM, STAState::STATE_DIM);
            for (int i = 0; i < STAState::STATE_DIM; ++i)
            {
                sqrt_L(i, i) = std::sqrt(std::max(L(i, i), 0.0));
            }
            K2_ = 1.5 * sqrt_L;
        }
    }

    void STAObserver::setBandwidth(double bandwidth)
    {
        Eigen::Matrix<double, 1, 1> L;
        L(0, 0) = bandwidth;
        setGains(L);
    }

    void STAObserver::setParameters(int covariance_window, double epsilon, double covariance_regularization,
                                    double max_covariance, double max_disturbance)
    {
        covariance_window_ = std::max(2, covariance_window);
        max_history_size_ = static_cast<std::size_t>(covariance_window_);
        epsilon_ = std::max(epsilon, 1e-12);
        covariance_regularization_ = std::max(covariance_regularization, 0.0);
        max_covariance_ = std::max(max_covariance, covariance_regularization_);
        max_disturbance_ = std::max(max_disturbance, 0.0);

        if (disturbance_history_.size() > max_history_size_)
        {
            const auto overflow = disturbance_history_.size() - max_history_size_;
            disturbance_history_.erase(disturbance_history_.begin(),
                                       disturbance_history_.begin() + static_cast<long>(overflow));
        }
    }

    STADisturbanceCov STAObserver::getDisturbanceCovariance() const
    {
        return computeDisturbanceCovariance(covariance_window_);
    }

    // 自适应计算扰动协方差矩阵
    STADisturbanceCov STAObserver::computeDisturbanceCovariance(int window_size) const
    {
        if (disturbance_history_.size() < 2)
        {
            return STADisturbanceCov::Identity() * covariance_regularization_;
        }

        const int n = std::max(2, std::min(static_cast<int>(disturbance_history_.size()), window_size));
        const int start_idx = static_cast<int>(disturbance_history_.size()) - n;

        // 计算扰动均值
        STADisturbanceVec mean = STADisturbanceVec::Zero();
        for (int i = start_idx; i < static_cast<int>(disturbance_history_.size()); ++i)
        {
            mean += disturbance_history_[i];
        }
        mean /= static_cast<double>(n);

        // 计算经验协方差
        STADisturbanceCov Q_d = STADisturbanceCov::Zero();
        for (int i = start_idx; i < static_cast<int>(disturbance_history_.size()); ++i)
        {
            STADisturbanceVec e = disturbance_history_[i] - mean;
            Q_d += e * e.transpose();
        }
        Q_d /= static_cast<double>(n);

        // 添加小正则化项确保正定性
        Q_d += STADisturbanceCov::Identity() * covariance_regularization_;
        for (int i = 0; i < STADisturbance::DISTURBANCE_DIM; ++i)
        {
            Q_d(i, i) = std::clamp(Q_d(i, i), covariance_regularization_, max_covariance_);
        }
        Q_d = 0.5 * (Q_d + Q_d.transpose());

        return Q_d;
    }

}