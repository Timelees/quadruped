#include "ocs2_quadruped_controller/estimator/STAObserver.h"
#include "ocs2_quadruped_controller/estimator/ErrorStateKalmanFilter.h" // for ESKFNominalState definition
#include <cmath>
#include <algorithm> // for clamp

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
        H_.setZero();
        // 初始化扰动输入矩阵B_d和扰动协方差矩阵Q_d
        Bd_.setIdentity();
        Qd_.setIdentity();

        disturbance_history_.reserve(max_history_size_);
        disturbance_history_.clear();
        disturbance_history_.push_back(z1_);
    }

    // STA更新过程
    void STAObserver::update(const double dt, ESKFNominalState &state, Eigen::Vector3d &acc_world)
    {
        z_.segment<3>(STAState::POS_IDX) = state.p;
        z_.segment<3>(STAState::VEL_IDX) = state.v;
        z_.segment<4>(STAState::ROT_IDX) = state.q.coeffs();

        std::cout << "ESKF 估计的状态state：" << std::endl;
        std::cout << "----p = " << z_.segment<3>(STAState::POS_IDX).transpose() << std::endl;
        std::cout << "----v = " << z_.segment<3>(STAState::VEL_IDX).transpose() << std::endl;
        std::cout << "----q = " << z_.segment<4>(STAState::ROT_IDX).transpose() << std::endl;

        double dt_ = dt;                               // 时间步长
        Eigen::Vector3d delta_theta = acc_world * dt_; // 旋转增量

        // 构建状态转移矩阵F
        F_.setIdentity();
        F_.block<3, 3>(STAState::POS_IDX, STAState::VEL_IDX) = dt_ * Eigen::Matrix3d::Identity();
        F_.block<4, 4>(STAState::ROT_IDX, STAState::ROT_IDX) = Sophus::SO3d::exp(delta_theta).matrix();

        // 构建输出矩阵H
        H_.setZero();
        H_.block<9, 9>(0, 0) = Eigen::MatrixXd::Identity(9, 9);

    // 计算跟踪误差：先从名义状态构建 STAStateVec，再计算误差
    STAStateVec x_vec = STAStateVec::Zero();
    x_vec.segment<3>(STAState::POS_IDX) = state.p;
    x_vec.segment<3>(STAState::VEL_IDX) = state.v;
    x_vec.segment<4>(STAState::ROT_IDX) = state.q.coeffs();
    STAStateVec e_ = H_ * x_vec - z_prev_;

        // 归一化处理
        double norm_sq = e_.squaredNorm();
        if (norm_sq < 1e-6)
        { // 避免除0
            e_ /= 1e-6;
        }
        STAStateVec e1_norm_ = e_ / std::sqrt(norm_sq);
        STAStateVec e2_norm_ = e_ / norm_sq;
        // 更新扰动估计
        z1_ = z1_prev_ + K2_ * e2_norm_ * dt_;

        // 更新状态估计
        z_ = F_ * z_prev_ + K1_ * e1_norm_ * dt_ + z1_ * dt_;

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
            double L_scalar = L(0, 0);
            K1_ = Eigen::MatrixXd::Identity(STAState::STATE_DIM, STAState::STATE_DIM) * (1.1 * L_scalar);
            K2_ = Eigen::MatrixXd::Identity(STADisturbance::DISTURBANCE_DIM, STADisturbance::DISTURBANCE_DIM) * (1.5 * std::sqrt(L_scalar));
        }
        else
        {
            // 矩阵形式的带宽
            K1_ = 1.1 * L;

            // 计算sqrt(L)（对角矩阵的平方根）
            Eigen::MatrixXd sqrt_L = Eigen::MatrixXd::Zero(STAState::STATE_DIM, STAState::STATE_DIM);
            for (int i = 0; i < STAState::STATE_DIM; ++i)
            {
                sqrt_L(i, i) = std::sqrt(L(i, i));
            }
            K2_ = 1.5 * sqrt_L;
        }
    }

    // 自适应计算扰动协方差矩阵
    STADisturbanceCov STAObserver::computeDisturbanceCovariance(int window_size)
    {
        if (disturbance_history_.size() < 2)
        {
            return STADisturbanceCov::Identity() * 0.01;
        }

        int n = std::min(static_cast<int>(disturbance_history_.size()), window_size);
        int start_idx = static_cast<int>(disturbance_history_.size()) - n;

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
        Q_d += STADisturbanceCov::Identity() * 1e-6;

        return Q_d;
    }

}