/*
 * Kalman filter based contact estimation for quadruped robots
*/

#include "ocs2_quadruped_controller/estimator/ContactEstimator.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <map>
#include <limits>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp> 
#include <pinocchio/algorithm/crba.hpp> 
#include <utility>
#include <chrono>
#include <atomic>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2::legged_robot
{
    ContactEstimator::ContactEstimator(PinocchioInterface pinocchio_interface, CentroidalModelInfo info,
                                       const PinocchioEndEffectorKinematics &ee_kinematics,
                                       CtrlInterfaces &ctrl_component,
                                       const rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : info_(std::move(info)),
          ctrl_component_(ctrl_component),
          pinocchio_interface_(std::move(pinocchio_interface)),
          ee_kinematics_(ee_kinematics.clone())
    {
        // 初始化机器人状态向量
        rbd_state_.setZero(2 * info_.generalizedCoordinatesNum);
        setup();
    }

    void ContactEstimator::setup()
    {
        const auto &model = pinocchio_interface_.getModel();
        const size_t nv = model.nv; // 广义速度维度 (18)

        A_.setZero(); // 状态转移矩阵
        B_.setIdentity();       // 输入矩阵
        H_.setZero();     // 观测矩阵 H: 观测 = [高度概率; 力概率]
        H_.block(0, 0, NUM_LEGS, NUM_LEGS) = Eigen::Matrix<scalar_t, NUM_LEGS, NUM_LEGS>::Identity();
        H_.block(NUM_LEGS, 0, NUM_LEGS, NUM_LEGS) = Eigen::Matrix<scalar_t, NUM_LEGS, NUM_LEGS>::Identity();
        I_.setIdentity(); // 单位矩阵
        // 过程噪声协方差 Q  降低过程噪声协方差会减小最优估计的协方差值
        Q_.setZero();
        Q_.block(0, 0, NUM_LEGS, NUM_LEGS) = contact_process_noise_ * Eigen::Matrix<scalar_t, NUM_LEGS, NUM_LEGS>::Identity();
        // 观测噪声协方差 R
        R_.setZero();
        R_.block(0, 0, NUM_LEGS, NUM_LEGS) = fheight_measure_noise_ * Eigen::Matrix<scalar_t, NUM_LEGS, NUM_LEGS>::Identity();           // 高度观测噪声
        R_.block(NUM_LEGS, NUM_LEGS, NUM_LEGS, NUM_LEGS) = tau_measure_noise_ * Eigen::Matrix<scalar_t, NUM_LEGS, NUM_LEGS>::Identity(); // 力观测噪声
        // 初始状态估计
        X_est_.setZero();
        P_est_.setIdentity();
        
        mu_fz_.fill(force_threshold_);
        
        qPino_ = vector_t::Zero(info_.generalizedCoordinatesNum);
        vPino_ = vector_t::Zero(info_.generalizedCoordinatesNum);
        zeroVel_ = vector_t::Zero(nv);
        J_full_.resize(6, nv);
        J_full_.setZero();
        qdot_.setZero();
        tau_input_.setZero();

        // 缓存足端帧索引
        for (int i = 0; i < NUM_LEGS; i++)
        {
            const auto frameId = model.getFrameId(foot_frame_names_[i]);
            ee_frame_ids_[i] = frameId;
        }

        // 足端力计算相关
        Jc_stack_.setZero();
        rd_.setZero();
        tauAct_.setZero();
        tauAct_prev_.setZero();
        Pk_.setZero();
        Xn_.setZero();
        Yn_1_.setZero();
        Yn_.setZero();

        for (int i = 0; i < 4; i++)
        {
            leg_parameters_[i].Jc.setZero();
            leg_parameters_[i].fi.setZero();
        }

        // 输出初始化
        contact_flags_.fill(false);
        contact_prob_.fill(0.0);

        ee_kinematics_->setPinocchioInterface(pinocchio_interface_);

        initialized_ = true;
    }

    void ContactEstimator::update(const std::array<scalar_t, NUM_LEGS> &foot_forces, const std::array<scalar_t, NUM_LEGS> &foot_heights,
                                  const std::array<scalar_t, NUM_LEGS> &contact_phase, const std::array<scalar_t, NUM_LEGS> &swing_phase)
    {
        if (!initialized_)
        {
            setup();
        }

        Eigen::Matrix<scalar_t, NUM_LEGS, 1> U;     // 输入：步态相位概率
        Eigen::Matrix<scalar_t, NUM_LEGS * 2, 1> Z; // 观测：[高度概率; 力概率]

        for (int i = 0; i < NUM_LEGS; ++i)
        {
            scalar_t gait_phase;
            if (swing_phase[i] == -1.0)  // 处于接触相
            {
                gait_phase = contact_phase[i];
                is_contact = 1;
                is_swing = 0;
            }
            else if (contact_phase[i] == -1.0) // 处于摆动相
            {
                gait_phase = swing_phase[i];
                is_contact = 0;
                is_swing = 1;
            }
            else
            {
                gait_phase = 0.5;
                is_contact = 1;
                is_swing = 0;
            }

            scalar_t P_gait = computeGaitPhaseProbability(gait_phase, i, is_swing, is_contact); // 基于步态相位的接触概率
            scalar_t P_height = computeHeightProbability(foot_heights[i], i);                   // 基于足端高度的接触概率
            scalar_t P_force = computeForceProbability(foot_forces[i], i, is_swing);            // 基于足端力的接触概率

            // std::cout << "Foot " << i << " - P_gait: " << P_gait << ", P_height: " << P_height << ", P_force: " << P_force << std::endl;

            if (std::isnan(P_gait)) P_gait = 0.0;
            if (std::isnan(P_height)) P_height = 0.0;
            if (std::isnan(P_force)) P_force = 0.0;

            // 构建输入和观测向量
            U(i) = P_gait;
            Z(i) = P_height;
            Z(i + NUM_LEGS) = P_force;
        }

        Q_.setZero();
        Q_.block(0, 0, NUM_LEGS, NUM_LEGS) = contact_process_noise_ * Eigen::Matrix<scalar_t, NUM_LEGS, NUM_LEGS>::Identity();
        R_.setZero();
        R_.block(0, 0, NUM_LEGS, NUM_LEGS) = fheight_measure_noise_ * Eigen::Matrix<scalar_t, NUM_LEGS, NUM_LEGS>::Identity();
        R_.block(NUM_LEGS, NUM_LEGS, NUM_LEGS, NUM_LEGS) = tau_measure_noise_ * Eigen::Matrix<scalar_t, NUM_LEGS, NUM_LEGS>::Identity();

        // 预测步骤
        X_pre_ = A_ * X_est_ + B_ * U;
        P_pre_ = A_ * P_est_ * A_.transpose() + Q_;
        // 更新步骤
        Vector8 Z_pred = H_ * X_pre_;
        Vector8 ey = Z - Z_pred;
        // 创新协方差
        Eigen::Matrix<scalar_t, NUM_LEGS * 2, NUM_LEGS * 2> S = H_ * P_pre_ * H_.transpose() + R_;
        // 卡尔曼增益 (使用LU分解求解)
        Vector8 S_inv_ey = S.lu().solve(ey);
        X_est_ = X_pre_ + P_pre_ * H_.transpose() * S_inv_ey;
        Eigen::Matrix<scalar_t, NUM_LEGS * 2, NUM_LEGS> S_inv_H = S.lu().solve(H_);
        P_est_ = (I_ - P_pre_ * H_.transpose() * S_inv_H) * P_pre_;
        // 确保协方差矩阵对称
        P_est_ = (P_est_ + P_est_.transpose()) * 0.5;

        for (int i = 0; i < NUM_LEGS; ++i)
        {
            contact_prob_[i] = std::clamp(X_est_(i), 0.0, 1.0);
            contact_flags_[i] = (contact_prob_[i] > contact_threshold_);
        }
    }

    // 动力学参数更新
    void ContactEstimator::updateDynamicsParameters()
    {
        const auto &model = pinocchio_interface_.getModel();
        auto &data = pinocchio_interface_.getData();
        const size_t actuatedDofNum = info_.actuatedDofNum;
        const size_t nv = model.nv; // 广义速度维度 (18)

        // 构建广义位置和速度向量
        qPino_.head<3>() = rbd_state_.segment<3>(3);                         // base position
        qPino_.segment<3>(3) = rbd_state_.head<3>();                         // 姿态 ZYX
        qPino_.tail(actuatedDofNum) = rbd_state_.segment(6, actuatedDofNum); // 关节位置

        vPino_.head<3>() = rbd_state_.segment<3>(info_.generalizedCoordinatesNum + 3);  // base 线速度 (world)
        vPino_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            qPino_.segment<3>(3),                                                        // 当前 ZYX 欧拉角
            rbd_state_.segment<3>(info_.generalizedCoordinatesNum));                      // 全局角速度 → 欧拉角速率
        vPino_.tail(actuatedDofNum) = rbd_state_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum); // 关节速度

        // 前向运动学和雅可比计算
        pinocchio::forwardKinematics(model, data, qPino_, vPino_);
        pinocchio::computeJointJacobians(model, data, qPino_);
        pinocchio::updateFramePlacements(model, data);

        // 足端接触雅可比矩阵提取 (3×nv 线性部分)
        for (int i = 0; i < NUM_LEGS; i++)
        {
            J_full_.setZero();
            const auto frameId = static_cast<pinocchio::FrameIndex>(ee_frame_ids_[i]);
            pinocchio::getFrameJacobian(model, data, frameId, pinocchio::LOCAL_WORLD_ALIGNED, J_full_);
            leg_parameters_[i].Jc = J_full_.topRows<3>();   // 雅可比矩阵线速度部分（3×nv）
            // 填充堆叠雅可比 Jc_stack_ (12×nv): [Jc0; Jc1; Jc2; Jc3]
            Jc_stack_.block<3, 18>(3 * i, 0) = leg_parameters_[i].Jc;
        }

        // 质量矩阵
        pinocchio::crba(model, data, qPino_);
        // pinocchio 的 CRBA 只填充上三角，使用 triangularView 高效对称化
        data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
        dynamics_parameters_.M = data.M;

        // 非线性效应 (科氏力 + 重力)
        pinocchio::nonLinearEffects(model, data, qPino_, vPino_);
        dynamics_parameters_.nle = data.nle; // C(q,v)*v + g(q)

        // 科氏力矩阵 C(q,v)
        pinocchio::computeCoriolisMatrix(model, data, qPino_, vPino_);
        dynamics_parameters_.C = data.C;
        
        // 重力项 G(q)
       dynamics_parameters_.grav = dynamics_parameters_.nle - dynamics_parameters_.C * vPino_;
    }

    // 计算足端力 - 基于动量观测器
    void ContactEstimator::computeFootForce(scalar_t dt)
    {
        const size_t actuatedDofNum = info_.actuatedDofNum;

        // 构建广义速度向量
        qdot_ = vPino_;

        // 构建广义力向量 (仅关节力矩)
        tau_input_.setZero();
        tau_input_.tail(actuatedDofNum) = tauAct_;

        // 动量观测器 (Momentum Observer)
        Pk_ = dynamics_parameters_.M * qdot_;   // 动量: P = M * qdot
        
        // 滤波器系数
        dt_ = dt;
        gamma_ = std::exp(-lambda_ * dt_);
        const scalar_t beta = (1.0 - gamma_) / (gamma_ * dt_);

        // 未滤波外力估计:
        Xn_ = beta * Pk_ + tau_input_ + dynamics_parameters_.C.transpose() * qdot_ - dynamics_parameters_.grav;

        // 一阶低通滤波 (指数滑动平均)
        Yn_1_ = Yn_;
        Yn_ = (1.0 - gamma_) * Xn_ + gamma_ * Yn_1_;
        
        // 最终扰动力估计 
        rd_ = beta * Pk_ - Yn_;

        // 从广义外力计算各腿足端力
        const Eigen::Matrix<double, 18, 18>& M = dynamics_parameters_.M;
        Eigen::LLT<Eigen::Matrix<double, 18, 18>> M_llt(M);
        Eigen::Matrix<double, 18, 1> Minv_rd = M_llt.solve(rd_);

        // Jc * M^{-1} * Jc^T  (12×12) — 操作空间惯性矩阵的逆
        Eigen::Matrix<double, 12, 18> Minv_JcT;
        for (int col = 0; col < 12; ++col)
        {
            Minv_JcT.row(col) = M_llt.solve(Jc_stack_.row(col).transpose()).transpose();
        }
        Eigen::Matrix<double, 12, 12> Lambda_inv = Jc_stack_ * M_llt.solve(Jc_stack_.transpose());

        // 正则化防止奇异
        constexpr double reg_eps = 1e-6;
        Lambda_inv.diagonal().array() += reg_eps;

        // 求解: f_stack = Lambda_inv^{-1} * Jc * M^{-1} * rd
        Eigen::Matrix<double, 12, 1> Jc_Minv_rd = Jc_stack_ * Minv_rd;
        Eigen::LLT<Eigen::Matrix<double, 12, 12>> Lambda_inv_llt(Lambda_inv);
        Eigen::Matrix<double, 12, 1> f_stack = Lambda_inv_llt.solve(Jc_Minv_rd);

        // 提取各腿足端力并低通滤波
        for (int i = 0; i < NUM_LEGS; i++)
        {
            leg_parameters_[i].fi = f_stack.segment<3>(3 * i);
            foot_forces_[i] = leg_parameters_[i].fi(2);
            // std::cout << "[ContactEstimator] leg[" << i << "] (" << foot_frame_names_[i]
            //           << ") foot_force_z: " << foot_forces_[i] << std::endl;
        }
    }

    scalar_t ContactEstimator::computeGaitPhaseProbability(scalar_t gait_phase, int leg_idx, int is_swing, int is_contact) 
    {
        // 使用误差函数计算概率
        constexpr scalar_t sqrt2 = 1.41421356237;

        scalar_t P_c0 = std::erf((gait_phase - mu_c0_) / (sigma_c0_ * sqrt2));
        scalar_t P_c1 = std::erf((mu_c1_ - gait_phase) / (sigma_c1_ * sqrt2));
        scalar_t P_s0 = std::erf((mu_s0_ - gait_phase) / (sigma_s0_ * sqrt2));
        scalar_t P_s1 = std::erf((gait_phase - mu_s1_) / (sigma_s1_ * sqrt2));

        P_contact_[leg_idx] = (is_contact * (P_c0 + P_c1) + is_swing * (2.0 + P_s0 + P_s1)) * 0.5;

        return P_contact_[leg_idx];
    }

    scalar_t ContactEstimator::computeHeightProbability(scalar_t foot_height, int leg_idx) 
    {
        // 足端越接近地面，接触概率越高
        constexpr scalar_t sqrt2 = 1.41421356237;
        // foot_height < mu_hz_时，更可能接触，概率大于0.5
        P_height_[leg_idx] = 0.5 * (1.0 + std::erf((mu_hz_ - foot_height) / (sigma_hz_ * sqrt2)));

        return P_height_[leg_idx];
    }

    scalar_t ContactEstimator::computeForceProbability(scalar_t foot_force, int leg_idx, bool is_swing)
    {
        // 力概率计算
        constexpr scalar_t sqrt2 = 1.41421356237;

        const scalar_t force = foot_force;
        const scalar_t sigma = std::max<scalar_t>(sigma_fz_, 1e-6);
        
        scalar_t raw_prob = 0.5 * (1.0 + std::erf((force - force_threshold_) / (sigma * sqrt2)));
        
        // 一阶低通滤波平滑概率变化，避免抖动
        constexpr scalar_t prob_alpha = 0.5; // 滤波系数
        P_force_[leg_idx] = prob_alpha * raw_prob + (1.0 - prob_alpha) * P_force_[leg_idx];

        return P_force_[leg_idx];
    }

    void ContactEstimator::loadSettings(const std::string &task_file, const bool verbose)
    {
        boost::property_tree::ptree pt;
        read_info(task_file, pt);
        const std::string prefix = "contactEstimator.";
        if (verbose)
        {
            std::cerr << "\n #### contactEstimator Noise:";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, contact_process_noise_, prefix + "contactProcessNoise", verbose);
        loadData::loadPtreeValue(pt, tau_measure_noise_, prefix + "tauMeasureNoise", verbose);
        loadData::loadPtreeValue(pt, fheight_measure_noise_, prefix + "fheightMeasureNoise", verbose);
        loadData::loadPtreeValue(pt, sigma_c0_, prefix + "contactPhaseSigma", verbose);
        loadData::loadPtreeValue(pt, sigma_c1_, prefix + "contactPhaseSigma", verbose);
        loadData::loadPtreeValue(pt, sigma_s0_, prefix + "swingPhaseSigma", verbose);
        loadData::loadPtreeValue(pt, sigma_s1_, prefix + "swingPhaseSigma", verbose);
        loadData::loadPtreeValue(pt, mu_hz_, prefix + "footHeightMu", verbose);
        loadData::loadPtreeValue(pt, sigma_hz_, prefix + "footHeightSigma", verbose);
        loadData::loadPtreeValue(pt, sigma_fz_, prefix + "footForceSigma", verbose);
        loadData::loadPtreeValue(pt, force_threshold_, prefix + "footForceThreshold", verbose);
    }

} // namespace ocs2::legged_robot
