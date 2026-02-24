//
// Kalman filter based contact estimation for quadruped robots
//

#include "ocs2_quadruped_controller/estimator/ContactEstimator.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <map>
#include <limits>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp> // 逆动力学（重力、科氏力）
#include <pinocchio/algorithm/crba.hpp> // 质量矩阵
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
        
        // 力低通滤波参数
        force_filtered_.fill(0.0);
        force_filter_alpha_ = 0.3;   // 滤波系数，越大响应越快
        swing_force_decay_ = 0.7;    // 摆动相力衰减系数
        
        // 足端力估计的历史值初始化
        foot_force_z_filtered_.fill(0.0);

   
        // ========== 预分配向量/矩阵初始化 ==========
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
        S_.setZero();
        ST_.setZero();
        ST_.block<12, 12>(6, 0) = Eigen::Matrix<double, 12, 12>::Identity();
        S_ = ST_.transpose();
        rd_.setZero();
        tauAct_.setZero();
        tauAct_prev_.setZero();
        Pk_.setZero();
        Xn_.setZero();
        Yn_1_.setZero();
        Yn_.setZero();

        for (int i = 0; i < 4; i++)
        {
            // 初始化选择矩阵 Sl: 每条腿在广义坐标中对应的3个关节位置
            // 广义坐标: [base(6), FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf,
            //            RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf]
            int joint_offset = 6 + i * 3; // 该腿第一个关节在广义坐标中的位置
            leg_parameters_[i].Sl.setZero();
            leg_parameters_[i].Sl.block<3, 3>(0, joint_offset) = Eigen::Matrix3d::Identity();
            leg_parameters_[i].Jc.setZero();
            leg_parameters_[i].fi.setZero();
        }

        // 输出初始化
        contact_flags_.fill(false);
        contact_prob_.fill(0.0);
        contact_phase_out_.fill(0.0);

        ee_kinematics_->setPinocchioInterface(pinocchio_interface_);

   
        
        logger.setEnabled(true);

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

        // std::array<bool, NUM_LEGS> is_swing_array;
        // std::array<scalar_t, NUM_LEGS> gait_phase_array;
    
        
        for (int i = 0; i < NUM_LEGS; ++i)
        {
            // 选择当前有效的相位值
            scalar_t gait_phase;
            if (swing_phase[i] >= 0.0)  // 摆动相
            {
                gait_phase = swing_phase[i];
                is_contact = 0;
                is_swing = 1;
            }
            else if (contact_phase[i] >= 0.0)
            {
                gait_phase = contact_phase[i];
                is_contact = 1;
                is_swing = 0;
            }
            else
            {
                gait_phase = 0.5;
            }

            // 计算各概率
            scalar_t P_gait = computeGaitPhaseProbability(gait_phase, i, is_swing, is_contact);
            scalar_t P_height = computeHeightProbability(foot_heights[i], i);
            scalar_t P_force = computeForceProbability(foot_forces[i], i, is_swing); 

            // NaN 保护（参考 cyber_guide ContactEstimator）
            if (std::isnan(P_gait)) P_gait = 0.0;
            if (std::isnan(P_height)) P_height = 0.0;
            if (std::isnan(P_force)) P_force = 0.0;

            // 存储调试信息
            debug_info_.P_gait[i] = P_gait;
            debug_info_.P_position[i] = P_height;
            debug_info_.P_force[i] = P_force;

            // 构建输入和观测向量
            U(i) = P_gait;
            Z(i) = P_height;
            Z(i + NUM_LEGS) = P_force;
        }



        // ========== 重新设置噪声协方差（与参考代码保持一致，每次更新重置） ==========
        Q_.setZero();
        Q_.block(0, 0, NUM_LEGS, NUM_LEGS) = contact_process_noise_ * Eigen::Matrix<scalar_t, NUM_LEGS, NUM_LEGS>::Identity();
        R_.setZero();
        R_.block(0, 0, NUM_LEGS, NUM_LEGS) = fheight_measure_noise_ * Eigen::Matrix<scalar_t, NUM_LEGS, NUM_LEGS>::Identity();
        R_.block(NUM_LEGS, NUM_LEGS, NUM_LEGS, NUM_LEGS) = tau_measure_noise_ * Eigen::Matrix<scalar_t, NUM_LEGS, NUM_LEGS>::Identity();

        // ========== 卡尔曼滤波 ==========
        // 预测步骤（平滑先验）
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

        

        // ========== 输出结果 ==========
        for (int i = 0; i < NUM_LEGS; ++i)
        {
            // 限制估计值在 [0, 1] 范围内
            contact_prob_[i] = std::clamp(X_est_(i), 0.0, 1.0);
            contact_flags_[i] = (contact_prob_[i] > contact_threshold_);
            contact_phase_out_[i] = contact_prob_[i];

            debug_info_.X_est[i] = X_est_(i);
        }
        // 限频打印原始量，用于确认单位/坐标系是否一致，否则 P_position/P_force 很容易长期饱和。
        static int printDecimator = 0;
        if (++printDecimator >= 5) { 
            printDecimator = 0;
            std::cout << std::fixed << std::setprecision(4);
            std::cout << "┌────────────────────────────────────────────────────────────────────────────────────────────────┐\n";
            std::cout << "│ Leg │ height(m) │ est_Fz(N) │ raw_Fz(N) │ P_gait │  P_h   │  P_f   │ Kalman P │ Contact │\n";
            std::cout << "├─────┼───────────┼───────────┼───────────┼────────┼────────┼────────┼──────────┼─────────┤\n";
            for (int i = 0; i < NUM_LEGS; ++i) {
                const char* est_str = contact_flags_[i] ? "YES" : "NO ";
                std::cout << "│  " << i << "  │   " << std::setw(7) << foot_heights[i]
                          << " │  " << std::setw(7) << foot_forces_[i]
                          << "  │  " << std::setw(7) << std::abs(foot_forces[i])
                          << "  │ " << std::setw(6) << debug_info_.P_gait[i]
                          << " │ " << std::setw(6) << debug_info_.P_position[i]
                          << " │ " << std::setw(6) << debug_info_.P_force[i]
                          << " │  " << std::setw(6) << X_est_(i)
                          << "  │  " << est_str
                          << "    │\n";
            }
            std::cout << "└─────┴───────────┴───────────┴───────────┴────────┴────────┴────────┴──────────┴─────────┘\n";
        }

    }

    // 动力学参数更新
    void ContactEstimator::updateDynamicsParameters()
    {
        const auto &model = pinocchio_interface_.getModel();
        auto &data = pinocchio_interface_.getData();
        const size_t actuatedDofNum = info_.actuatedDofNum;
        const size_t nv = model.nv; // 广义速度维度 (18)

        // ========== 1. 构建广义位置和速度向量 ==========
        // 使用成员变量避免每次分配
        qPino_.segment<3>(0) = rbd_state_.segment<3>(3);                     // base position
        qPino_.segment<3>(3) = rbd_state_.head<3>();                         // 姿态 ZYX
        qPino_.tail(actuatedDofNum) = rbd_state_.segment(6, actuatedDofNum); // 关节位置

        // 速度: 从世界系角速度转换到欧拉角导数
        vPino_.segment<3>(0) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            qPino_.segment<3>(3),
            rbd_state_.segment<3>(info_.generalizedCoordinatesNum));                                           // base角速度
        vPino_.segment<3>(3) = rbd_state_.segment<3>(info_.generalizedCoordinatesNum + 3);                     // base线速度
        vPino_.tail(actuatedDofNum) = rbd_state_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum); // 关节速度

        // ========== 2. 前向运动学和雅可比计算 ==========
        pinocchio::forwardKinematics(model, data, qPino_, vPino_);
        pinocchio::computeJointJacobians(model, data, qPino_);
        pinocchio::updateFramePlacements(model, data);

        // ========== 3. 足端接触雅可比矩阵提取 (3×nv 线性部分) ==========
        // 参考代码使用完整的接触雅可比 Jc (3×nDof)，而非仅关节部分
        for (int i = 0; i < NUM_LEGS; i++)
        {
            J_full_.setZero();
            const auto frameId = static_cast<pinocchio::FrameIndex>(ee_frame_ids_[i]);

            // LOCAL_WORLD_ALIGNED: 雅可比在世界坐标系下表达
            pinocchio::getFrameJacobian(model, data, frameId, pinocchio::LOCAL_WORLD_ALIGNED, J_full_);

            // 提取雅可比矩阵线速度部分
            leg_parameters_[i].Jc = J_full_.topRows<3>(); // 完整的 3×nv 接触雅可比
        }

        // ========== 4. 质量矩阵 ==========
        pinocchio::crba(model, data, qPino_);
        // pinocchio 的 CRBA 只填充上三角，使用 triangularView 高效对称化
        data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
        dynamics_parameters_.M = data.M;

        // ========== 5. 重力项 G(q) ==========
        // 使用零速度调用 rnea 获得纯重力项
        dynamics_parameters_.grav = pinocchio::rnea(model, data, qPino_, zeroVel_, zeroVel_);

        // ========== 6. 非线性效应 (科氏力 + 重力) ==========
        pinocchio::nonLinearEffects(model, data, qPino_, vPino_);
        dynamics_parameters_.nle = data.nle; // C(q,v)*v + g(q)

        // 科氏力矩阵 C(q,v)
        pinocchio::computeCoriolisMatrix(model, data, qPino_, vPino_);
        dynamics_parameters_.C = data.C;

        std::cout << "dynamics_parameters_: " << std::endl;
        std::cout << "M: " << std::endl << dynamics_parameters_.M << std::endl;
        std::cout << "g: " << std::endl << dynamics_parameters_.grav << std::endl;
        std::cout << "C: " << std::endl << dynamics_parameters_.C << std::endl;
    }

    // 计算足端力 - 基于动量观测器 (参考 cyber_guide ContactEstimator)
    void ContactEstimator::computeFootForce(scalar_t dt)
    {
        const size_t actuatedDofNum = info_.actuatedDofNum;

        // ========== 1. 构建广义速度向量 ==========
        qdot_ = vPino_;

        // ========== 2. 构建广义力向量 (仅关节力矩) ==========
        tau_input_.setZero();
        tau_input_.tail(actuatedDofNum) = tauAct_;

        // ========== 3. 动量观测器 (Momentum Observer) ==========
        // 参考: "Contact State Estimation using Kalman Filtering"
        // 动量: P = M * qdot
        Pk_ = dynamics_parameters_.M * qdot_;
        
        // 滤波器系数
        dt_ = dt;
        gamma_ = std::exp(-lambda_ * dt_);
        const scalar_t beta = (1.0 - gamma_) / (gamma_ * dt_);

        // 未滤波外力估计:
        // X = beta * P + tau_input + C^T * qdot - g(q)
        // 注：此处使用 C^T * qdot (科氏力矩阵转置乘以速度)
        Xn_ = beta * Pk_ + tau_input_ + dynamics_parameters_.C.transpose() * qdot_ - dynamics_parameters_.grav;

        // 一阶低通滤波 (指数滑动平均)
        Yn_1_ = Yn_;
        Yn_ = (1.0 - gamma_) * Xn_ + gamma_ * Yn_1_;
        
        // 最终扰动力估计 (广义外力)
        // rd = beta * P - Y  (这是对系统外广义力的估计)
        rd_ = beta * Pk_ - Yn_;

        // ========== 4. 从广义外力计算各腿足端力 ==========
        // 参考代码方法: fi = (Sl * Jc^T)^{-1} * Sl * rd
        // Sl: 选择矩阵 (3×nv), 仅该腿关节对应位置为单位阵
        // Jc: 接触雅可比 (3×nv), 完整的线性速度雅可比
        // rd: 广义扰动力 (nv×1)
        // 
        // 物理意义: 利用虚功原理 tau_leg = Jc^T * f_contact
        // 选择矩阵 Sl 提取该腿关节对应的广义力分量
        // Sl * rd ≈ Sl * Jc^T * f，求解 f = (Sl * Jc^T)^{-1} * Sl * rd
        for (int i = 0; i < NUM_LEGS; i++)
        {
            // Sl * Jc^T -> (3×nv) * (nv×3) = (3×3)
            Eigen::Matrix3d SlJcT = leg_parameters_[i].Sl * leg_parameters_[i].Jc.transpose();
            
            // Sl * rd -> (3×nv) * (nv×1) = (3×1)
            Eigen::Vector3d Sl_rd = leg_parameters_[i].Sl * rd_;
            
            // 求解足端力: fi = (Sl * Jc^T)^{-1} * Sl * rd
            // 添加正则化防止奇异
            const double eps = 1e-8;
            Eigen::Matrix3d SlJcT_reg = SlJcT;
            SlJcT_reg.diagonal().array() += eps;
            
            leg_parameters_[i].fi = SlJcT_reg.inverse() * Sl_rd;
            
            // 动量观测器估计的足端力 fi(2) 符号说明：
            // 在 LOCAL_WORLD_ALIGNED 坐标系下，地面对足端的支撑力方向向上 (z+)
            // rd = beta*P - Y 代表外广义力，接触力通过 Jc^T * f 映射
            // 因此 fi(2) > 0 表示有接触 (地面推足端向上)
            // 直接使用 fi(2) 作为接触力指标，无需取负
            const scalar_t raw_fz = leg_parameters_[i].fi(2);
            
            // 对足端力 z 分量做低通滤波，减少动量观测器的噪声
            constexpr scalar_t force_lpf_alpha = 0.3; // 滤波系数 (0~1, 越小越平滑)
            foot_force_z_filtered_[i] = force_lpf_alpha * raw_fz + (1.0 - force_lpf_alpha) * foot_force_z_filtered_[i];
            
            foot_forces_[i] = foot_force_z_filtered_[i];
        }
    }

    scalar_t ContactEstimator::computeGaitPhaseProbability(scalar_t gait_phase, int leg_idx, int is_swing, int is_contact) 
    {
        // 使用误差函数计算概率（参考原始 ContactEstimator）
        // erf(x) 将高斯分布映射到 [-1, 1]，再归一化到 [0, 1]
        constexpr scalar_t sqrt2 = 1.41421356237;

        scalar_t P_c0 = std::erf((gait_phase - mu_c0_) / (sigma_c0_ * sqrt2));
        scalar_t P_c1 = std::erf((mu_c1_ - gait_phase) / (sigma_c1_ * sqrt2));
        scalar_t P_s0 = std::erf((mu_s0_ - gait_phase) / (sigma_s0_ * sqrt2));
        scalar_t P_s1 = std::erf((gait_phase - mu_s1_) / (sigma_s1_ * sqrt2));

        P_contact_[leg_idx] = (is_contact * (P_c0 + P_c1) + is_swing * (2.0 + P_s0 + P_s1)) * 0.5;

        // 限制概率范围 [0, 1]
        P_contact_[leg_idx] = std::clamp(P_contact_[leg_idx], 0.0, 1.0);

        return P_contact_[leg_idx];
    }

    scalar_t ContactEstimator::computeHeightProbability(scalar_t foot_height, int leg_idx) 
    {
        // 足端越接近地面，接触概率越高
        // 使用高斯CDF：P = 0.5 * (1 + erf((mu_hz - height) / (sigma_hz * sqrt(2))))
        constexpr scalar_t sqrt2 = 1.41421356237;
        // foot_height < mu_hz_时，更可能接触，概率大于0.5
        P_height_[leg_idx] = 0.5 * (1.0 + std::erf((mu_hz_ - foot_height) / (sigma_hz_ * sqrt2)));
        // std::cout << "[computeHeightProbability] leg[" << leg_idx << "] P (before clamp): " << P_height_[leg_idx] << std::endl;

        // 限制概率范围 [0, 1]
        P_height_[leg_idx] = std::clamp(P_height_[leg_idx], 0.0, 1.0);

        return P_height_[leg_idx];
    }

    scalar_t ContactEstimator::computeForceProbability(scalar_t foot_force, int leg_idx, bool is_swing)
    {
        // 力概率计算：
        // foot_force > 0 表示有接触力（地面推足端向上）
        // 使用 sigmoid 风格的概率曲线：
        //   P = 0.5 * (1 + erf((force - threshold) / (sigma * sqrt2)))
        // force > threshold 时 P > 0.5，force < 0 时 P ≈ 0
        constexpr scalar_t sqrt2 = 1.41421356237;

        const scalar_t force = foot_force;
        const scalar_t sigma = std::max<scalar_t>(sigma_fz_, 1e-6);
        
        // 直接的高斯 CDF 概率，不做额外的缩放
        // force_threshold_ 作为判定接触的力阈值（通常 10~20N）
        scalar_t raw_prob = 0.5 * (1.0 + std::erf((force - force_threshold_) / (sigma * sqrt2)));
        
        // 一阶低通滤波平滑概率变化，避免抖动
        constexpr scalar_t prob_alpha = 0.4; // 滤波系数
        P_force_[leg_idx] = prob_alpha * raw_prob + (1.0 - prob_alpha) * P_force_[leg_idx];

        P_force_[leg_idx] = std::clamp(P_force_[leg_idx], 0.0, 1.0);

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
        loadData::loadPtreeValue(pt, sigma_c0_, prefix + "phaseSigma", verbose);
        loadData::loadPtreeValue(pt, sigma_c1_, prefix + "phaseSigma", verbose);
        loadData::loadPtreeValue(pt, sigma_s0_, prefix + "swingPhaseSigma", verbose);
        loadData::loadPtreeValue(pt, sigma_s1_, prefix + "swingPhaseSigma", verbose);
        loadData::loadPtreeValue(pt, mu_hz_, prefix + "footHeightMu", verbose);
        loadData::loadPtreeValue(pt, sigma_hz_, prefix + "footHeightSigma", verbose);
        loadData::loadPtreeValue(pt, sigma_fz_, prefix + "footForceSigma", verbose);
        loadData::loadPtreeValue(pt, force_filter_alpha_, prefix + "forceFilterAlpha", verbose);
        loadData::loadPtreeValue(pt, swing_force_decay_, prefix + "swingForceDecay", verbose);
        loadData::loadPtreeValue(pt, force_threshold_, prefix + "footForceThreshold", verbose);
        loadData::loadPtreeValue(pt, contact_threshold_, prefix + "contactThreshold", verbose);
        loadData::loadPtreeValue(pt, lambda_, prefix + "observerCutoffFreq", verbose);
    }

} // namespace ocs2::legged_robot
