#include <initializer_list>
#include <pinocchio/fwd.hpp>

#include "ocs2_quadruped_controller/estimator/ErrorStateKalmanFilter.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <utility>
#include <chrono>
#include <thread>
#include <limits>
#include <algorithm>
#include <sstream>
#include <iomanip>

#include <ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace
{
constexpr ocs2::scalar_t kTrotCycleDuration = 0.6;
constexpr int kTrotGaitSampleCount = 12;

bool isPureTrotGait(ocs2::scalar_t current_time,
                      const ocs2::legged_robot::SwitchedModelReferenceManager& reference_manager)
{
    const auto& mode_schedule = reference_manager.getModeSchedule();
    if (mode_schedule.modeSequence.empty())
    {
        return false;
    }

    const size_t current_mode = mode_schedule.modeAtTime(current_time);
    if (current_mode != ocs2::legged_robot::LF_RH && current_mode != ocs2::legged_robot::RF_LH)
    {
        return false;
    }

    bool has_lf_rh = false;
    bool has_rf_lh = false;
    for (int i = 0; i <= kTrotGaitSampleCount; ++i)
    {
        const ocs2::scalar_t sample_time =
            current_time +
            kTrotCycleDuration * static_cast<ocs2::scalar_t>(i) /
                static_cast<ocs2::scalar_t>(kTrotGaitSampleCount);
        const size_t mode = mode_schedule.modeAtTime(sample_time);
        if (mode != ocs2::legged_robot::LF_RH && mode != ocs2::legged_robot::RF_LH)
        {
            return false;
        }
        has_lf_rh |= mode == ocs2::legged_robot::LF_RH;
        has_rf_lh |= mode == ocs2::legged_robot::RF_LH;
    }

    return has_lf_rh && has_rf_lh;
}
}  // namespace

namespace ocs2::legged_robot
{
    ErrorStateKalmanFilter::ErrorStateKalmanFilter(PinocchioInterface pinocchio_interface, CentroidalModelInfo info,
                                                   const PinocchioEndEffectorKinematics &ee_kinematics,
                                                   CtrlInterfaces &ctrl_component,
                                                   const rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : StateEstimateBase(std::move(info), ctrl_component, node),
          pinocchio_interface_(std::move(pinocchio_interface)),
          ee_kinematics_(ee_kinematics.clone())
    {
        // 日志保存器初始化
        logger_ = std::make_unique<quadruped::logs::LogSave>();
        if (is_logger_enabled_)
        {
            logger_->setEnabled(true);
        }
        else
        {
            logger_->setEnabled(false);
        }

        ee_kinematics_->setPinocchioInterface(pinocchio_interface_);

        // 初始化滤波器状态
        init();

        // 初始化话题发布
        initPublishers();

        // 初始化 Ground Truth 发布器 (用于与状态估计对比)
        ground_truth_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("Ground_Truth", 10);

        ground_truth_enabled_ = true; // 启用 Ground Truth 对比

        // 初始化接触估计器
        if (use_contact_estimator_)
        {
            PinocchioInterface pinocchio_interface_copy(pinocchio_interface_);
            contact_estimator_ = std::make_unique<ContactEstimator>(std::move(pinocchio_interface_copy), info_,
                                                                    ee_kinematics, ctrl_component, node);
            RCLCPP_INFO(node_->get_logger(), "ContactEstimator initialized successfully.");
        }
    }

    ErrorStateKalmanFilter::~ErrorStateKalmanFilter()
    {
    }

    void ErrorStateKalmanFilter::init()
    {

        const auto &model = pinocchio_interface_.getModel();
        const size_t nv = model.nv; // 广义速度维度 (18)

        // 初始化名义状态
        xHat_.p = Eigen::Vector3d::Zero();
        xHat_.v = Eigen::Vector3d::Zero();
        xHat_.q = Eigen::Quaterniond::Identity();

        // 足端初始位置
        xHat_.f[0] = Eigen::Vector3d::Zero();
        xHat_.f[1] = Eigen::Vector3d::Zero();
        xHat_.f[2] = Eigen::Vector3d::Zero();
        xHat_.f[3] = Eigen::Vector3d::Zero();
        // 偏置
        xHat_.ba = Eigen::Vector3d::Zero();
        xHat_.bg = Eigen::Vector3d::Zero();

        // 缓存足端帧索引
        for (int i = 0; i < 4; i++)
        {
            const auto frameId = model.getFrameId(foot_frame_names_[i]);
            ee_frame_ids_[i] = frameId;
        }

        J_full_.resize(6, nv);
        J_full_.setZero();
        Jc.resize(4);
        for (auto &Ji : Jc)
        {
            Ji.resize(3, nv);
            Ji.setZero();
        }
        // 初始化状态误差
        dx.setZero();

        // 初始化协方差: 位置/速度/姿态/足端起手可以宽松，偏置要小一些，
        // 否则第一次观测会把 ba/bg 拉飞。
        P.setIdentity();
        P.block<3, 3>(ESKFNominalState::POS_IDX, ESKFNominalState::POS_IDX) *= 1e-2;   // 1cm σ
        P.block<3, 3>(ESKFNominalState::VEL_IDX, ESKFNominalState::VEL_IDX) *= 1e-2;   // 0.1 m/s σ
        P.block<3, 3>(ESKFNominalState::ROT_IDX, ESKFNominalState::ROT_IDX) *= 1e-2;   // ~6° σ
        for (int i = 0; i < 4; ++i)
        {
            const int f_idx = ESKFNominalState::F1_IDX + 3 * i;
            P.block<3, 3>(f_idx, f_idx) *= 1e-2;
        }
        P.block<3, 3>(ESKFNominalState::BA_IDX, ESKFNominalState::BA_IDX) *= 1e-4;     // 0.01 m/s² σ
        P.block<3, 3>(ESKFNominalState::BG_IDX, ESKFNominalState::BG_IDX) *= 1e-4;     // 0.01 rad/s σ

        Q.setIdentity();
        R.setIdentity();
    }

    vector_t ErrorStateKalmanFilter::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        double dt = period.seconds();
        controller_time_ += dt;
        // 更新基础状态（关节、接触、IMU）
        updateJointStates();
        if (use_contact_estimator_ && contact_estimator_){
            footStateEstimation(time, period); // 足端力和接触状态估计
        }else{
            updateContact();        // 使用足端力阈值进行接触状态更新
            for (int i = 0; i < 4; ++i)
            {
                contact_prob_[i] = contact_flag_[i] ? 1.0 : 0.0;
            }
        }
        updateImu();

        // 运动学计算
        // qPino 布局: [position(3), orientation(3), joint_angles(12)]
        // vPino 布局: [linear_vel(3), angular_vel(3), joint_velocities(12)]
        // 获取body系下的足端位置
        const auto &model = pinocchio_interface_.getModel();
        auto &data = pinocchio_interface_.getData();
        size_t actuatedDofNum = info_.actuatedDofNum; // 12
        // 构建 Pinocchio 需要的状态向量
        vector_t qPino(info_.generalizedCoordinatesNum); // 18维
        vector_t vPino(info_.generalizedCoordinatesNum); // 18维

        qPino.setZero();
        qPino.segment<3>(3) = rbd_state_.head<3>();                         // Only set orientation, let position in origin.
        qPino.tail(actuatedDofNum) = rbd_state_.segment(6, actuatedDofNum); // 关节角度

        vPino.setZero();
        vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            qPino.segment<3>(3),
            rbd_state_.segment<3>(info_.generalizedCoordinatesNum));
        vPino.tail(actuatedDofNum) = rbd_state_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum); // 关节速度

        // Pinocchio 正运动学计算
        forwardKinematics(model, data, qPino, vPino);
        computeJointJacobians(model, data, qPino);
        updateFramePlacements(model, data);

        // 获取足端位置和速度, 返回从基座坐标系原点（世界系原点）到足端的向量（世界系）
        foot_pos_ = ee_kinematics_->getPosition(vector_t());
        foot_vel_ = ee_kinematics_->getVelocity(vector_t(), vector_t());

        // 足端接触雅可比矩阵提取
        for (int i = 0; i < 4; i++)
        {
            J_full_.setZero();
            const auto frameId = static_cast<pinocchio::FrameIndex>(ee_frame_ids_[i]);
            pinocchio::getFrameJacobian(model, data, frameId, pinocchio::LOCAL_WORLD_ALIGNED, J_full_);
            Jc[i] = J_full_.topRows<3>();
        }

        // 初始化机体高度和足端世界坐标
        if (!feet_world_initialized_)
        {
            xHat_.q = quat_;
            xHat_.q.normalize();

            double base_height_sum = 0.0;
            int base_height_count = 0;
            for (int i = 0; i < 4; ++i)
            {
                if (contact_flag_[i])
                {
                    const scalar_t terrain_height = 0.0;
                    base_height_sum += static_cast<double>(terrain_height) + foot_radius_ - foot_pos_[i].z();
                    ++base_height_count;
                }
            }
            if (base_height_count == 0)
            {
                for (int i = 0; i < 4; ++i)
                {
                    const scalar_t terrain_height =  0.0;
                    base_height_sum += static_cast<double>(terrain_height) + foot_radius_ - foot_pos_[i].z();
                }
                base_height_count = 4;
            }
            xHat_.p.z() = base_height_sum / static_cast<double>(base_height_count);
            xHat_.v.setZero();

            for (int i = 0; i < 4; ++i)
            {
                xHat_.f[i] = xHat_.p + foot_pos_[i];
            }
            feet_world_initialized_ = true;
        }

        // ESKF 输入
        ESKFInput in;

        in.acc = linear_accel_local_;
        in.gyro = angular_vel_local_;
        // 暂时不加扰动
        in.dist_acc = Eigen::Vector3d::Zero();
        in.dist_vel = Eigen::Vector3d::Zero();
        in.dist_rot = Eigen::Vector3d::Zero();

        // 记录预测前的状态
        ESKFNominalState x_before_predict = xHat_; // 预测前状态

        // ESKF 预测
        eskfPrediction(dt, in);

        // 观测模型
        measurementModelKinematics(xHat_, in, y, C, R);

        // ESKF更新
        eskfUpdate(y, C, R);

        // IMU 姿态软约束：防止 ESKF 旋转估计偏离 IMU 的姿态信息
        fuseImuAttitude();

        // 更新 Ground Truth 数据
        updateGroundTruth();

        // 日志记录
        static int log_counter = 0;
        log_counter++;
        if (is_logger_enabled_ && (log_counter % 10 == 0))
        {
            // 状态估计值
            Eigen::Vector3d state_zyx = quatToZyx(xHat_.q);
            Eigen::Vector3d state_pos = xHat_.p;
            Eigen::Vector3d state_vel = xHat_.v;

            const double log_time = controller_time_;

            logger_->saveDataToCSV("state_pos.csv", "pos_x, pos_y, pos_z", log_time, state_pos);
            logger_->saveDataToCSV("state_vel.csv", "vel_x, vel_y, vel_z", log_time, state_vel);
            logger_->saveDataToCSV("state_zyx.csv", "yaw, pitch, roll", log_time, state_zyx);

            // Ground Truth 对比日志仅在纯 trot 步态下写入，避免站立/切步态阶段污染评估数据
            const bool is_trot_gait =
                reference_manager_ptr_ && isPureTrotGait(controller_time_, *reference_manager_ptr_);
            if (ground_truth_enabled_ && sim_mode_ && is_trot_gait)
            {
                const Eigen::Vector3d gt_rot_zyx = quatToZyx(gt_orientation_);

                const Eigen::Vector3d pos_error = state_pos - gt_position_;
                const Eigen::Vector3d vel_error = state_vel - gt_linear_velocity_;
                const Eigen::Vector3d rot_error = state_zyx - gt_rot_zyx;
                const Eigen::Vector3d angvel_error = angular_vel_local_ - gt_angular_velocity_;

                logger_->saveDataToCSV("pos_est_true_x.csv", "est_x, gt_x", log_time,
                                       std::initializer_list<double>{state_pos(0), gt_position_(0)});
                logger_->saveDataToCSV("pos_est_true_y.csv", "est_y, gt_y", log_time,
                                       std::initializer_list<double>{state_pos(1), gt_position_(1)});
                logger_->saveDataToCSV("pos_est_true_z.csv", "est_z, gt_z", log_time,
                                       std::initializer_list<double>{state_pos(2), gt_position_(2)});
                logger_->saveDataToCSV("pos_error.csv", "pos_x, pos_y, pos_z", log_time, pos_error);

                logger_->saveDataToCSV("vel_est_true_x.csv", "est_vx, gt_vx", log_time,
                                       std::initializer_list<double>{state_vel(0), gt_linear_velocity_(0)});
                logger_->saveDataToCSV("vel_est_true_y.csv", "est_vy, gt_vy", log_time,
                                       std::initializer_list<double>{state_vel(1), gt_linear_velocity_(1)});
                logger_->saveDataToCSV("vel_est_true_z.csv", "est_vz, gt_vz", log_time,
                                       std::initializer_list<double>{state_vel(2), gt_linear_velocity_(2)});
                logger_->saveDataToCSV("vel_error.csv", "vel_x, vel_y, vel_z", log_time, vel_error);

                logger_->saveDataToCSV("rot_est_true_yaw.csv", "est_yaw, gt_yaw", log_time,
                                       std::initializer_list<double>{state_zyx(0), gt_rot_zyx(0)});
                logger_->saveDataToCSV("rot_est_true_pitch.csv", "est_pitch, gt_pitch", log_time,
                                       std::initializer_list<double>{state_zyx(1), gt_rot_zyx(1)});
                logger_->saveDataToCSV("rot_est_true_roll.csv", "est_roll, gt_roll", log_time,
                                       std::initializer_list<double>{state_zyx(2), gt_rot_zyx(2)});
                logger_->saveDataToCSV("rot_error.csv", "rot_x, rot_y, rot_z", log_time, rot_error);

                logger_->saveDataToCSV("angvel_est_true_wx.csv", "est_wx, gt_wx", log_time,
                                       std::initializer_list<double>{angular_vel_local_(0), gt_angular_velocity_(0)});
                logger_->saveDataToCSV("angvel_est_true_wy.csv", "est_wy, gt_wy", log_time,
                                       std::initializer_list<double>{angular_vel_local_(1), gt_angular_velocity_(1)});
                logger_->saveDataToCSV("angvel_est_true_wz.csv", "est_wz, gt_wz", log_time,
                                       std::initializer_list<double>{angular_vel_local_(2), gt_angular_velocity_(2)});
                logger_->saveDataToCSV("angvel_error.csv", "angvel_x, angvel_y, angvel_z", log_time, angvel_error);
            }
        }

        // ==================== 把 ESKF 估计结果写回 rbd_state_ ====================
        // rbd_state_ 布局: [姿态 zyx(3) | base 位置(3) | 关节位置(12)
        //                  | base 角速度(3) | base 线速度(3) | 关节速度(12)]
        // 注意 base 角速度采用 OCS2 的 "global angular velocity"（Euler ZYX 导数变换出来的全局形式）。
        // 这里直接用 R_wb * (gyro - bg) 即可，等价但少一次中间转换。
        Eigen::Matrix3d R_wb_out = xHat_.q.toRotationMatrix();
        Eigen::Vector3d omega_body_out = in.gyro - xHat_.bg;
        Eigen::Vector3d omega_world_out = R_wb_out * omega_body_out;

        const Eigen::Vector3d p_body = xHat_.p;
        const Eigen::Vector3d v_body = xHat_.v;
        const vector3_t zyx_eskf = quatToZyx(xHat_.q);

        rbd_state_.segment<3>(0) = zyx_eskf;                                                      // 姿态(ESKF)
        rbd_state_.segment<3>(3) = p_body;                                                        // base 位置
        rbd_state_.segment<3>(info_.generalizedCoordinatesNum) = omega_world_out;                 // base 角速度(ESKF, world)
        rbd_state_.segment<3>(info_.generalizedCoordinatesNum + 3) = v_body;                      // base 线速度

        auto odom = getOdomMsg();
        odom.header.stamp = time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base";
        publishMsgs(odom);

        publishGroundTruth(time);

        return rbd_state_;
    }


    void ErrorStateKalmanFilter::eskfPrediction(double dt, const ESKFInput &u)
    {
        // 设置过程噪声 Q
        setCovarianceQ(Q, dt);

        Eigen::Vector3d a_unbias = u.acc - xHat_.ba;      // 去偏置后的加速度（body系）
        Eigen::Vector3d omega_unbias = u.gyro - xHat_.bg; // 去偏置后的陀螺仪角速度（body系）
        Eigen::Matrix3d R = xHat_.q.toRotationMatrix();   // 当前姿态旋转矩阵
        Eigen::Vector3d g(0.0, 0.0, -9.8);                // 重力加速度 (世界系)
        Eigen::Vector3d a_world = R * a_unbias + g;       // 世界系下的加速度

        // =================== 名义状态传播 ===================
        xHat_.p = xHat_.p + xHat_.v * dt + 0.5 * a_world * dt * dt;
        xHat_.v = xHat_.v + a_world * dt;
        Eigen::Vector3d delta_theta = omega_unbias * dt;
        Eigen::Quaterniond dq = Sophus::SO3d::exp(delta_theta).unit_quaternion();
        xHat_.q = xHat_.q * dq;
        xHat_.q.normalize();
        // 足端位置、偏置保持不变
        // =================== 误差状态协方差传播 ===================
        // 计算离散化的状态转移矩阵 A 和噪声雅可比 B
        // 误差状态布局: [δp, δv, δθ, δf1, δf2, δf3, δf4, δba, δbg]
        A = StateCov::Identity();
        A.block<3, 3>(ESKFNominalState::POS_IDX, ESKFNominalState::VEL_IDX) = Eigen::Matrix3d::Identity() * dt;         // ∂δp_dot/∂δv = I
        A.block<3, 3>(ESKFNominalState::VEL_IDX, ESKFNominalState::ROT_IDX) = -R * Sophus::SO3d::hat(a_unbias) * dt;    // ∂δv_dot/∂δθ = -R * [a_unbias]_x * dt
        A.block<3, 3>(ESKFNominalState::VEL_IDX, ESKFNominalState::BA_IDX) = -R * dt;                                   // ∂δv_dot/∂δba = -R * dt
        A.block<3, 3>(ESKFNominalState::ROT_IDX, ESKFNominalState::ROT_IDX) = Sophus::SO3d::exp(-delta_theta).matrix(); // ∂δθ_dot/∂δθ = I - [ω_unbias]_x * dt ≈ Exp(-ω * dt)
        A.block<3, 3>(ESKFNominalState::ROT_IDX, ESKFNominalState::BG_IDX) = -Eigen::Matrix3d::Identity() * dt;         // ∂δθ_dot/∂δbg = -I * dt

        // 噪声雅可比 B     噪声布局: [n_g(3), n_a(3), n_f1(3), n_f2(3), n_f3(3), n_f4(3), n_ba(3), n_bg(3)]
        B.setZero();
        B.block<3, 3>(ESKFNominalState::ROT_IDX, 0) = -Eigen::Matrix3d::Identity() * dt; // 陀螺仪噪声 -> 姿态误差
        B.block<3, 3>(ESKFNominalState::VEL_IDX, 3) = -R * dt;                           // 加速度噪声 -> 速度误差
        // 足端过程噪声 -> 足端位置误差
        B.block<3, 3>(ESKFNominalState::F1_IDX, 6) = R * dt;
        B.block<3, 3>(ESKFNominalState::F2_IDX, 9) = R * dt;
        B.block<3, 3>(ESKFNominalState::F3_IDX, 12) = R * dt;
        B.block<3, 3>(ESKFNominalState::F4_IDX, 15) = R * dt;
        // 偏置随机游走噪声
        B.block<3, 3>(ESKFNominalState::BA_IDX, 18) = Eigen::Matrix3d::Identity() * dt;
        B.block<3, 3>(ESKFNominalState::BG_IDX, 21) = Eigen::Matrix3d::Identity() * dt;

        // 协方差传播: P_k = A * P_{k-1} * A^T + B * Q * B^T
        P = A * P * A.transpose() + B * Q * B.transpose();

        // 偏置可观测性较弱，但不要把 P 的 ba/bg 行列整体清零——那等价于
        // 把交叉协方差也抹掉，会让姿态观测 (fuseImuAttitude / 足端运动学) 失去
        // 通过 A(ROT, BG) = -I*dt 反推 bg 的能力，长期 yaw / pitch 漂移得不到修正。
        // 这里只对 ba/bg 的方差做下限/上限，防止数值爆炸或过度自信。
        const double bias_var_min = 1e-8;
        const double bias_var_max = 1.0;
        auto clampDiag = [&](int idx)
        {
            for (int i = 0; i < 3; ++i)
            {
                double &v = P(idx + i, idx + i);
                if (!std::isfinite(v) || v < bias_var_min) v = bias_var_min;
                if (v > bias_var_max) v = bias_var_max;
            }
        };
        clampDiag(ESKFNominalState::BA_IDX);
        clampDiag(ESKFNominalState::BG_IDX);

        // 确保P对称正定
        P = 0.5 * (P + P.transpose());
    }

    void ErrorStateKalmanFilter::eskfUpdate(const MeasVec &y, const MeasJacobian &C, const MeasCov &R)
    {
        // 1. 计算卡尔曼增益
        MeasCov S = C * P * C.transpose() + R;

        // 使用 LDLT（对称）分解
        Eigen::LDLT<MeasCov> ldlt(S);
        Eigen::Matrix<double, ESKFNominalState::MEAS_DIM, ESKFNominalState::STATE_DIM> S_inv_C = ldlt.solve(C);
        Eigen::Matrix<double, ESKFNominalState::STATE_DIM, ESKFNominalState::MEAS_DIM> K = P * S_inv_C.transpose();

        // 2. 计算误差状态修正
        dx = K * y;

        // 3. 将误差状态注入名义状态
        injectErrorState(dx);

        // 4. 更新协方差 (约瑟夫形式保证矩阵正定性)
        StateCov I_KH = StateCov::Identity() - K * C;
        P = I_KH * P * I_KH.transpose() + K * R * K.transpose();

        // 5. 误差状态重置 (姿态部分)
        resetErrorState(dx);

        // 确保对称性
        P = 0.5 * (P + P.transpose());
    }

    void ErrorStateKalmanFilter::setCovarianceQ(ProcessNoiseCov &Q, double dt)
    {
        (void)dt;

        Q.setIdentity();
        // 过程噪声协方差 (连续时间噪声)
        // [ n_g(3), n_a(3), n_f1(3), n_f2(3), n_f3(3), n_f4(3), n_ba(3), n_bg(3) ]
        Q.block(0, 0, 3, 3) *= (proc_noise_gyro_ * proc_noise_gyro_);      // 角速度噪声（影响姿态积分）
        Q.block(3, 3, 3, 3) *= (proc_noise_acc_ * proc_noise_acc_);        // 加速度噪声（影响速度积分）
        Q.block(6, 6, 12, 12) *= (proc_noise_fpos_ * proc_noise_fpos_);    // 足端过程噪声
        const bool has_measured_contact = std::any_of(contact_flag_.begin(), contact_flag_.end(), [](bool contact) { return contact; });
        for (int i = 0; i < 4; ++i)
        {
            bool isContact = contact_flag_[i] || !has_measured_contact;
            Q.block(6 + 3 * i, 6 + 3 * i, 3, 3) = (isContact ? 1.0 : high_suspect_number) * Q.block(6 + 3 * i, 6 + 3 * i, 3, 3);
        }
        Q.block(18, 18, 3, 3) *= (proc_noise_ba_ * proc_noise_ba_); // 偏置加速度噪声
        Q.block(21, 21, 3, 3) *= (proc_noise_bg_ * proc_noise_bg_); // 偏置陀螺仪噪声
    }

    void ErrorStateKalmanFilter::measurementModelKinematics(const ESKFNominalState &x, const ESKFInput &in, MeasVec &y, MeasJacobian &C, MeasCov &R)
    {
        (void)in;

        // 初始化输出
        y.setZero();
        C.setZero();
        R.setIdentity();

        Eigen::Matrix3d Rw = x.q.toRotationMatrix(); // G: body->world
        Eigen::Matrix3d RwT = Rw.transpose();        // G^T: world->body

        Eigen::Matrix3d R_fpos = Eigen::Matrix3d::Identity() * (meas_noise_fpos_ * meas_noise_fpos_);
        Eigen::Matrix3d R_fvel = Eigen::Matrix3d::Identity() * (meas_noise_fvel_ * meas_noise_fvel_);
        double R_fheight = meas_noise_fheight_ * meas_noise_fheight_;
        Eigen::Matrix3d R_phi = Eigen::Matrix3d::Identity() * (meas_noise_encoder_ * meas_noise_encoder_);

        Eigen::Vector3d p_k = x.p; // body位置 (世界系)
        Eigen::Vector3d v_k = x.v; // body速度 (世界系)

        const bool has_measured_contact = std::any_of(contact_flag_.begin(), contact_flag_.end(), [](bool contact) { return contact; });
        const double missing_contact_noise_scale = has_measured_contact ? 1.0 : 10.0;

        for (int i = 0; i < 4; ++i)
        {
            // 若四条腿都未报接触，临时用弱运动学支撑约束，避免高度只靠IMU积分漂移。
            const bool isContact = contact_flag_[i] || !has_measured_contact;
            const int foot_state_idx = ESKFNominalState::F1_IDX + 3 * i;

            // ==================== 足端位置观测 ====================
            Eigen::Vector3d diff_world = x.f[i] - p_k;           // f - p (世界系)
            Eigen::Vector3d diff_body = RwT * diff_world;         // G^T(f - p) (体系)
            Eigen::Vector3d z_pos = RwT * foot_pos_[i];           // FK world-aligned 测量转到体系
            Eigen::Vector3d delta_ypos = z_pos - diff_body;       // 残差 (体系)
            Eigen::Matrix3d R_pos_i = R_fpos;

            if (isContact) {
                y.segment<3>(ESKFNominalState::MEAS_FPOS_IDX + 3 * i) = delta_ypos;
                C.block<3, 3>(ESKFNominalState::MEAS_FPOS_IDX + 3 * i, ESKFNominalState::POS_IDX) = -RwT;
                C.block<3, 3>(ESKFNominalState::MEAS_FPOS_IDX + 3 * i, ESKFNominalState::ROT_IDX) = Sophus::SO3d::hat(diff_body);
                C.block<3, 3>(ESKFNominalState::MEAS_FPOS_IDX + 3 * i, foot_state_idx) = RwT;

                // 编码器噪声传播
                Eigen::Matrix3d Jc_leg_body = RwT * Jc[i].middleCols<3>(6 + 3 * i);
                R_pos_i += Jc_leg_body * R_phi * Jc_leg_body.transpose();
                R_pos_i *= missing_contact_noise_scale;
            } else {
                y.segment<3>(ESKFNominalState::MEAS_FPOS_IDX + 3 * i) = Eigen::Vector3d::Zero();
                C.block<3, ESKFNominalState::STATE_DIM>(ESKFNominalState::MEAS_FPOS_IDX + 3 * i, 0).setZero();
                R_pos_i *= swing_inflate_;
            }

            // ==================== 足端速度观测 ====================
            Eigen::Matrix3d R_vel_i = R_fvel;
            if (isContact) {
                y.segment<3>(ESKFNominalState::MEAS_FVEL_IDX + 3 * i) = -(RwT * (v_k + foot_vel_[i]));
                C.block<3, 3>(ESKFNominalState::MEAS_FVEL_IDX + 3 * i, ESKFNominalState::VEL_IDX) = RwT;
                R_vel_i *= missing_contact_noise_scale;
            } else {
                y.segment<3>(ESKFNominalState::MEAS_FVEL_IDX + 3 * i).setZero();
                C.block<3, ESKFNominalState::STATE_DIM>(ESKFNominalState::MEAS_FVEL_IDX + 3 * i, 0).setZero();
                R_vel_i *= swing_inflate_;
            }

            // ==================== 足端高度观测 ====================
            double R_height_i = R_fheight;
            if (isContact) {
                const scalar_t terrain_height = 0.0;
                y(ESKFNominalState::MEAS_FHEIGHT_IDX + i) = static_cast<double>(terrain_height) + foot_radius_ - x.f[i].z();
                C(ESKFNominalState::MEAS_FHEIGHT_IDX + i, foot_state_idx + 2) = 1.0;
                R_height_i *= missing_contact_noise_scale;
            } else {
                y(ESKFNominalState::MEAS_FHEIGHT_IDX + i) = 0.0;
                C.row(ESKFNominalState::MEAS_FHEIGHT_IDX + i).setZero();
                R_height_i *= swing_inflate_;
            }

            // ==================== 观测噪声矩阵更新 ====================
            R.block<3, 3>(ESKFNominalState::MEAS_FPOS_IDX + 3 * i, ESKFNominalState::MEAS_FPOS_IDX + 3 * i) = R_pos_i;
            R.block<3, 3>(ESKFNominalState::MEAS_FVEL_IDX + 3 * i, ESKFNominalState::MEAS_FVEL_IDX + 3 * i) = R_vel_i;
            R(ESKFNominalState::MEAS_FHEIGHT_IDX + i, ESKFNominalState::MEAS_FHEIGHT_IDX + i) = R_height_i;
        }
    }

    void ErrorStateKalmanFilter::injectErrorState(const StateVec &dx)
    {
        // 位置
        xHat_.p += dx.segment<3>(ESKFNominalState::POS_IDX);

        // 速度
        xHat_.v += dx.segment<3>(ESKFNominalState::VEL_IDX);

        // 姿态 (使用李群更新: q ← q ⊗ Exp(δθ))
        Eigen::Vector3d delta_theta = dx.segment<3>(ESKFNominalState::ROT_IDX);
        Eigen::Quaterniond dq = Sophus::SO3d::exp(delta_theta).unit_quaternion();
        xHat_.q = xHat_.q * dq;
        xHat_.q.normalize();

        // 足端位置
        xHat_.f[0] += dx.segment<3>(ESKFNominalState::F1_IDX);
        xHat_.f[1] += dx.segment<3>(ESKFNominalState::F2_IDX);
        xHat_.f[2] += dx.segment<3>(ESKFNominalState::F3_IDX);
        xHat_.f[3] += dx.segment<3>(ESKFNominalState::F4_IDX);

        // 偏置
        xHat_.ba += dx.segment<3>(ESKFNominalState::BA_IDX);
        xHat_.bg += dx.segment<3>(ESKFNominalState::BG_IDX);

        const double ba_max = 0.05;
        const double bg_max = 0.05;
        xHat_.ba = xHat_.ba.cwiseMax(-ba_max).cwiseMin(ba_max);
        xHat_.bg = xHat_.bg.cwiseMax(-bg_max).cwiseMin(bg_max);
    }

    void ErrorStateKalmanFilter::resetErrorState(const StateVec &dx)
    {
        Eigen::Vector3d delta_theta = dx.segment<3>(ESKFNominalState::ROT_IDX);

        // 构造重置雅可比 (只有姿态部分需要修正)
        StateCov G_reset = StateCov::Identity();
        G_reset.block<3, 3>(ESKFNominalState::ROT_IDX, ESKFNominalState::ROT_IDX) = Eigen::Matrix3d::Identity() - 0.5 * Sophus::SO3d::hat(delta_theta);

        // 更新协方差
        P = G_reset * P * G_reset.transpose();
    }

    void ErrorStateKalmanFilter::fuseImuAttitude()
    {
        // ========================================================================
        // IMU 姿态软约束：以 IMU 自带的方向作为附加观测，柔性约束 ESKF 旋转。
        //
        // 与足端运动学观测保持一致，采用 body-frame 误差: q_true = q_eskf ⊗ Exp(δθ_body)
        //     y_att = Log(q_eskf^{-1} ⊗ q_imu)        (body frame)
        //     C_att = I (3x3) at ROT_IDX
        //
        // 噪声分离: roll/pitch 用 meas_noise_attitude_，yaw 用 meas_noise_attitude_yaw_。
        // 这两个值是 world-frame 下 zyx 上的标准差；为保持与 body-frame 残差一致，
        // 通过 R_att_body = R_wb^T * diag(rp, rp, yaw)^2 * R_wb 投影回 body frame。
        // ========================================================================

        // 1) body-frame 姿态残差
        Eigen::Quaterniond dq = xHat_.q.inverse() * quat_;
        dq.normalize();
        Eigen::Vector3d y_att = Sophus::SO3d(dq).log();

        // 2) 观测雅可比：仅作用在 δθ_body 上
        Eigen::Matrix<double, 3, ESKFNominalState::STATE_DIM> C_att;
        C_att.setZero();
        C_att.block<3, 3>(0, ESKFNominalState::ROT_IDX) = Eigen::Matrix3d::Identity();

        // 3) world-frame 上 roll/pitch/yaw 噪声 → body frame
        const double sigma_rp2 = meas_noise_attitude_ * meas_noise_attitude_;
        const double sigma_yaw2 = meas_noise_attitude_yaw_ * meas_noise_attitude_yaw_;
        Eigen::Matrix3d R_att_world = Eigen::Matrix3d::Zero();
        R_att_world(0, 0) = sigma_rp2;
        R_att_world(1, 1) = sigma_rp2;
        R_att_world(2, 2) = sigma_yaw2;
        const Eigen::Matrix3d R_wb = xHat_.q.toRotationMatrix();
        Eigen::Matrix3d R_att = R_wb.transpose() * R_att_world * R_wb;

        // 4) 标准卡尔曼更新（约束在姿态子块上，避免 yaw 大噪声把别的状态也轻易拉走）
        Eigen::Matrix3d S_att = C_att * P * C_att.transpose() + R_att;
        Eigen::Matrix<double, ESKFNominalState::STATE_DIM, 3> K_att =
            P * C_att.transpose() * S_att.ldlt().solve(Eigen::Matrix3d::Identity());
        StateVec dx_att = K_att * y_att;

        // 5) 单步姿态修正量限幅，避免大跳变
        const double max_att_correction = 0.05; // rad (~2.8°)
        for (int i = 0; i < 3; ++i)
        {
            dx_att(ESKFNominalState::ROT_IDX + i) = std::clamp(
                dx_att(ESKFNominalState::ROT_IDX + i), -max_att_correction, max_att_correction);
        }

        injectErrorState(dx_att);

        StateCov I_KC = StateCov::Identity() - K_att * C_att;
        P = I_KC * P * I_KC.transpose() + K_att * R_att * K_att.transpose();

        resetErrorState(dx_att);

        P = 0.5 * (P + P.transpose());
    }

    void ErrorStateKalmanFilter::footStateEstimation(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        double dt = period.seconds();
        Eigen::Matrix<double, 12, 1> joint_torques;
        for (int i = 0; i < 12; ++i)
        {
            joint_torques(i) = ctrl_component_.joint_effort_state_interface_[i].get().get_value();
        }
        contact_estimator_->setJointTorques(joint_torques); // 设置关节力矩
        contact_estimator_->setRbdState(rbd_state_);        // 设置机器人状态
        contact_estimator_->updateAndComputeForce(dt);      // 更新动力学参数并计算足端力

        // 获取足端高度(坐标系方向与world对齐，值是相较于base)
        const auto eePosRelWorld = ee_kinematics_->getPosition(vector_t());
        std::array<double, 4> foot_heights;
        const vector3_t p_wb = xHat_.p; // base world position (来自 KF 估计)
        for (int i = 0; i < 4; ++i)
        {
            const scalar_t terrainHeight =0.0;
            foot_heights[i] = static_cast<double>(p_wb.z() + eePosRelWorld[i].z() - terrainHeight);
        }

        // 从 MPC 的 ModeSchedule 获取接触相位和摆动相位
        std::array<double, 4> contact_phase = {0.5, 0.5, 0.5, 0.5};
        std::array<double, 4> swing_phase = {-1.0, -1.0, -1.0, -1.0};
        // 与 MPC/ReferenceManager 的时间基准保持一致（使用从 0 累加的控制时间，而非 ROS 时间戳）。
        const double current_time = controller_time_;
        const auto &modeSchedule = reference_manager_ptr_->getModeSchedule();
        const auto contactPhasePerLeg = getContactPhasePerLeg(current_time, modeSchedule);
        const auto swingPhasePerLeg = getSwingPhasePerLeg(current_time, modeSchedule);

        for (size_t i = 0; i < 4; ++i)
        {
            contact_phase[i] = contactPhasePerLeg[i].phase;
            swing_phase[i] = swingPhasePerLeg[i].phase;
        }       

        // 获取估计的足端力
        auto foot_forces = contact_estimator_->getFootForces();

        // 更新接触估计
        contact_estimator_->update(foot_forces, foot_heights, contact_phase, swing_phase);

        contact_prob_ = contact_estimator_->getContactProbabilities();

        // 获取接触标志并更新到状态估计
        auto contact_flags = contact_estimator_->getContactFlags();
        for (int i = 0; i < 4; ++i)
        {
            contact_flag_[i] = contact_flags[i];
        }
    }

    nav_msgs::msg::Odometry ErrorStateKalmanFilter::getOdomMsg()
    {
        nav_msgs::msg::Odometry odom;

        odom.pose.covariance.fill(0.0);
        odom.twist.covariance.fill(0.0);

        // ==================== IMU点 -> Body点 坐标转换 ====================
        // xHat_.p/v 是 IMU 点在世界系下的位置/速度
        // 需要转换到 body (base_link) 点
        // p_body = p_imu + R_world_body * t_imu_to_body
        // v_body = v_imu + omega_world × (R_world_body * t_imu_to_body)
        Eigen::Matrix3d R_wb = xHat_.q.toRotationMatrix();
        Eigen::Vector3d r_world = R_wb * imu_to_body_translation_; // 杆臂在世界系下的表示

        Eigen::Vector3d p_body = xHat_.p + r_world; // body在世界系下的位置

        // 角速度 (body系，去偏后)
        Eigen::Vector3d omega_body = angular_vel_local_ - xHat_.bg;
        Eigen::Vector3d omega_world = R_wb * omega_body;
        Eigen::Vector3d v_body = xHat_.v + omega_world.cross(r_world); // body在世界系下的速度

        odom.pose.pose.position.x = p_body.x();
        odom.pose.pose.position.y = p_body.y();
        odom.pose.pose.position.z = p_body.z();

        // 输出 ESKF 融合后的姿态，而不是裸 IMU 的 quat_
        const Eigen::Quaterniond q_out = xHat_.q.normalized();
        odom.pose.pose.orientation.x = q_out.x();
        odom.pose.pose.orientation.y = q_out.y();
        odom.pose.pose.orientation.z = q_out.z();
        odom.pose.pose.orientation.w = q_out.w();

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                odom.pose.covariance[6 * i + j] = P(ESKFNominalState::POS_IDX + i, ESKFNominalState::POS_IDX + j);
                odom.pose.covariance[6 * (3 + i) + (3 + j)] = P(ESKFNominalState::ROT_IDX + i, ESKFNominalState::ROT_IDX + j);
            }
        }

        odom.twist.twist.linear.x = v_body.x();
        odom.twist.twist.linear.y = v_body.y();
        odom.twist.twist.linear.z = v_body.z();

        odom.twist.twist.angular.x = omega_body.x();
        odom.twist.twist.angular.y = omega_body.y();
        odom.twist.twist.angular.z = omega_body.z();

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                odom.twist.covariance[6 * i + j] = P(ESKFNominalState::VEL_IDX + i, ESKFNominalState::VEL_IDX + j);
                odom.twist.covariance[6 * (3 + i) + (3 + j)] = P(ESKFNominalState::BG_IDX + i, ESKFNominalState::BG_IDX + j);
            }
        }
        return odom;
    }

    void ErrorStateKalmanFilter::loadSettings(const std::string &task_file, const bool verbose)
    {
        boost::property_tree::ptree pt;
        read_info(task_file, pt);
        const std::string prefix = "ErrorStateKalmanFilter.";
        if (verbose)
        {
            std::cerr << "\n #### ErrorStateKalmanFilter Noise:";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, is_logger_enabled_, prefix + "is_enabled_logger", verbose);
        loadData::loadPtreeValue(pt, sim_mode_, prefix + "sim_mode", verbose);

        loadData::loadPtreeValue(pt, swing_inflate_, prefix + "SwingInflate", verbose);

        loadData::loadPtreeValue(pt, proc_noise_acc_, prefix + "ProcessNoiseAcceleration", verbose);
        loadData::loadPtreeValue(pt, proc_noise_gyro_, prefix + "ProcessNoiseGyroscope", verbose);
        loadData::loadPtreeValue(pt, proc_noise_fpos_, prefix + "ProcessNoiseFootPosition", verbose);
        loadData::loadPtreeValue(pt, proc_noise_ba_, prefix + "ProcessNoiseBiasAcceleration", verbose);
        loadData::loadPtreeValue(pt, proc_noise_bg_, prefix + "ProcessNoiseBiasGyroscope", verbose);

        loadData::loadPtreeValue(pt, meas_noise_fvel_, prefix + "MeasurementNoiseFootVelocity", verbose);
        loadData::loadPtreeValue(pt, meas_noise_fpos_, prefix + "MeasurementNoiseFootPosition", verbose);
        loadData::loadPtreeValue(pt, meas_noise_fheight_, prefix + "MeasurementNoiseFootHeight", verbose);
        loadData::loadPtreeValue(pt, meas_noise_ba_, prefix + "MeasurementNoiseBiasAcceleration", verbose);
        loadData::loadPtreeValue(pt, meas_noise_bg_, prefix + "MeasurementNoiseBiasGyroscope", verbose);
        loadData::loadPtreeValue(pt, meas_noise_encoder_, prefix + "MeasurementNoiseEncoder", verbose);
        loadData::loadPtreeValue(pt, meas_noise_attitude_, prefix + "MeasurementNoiseAttitude", verbose);
        loadData::loadPtreeValue(pt, meas_noise_attitude_yaw_, prefix + "MeasurementNoiseAttitudeYaw", verbose);
        loadData::loadPtreeValue(pt, foot_radius_, prefix + "FootRadius", verbose);
        loadData::loadPtreeValue(pt, min_contact_trust_, prefix + "MinContactTrust", verbose);

        if (use_contact_estimator_ && contact_estimator_)
        {
            contact_estimator_->loadSettings(task_file, verbose);
        }
    }

    void ErrorStateKalmanFilter::updateGroundTruth()
    {
        const size_t num_interfaces = ctrl_component_.odom_state_interface_.size();

        // Position (indices 0-2)
        gt_position_(0) = ctrl_component_.odom_state_interface_[0].get().get_value();
        gt_position_(1) = ctrl_component_.odom_state_interface_[1].get().get_value();
        gt_position_(2) = ctrl_component_.odom_state_interface_[2].get().get_value();

        // Linear velocity (indices 3-5)
        gt_linear_velocity_(0) = ctrl_component_.odom_state_interface_[3].get().get_value();
        gt_linear_velocity_(1) = ctrl_component_.odom_state_interface_[4].get().get_value();
        gt_linear_velocity_(2) = ctrl_component_.odom_state_interface_[5].get().get_value();

        // Orientation quaternion (indices 6-9)
        if (num_interfaces > 9)
        {
            gt_orientation_.x() = ctrl_component_.odom_state_interface_[6].get().get_value();
            gt_orientation_.y() = ctrl_component_.odom_state_interface_[7].get().get_value();
            gt_orientation_.z() = ctrl_component_.odom_state_interface_[8].get().get_value();
            gt_orientation_.w() = ctrl_component_.odom_state_interface_[9].get().get_value();
            gt_orientation_.normalize();
        }

        // Angular velocity (indices 10-12)
        if (num_interfaces > 12)
        {
            gt_angular_velocity_(0) = ctrl_component_.odom_state_interface_[10].get().get_value();
            gt_angular_velocity_(1) = ctrl_component_.odom_state_interface_[11].get().get_value();
            gt_angular_velocity_(2) = ctrl_component_.odom_state_interface_[12].get().get_value();
        }
    }

    void ErrorStateKalmanFilter::publishGroundTruth(const rclcpp::Time &time)
    {

        nav_msgs::msg::Odometry gt_msg;
        gt_msg.header.stamp = time;
        gt_msg.header.frame_id = "odom";
        gt_msg.child_frame_id = "base";

        // Position (from Gazebo WorldPose)
        gt_msg.pose.pose.position.x = gt_position_(0);
        gt_msg.pose.pose.position.y = gt_position_(1);
        gt_msg.pose.pose.position.z = gt_position_(2);

        // Orientation (from Gazebo WorldPose)
        gt_msg.pose.pose.orientation.x = gt_orientation_.x();
        gt_msg.pose.pose.orientation.y = gt_orientation_.y();
        gt_msg.pose.pose.orientation.z = gt_orientation_.z();
        gt_msg.pose.pose.orientation.w = gt_orientation_.w();

        // Linear velocity (from Gazebo WorldLinearVelocity)
        gt_msg.twist.twist.linear.x = gt_linear_velocity_(0);
        gt_msg.twist.twist.linear.y = gt_linear_velocity_(1);
        gt_msg.twist.twist.linear.z = gt_linear_velocity_(2);

        // Angular velocity (from Gazebo AngularVelocity)
        gt_msg.twist.twist.angular.x = gt_angular_velocity_(0);
        gt_msg.twist.twist.angular.y = gt_angular_velocity_(1);
        gt_msg.twist.twist.angular.z = gt_angular_velocity_(2);

        ground_truth_pub_->publish(gt_msg);
    }
}
