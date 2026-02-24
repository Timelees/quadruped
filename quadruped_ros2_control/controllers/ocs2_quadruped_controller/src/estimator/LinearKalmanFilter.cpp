//
// Created by qiayuan on 2022/7/24.
//

#include <map>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <utility>
#include <sstream>

#include <ocs2_quadruped_controller/estimator/LinearKalmanFilter.h>
#include <ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h>
#include <ocs2_core/misc/LoadData.h>

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2::legged_robot
{
    KalmanFilterEstimate::KalmanFilterEstimate(PinocchioInterface pinocchio_interface,
                                               CentroidalModelInfo info,
                                               const PinocchioEndEffectorKinematics &ee_kinematics,
                                               CtrlInterfaces &ctrl_component,
                                               const rclcpp_lifecycle::LifecycleNode::SharedPtr &node)
        : StateEstimateBase(std::move(info), ctrl_component,
                            node),
          pinocchio_interface_(std::move(pinocchio_interface)),
          ee_kinematics_(ee_kinematics.clone()),
          numContacts_(info_.numThreeDofContacts + info_.numSixDofContacts),
          dimContacts_(3 * numContacts_),
          numState_(6 + dimContacts_),
          numObserve_(2 * dimContacts_ + numContacts_)
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

        xHat_.setZero(numState_);
        ps_.setZero(dimContacts_);
        vs_.setZero(dimContacts_);
        a_.setIdentity(numState_, numState_);
        b_.setZero(numState_, 3);

        matrix_t c1(3, 6), c2(3, 6);
        c1 << matrix3_t::Identity(), matrix3_t::Zero();
        c2 << matrix3_t::Zero(), matrix3_t::Identity();
        c_.setZero(numObserve_, numState_);
        for (ssize_t i = 0; i < numContacts_; ++i)
        {
            c_.block(3 * i, 0, 3, 6) = c1;
            c_.block(3 * (numContacts_ + i), 0, 3, 6) = c2;
            c_(2 * dimContacts_ + i, 6 + 3 * i + 2) = 1.0;
        }
        c_.block(0, 6, dimContacts_, dimContacts_) = -matrix_t::Identity(dimContacts_, dimContacts_);

        q_.setIdentity(numState_, numState_);
        p_ = 1. * q_;
        r_.setIdentity(numObserve_, numObserve_);
        feet_heights_.setZero(numContacts_);

        ee_kinematics_->setPinocchioInterface(pinocchio_interface_);
        initPublishers();

        // 初始化 Ground Truth 发布器 (用于与状态估计对比)
        if (sim_mode_)
        {
            ground_truth_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("Ground_Truth", 10);
            ground_truth_enabled_ = true; // 启用 Ground Truth 对比
        }

        // 初始化接触估计器
        
        // 创建 ContactEstimator 需要单独的 PinocchioInterface 副本
        PinocchioInterface pinocchio_interface_copy(pinocchio_interface_);
        contact_estimator_ = std::make_unique<ContactEstimator>(
                std::move(pinocchio_interface_copy),
                info_,
                ee_kinematics,
                ctrl_component,
                node);
        RCLCPP_INFO(node_->get_logger(), "ContactEstimator initialized successfully.");
        
    }

    vector_t KalmanFilterEstimate::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        scalar_t dt = period.seconds(); // 控制器时间周期

        updateJointStates();        // 更新关节状态
        computeForwardKinematics(); // 计算前向运动学

        // 使用 ContactEstimator 估计足端接触状态和足端力
        if (use_contact_estimator_ && contact_estimator_)
        {
            // 设置关节力矩
            Eigen::Matrix<double, 12, 1> joint_torques;
            for (int i = 0; i < 12; ++i)
            {
                joint_torques(i) = ctrl_component_.joint_effort_state_interface_[i].get().get_value();
            }
            // std::cout << "[KalmanFilter] joint_torque: " << joint_torques.transpose() << std::endl;
            contact_estimator_->setJointTorques(joint_torques);

            // 设置机器人状态
            contact_estimator_->setRbdState(rbd_state_);

            // 更新动力学参数并计算足端力
            contact_estimator_->updateAndComputeForce(dt);

            // 获取足端高度
            // 注意：ee_kinematics_->getPosition() 返回的是 base 坐标系下的足端位置。
            // ContactEstimator 的 P_h 期望输入“足端离地高度”(world z - ground_z)。
            // 这里在平地假设 ground_z = 0 的条件下，将 base->foot 变换到 world，取 foot_world.z 作为 height。
            const auto eePosBase = ee_kinematics_->getPosition(vector_t());
            std::array<double, 4> foot_heights;

            // base 在 world 中的姿态（用当前估计的 quat_ 更一致；如果你更希望用 rbd_state_ 的姿态，也可以替换）
            const matrix3_t R_wb = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_));

            // base 在 world 中的位置
            const vector3_t p_wb = xHat_.segment<3>(0);

            constexpr scalar_t ground_z = 0.0; // 平地假设；后续可接 elevation map / raycast
            for (int i = 0; i < 4; ++i)
            {
                const vector3_t p_bf = eePosBase[i];
                const vector3_t p_wf = p_wb + R_wb * p_bf;
                foot_heights[i] = static_cast<double>(p_wf.z() - ground_z);
            }

            // 从 MPC 的 ModeSchedule 获取接触相位和摆动相位
            std::array<double, 4> contact_phase;
            std::array<double, 4> swing_phase;
            const double current_time = time.seconds();
            getGaitPhaseFromMPC(current_time, contact_phase, swing_phase);

            // 从 MPC 获取期望接触状态并设置到 ContactEstimator
            contact_flag_t expected_contact;
            expected_contact.fill(false);
            if (reference_manager_ptr_)
            {
                try
                {
                    const auto &modeSchedule = reference_manager_ptr_->getModeSchedule();
                    const auto mode = modeSchedule.modeAtTime(current_time);
                    expected_contact = modeNumber2StanceLeg(mode);
                }
                catch (const std::exception &e)
                {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                                         "Failed to get expected contact from ModeSchedule: %s", e.what());
                }
            }
            contact_estimator_->setExpectedContact(expected_contact);

            // std::cout << "[Linear KalmanFilter] contact_phase: [";
            // for (size_t i = 0; i < contact_phase.size(); ++i) {
            //     if (i) std::cout << ", ";
            //     std::cout << contact_phase[i];
            // }
            // std::cout << "]" << std::endl;
            // std::cout << "[Linear KalmanFilter] swing_phase: [";
            // for (size_t i = 0; i < swing_phase.size(); ++i) {
            //     if (i) std::cout << ", ";
            //     std::cout << swing_phase[i];
            // }
            // std::cout << "]" << std::endl;

            // 获取估计的足端力
            auto foot_forces = contact_estimator_->getFootForces();

            // std::cout << "[Linear KalmanFilter] estimate foot_forces: [";
            // for (size_t i = 0; i < foot_forces.size(); ++i) {
            //     if (i) std::cout << ", ";
            //     std::cout << foot_forces[i];
            // }
            // std::cout << "]" << std::endl;

            // 更新接触估计
            contact_estimator_->update(foot_forces, foot_heights, contact_phase, swing_phase);

            // 获取接触标志并更新到状态估计
            auto contact_flags = contact_estimator_->getContactFlags();
            for (int i = 0; i < 4; ++i)
            {
                contact_flag_[i] = contact_flags[i];
            }
        }
        else
        {   
            // 设置关节力矩
            Eigen::Matrix<double, 12, 1> joint_torques;
            for (int i = 0; i < 12; ++i)
            {
                joint_torques(i) = ctrl_component_.joint_effort_state_interface_[i].get().get_value();
            }
            // std::cout << "[KalmanFilter] joint_torque: " << joint_torques.transpose() << std::endl;
            contact_estimator_->setJointTorques(joint_torques);

            // 设置机器人状态
            contact_estimator_->setRbdState(rbd_state_);

            // 更新动力学参数并计算足端力
            contact_estimator_->updateAndComputeForce(dt);

            // 获取足端力
            foot_forces = contact_estimator_->getFootForces();
            
            updateContact(); // 使用原有的接触更新逻辑
        }
        updateImu(); // 获取 IMU 数据

        a_.block(0, 3, 3, 3) = dt * matrix3_t::Identity();
        b_.block(0, 0, 3, 3) = 0.5 * dt * dt * matrix3_t::Identity();
        b_.block(3, 0, 3, 3) = dt * matrix3_t::Identity();
        q_.block(0, 0, 3, 3) = (dt / 20.f) * matrix3_t::Identity();
        q_.block(3, 3, 3, 3) = (dt * 9.81f / 20.f) * matrix3_t::Identity();
        q_.block(6, 6, dimContacts_, dimContacts_) = dt * matrix_t::Identity(dimContacts_, dimContacts_);

        // eePos和eeVel的值为负，使用时要取反才能正确表示足端位置和速度
        const auto eePos = ee_kinematics_->getPosition(vector_t());
        const auto eeVel = ee_kinematics_->getVelocity(vector_t(), vector_t());

        matrix_t q = matrix_t::Identity(numState_, numState_);
        q.block(0, 0, 3, 3) = q_.block(0, 0, 3, 3) * imu_process_noise_position_;
        q.block(3, 3, 3, 3) = q_.block(3, 3, 3, 3) * imu_process_noise_velocity_;
        q.block(6, 6, dimContacts_, dimContacts_) =
            q_.block(6, 6, dimContacts_, dimContacts_) * footProcessNoisePosition_;

        matrix_t r = matrix_t::Identity(numObserve_, numObserve_);
        r.block(0, 0, dimContacts_, dimContacts_) =
            r_.block(0, 0, dimContacts_, dimContacts_) * footSensorNoisePosition_;
        r.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) =
            r_.block(dimContacts_, dimContacts_, dimContacts_, dimContacts_) * footSensorNoiseVelocity_;
        r.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_) =
            r_.block(2 * dimContacts_, 2 * dimContacts_, numContacts_, numContacts_) * footHeightSensorNoise_;

        for (int i = 0; i < numContacts_; i++)
        {
            int i1 = 3 * i;

            int qIndex = 6 + i1;
            int rIndex1 = i1;
            int rIndex2 = dimContacts_ + i1;
            int rIndex3 = 2 * dimContacts_ + i;
            bool isContact = contact_flag_[i];

            scalar_t high_suspect_number(100);
            q.block(qIndex, qIndex, 3, 3) = (isContact ? 1. : high_suspect_number) * q.block(qIndex, qIndex, 3, 3);
            r.block(rIndex1, rIndex1, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex1, rIndex1, 3, 3);
            r.block(rIndex2, rIndex2, 3, 3) = (isContact ? 1. : high_suspect_number) * r.block(rIndex2, rIndex2, 3, 3);
            r(rIndex3, rIndex3) = (isContact ? 1. : high_suspect_number) * r(rIndex3, rIndex3);

            ps_.segment(3 * i, 3) = -eePos[i];        // foot相较于base在下方，需要取反
            ps_.segment(3 * i, 3)[2] += foot_radius_; // body质心高度
            vs_.segment(3 * i, 3) = -eeVel[i];
        }

        vector3_t g(0, 0, -9.81);
        vector3_t accel = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)) * linear_accel_local_ + g;

        vector_t y(numObserve_);
        y << ps_, vs_, feet_heights_;
        xHat_ = a_ * xHat_ + b_ * accel;
        matrix_t at = a_.transpose();
        matrix_t pm = a_ * p_ * at + q;
        matrix_t cT = c_.transpose();
        matrix_t yModel = c_ * xHat_;
        matrix_t ey = y - yModel;
        matrix_t s = c_ * pm * cT + r;

        vector_t sEy = s.lu().solve(ey);
        xHat_ += pm * cT * sEy;

        matrix_t sC = s.lu().solve(c_);
        p_ = (matrix_t::Identity(numState_, numState_) - pm * cT * sC) * pm;

        matrix_t pt = p_.transpose();
        p_ = (p_ + pt) / 2.0;

        //  if (p_.block(0, 0, 2, 2).determinant() > 0.000001) {
        //    p_.block(0, 2, 2, 16).setZero();
        //    p_.block(2, 0, 16, 2).setZero();
        //    p_.block(0, 0, 2, 2) /= 10.;
        //  }

        if (sim_mode_)
        {
            // 更新 Ground Truth 数据 (在日志记录之前更新，确保数据有效)
            updateGroundTruth();
        }

        // 日志记录
        static int log_counter = 0;
        log_counter++;
        if (is_logger_enabled_ && (log_counter % 10 == 0))
        {
            // 状态估计值
            Eigen::Vector3d state_zyx = quatToZyx(quat_);    // 姿态
            Eigen::Vector3d state_pos = xHat_.segment<3>(0); // 位置估计值
            Eigen::Vector3d state_vel = xHat_.segment<3>(3); // 速度估计值

            logger_->saveDataToCSV("state_pos.csv", "pos_x, pos_y, pos_z", state_pos);
            logger_->saveDataToCSV("state_vel.csv", "vel_x, vel_y, vel_z", state_vel);
            logger_->saveDataToCSV("state_zyx.csv", "yaw, pitch, roll", state_zyx);

            matrix_t p_log = p_;
            Eigen::Vector3d P_diag_pos(p_log(0, 0), p_log(1, 1), p_log(2, 2));
            Eigen::Vector3d P_diag_vel(p_log(3, 3), p_log(4, 4), p_log(5, 5));
            Eigen::Vector3d P_diag_foot1(p_log(6, 6), p_log(7, 7), p_log(8, 8));
            Eigen::Vector3d P_diag_foot2(p_log(9, 9), p_log(10, 10), p_log(11, 11));
            Eigen::Vector3d P_diag_foot3(p_log(12, 12), p_log(13, 13), p_log(14, 14));
            Eigen::Vector3d P_diag_foot4(p_log(15, 15), p_log(16, 16), p_log(17, 17));
            logger_->saveDataToCSV("P_diag_pos.csv", "P_diag_pos_x, P_diag_pos_y, P_diag_pos_z", P_diag_pos);
            logger_->saveDataToCSV("P_diag_vel.csv", "P_diag_vel_x, P_diag_vel_y, P_diag_vel_z", P_diag_vel);
            logger_->saveDataToCSV("P_diag_foot1.csv", "P_diag_foot1_x, P_diag_foot1_y, P_diag_foot1_z", P_diag_foot1);
            logger_->saveDataToCSV("P_diag_foot2.csv", "P_diag_foot2_x, P_diag_foot2_y, P_diag_foot2_z", P_diag_foot2);
            logger_->saveDataToCSV("P_diag_foot3.csv", "P_diag_foot3_x, P_diag_foot3_y, P_diag_foot3_z", P_diag_foot3);
            logger_->saveDataToCSV("P_diag_foot4.csv", "P_diag_foot4_x, P_diag_foot4_y, P_diag_foot4_z", P_diag_foot4);

            // 观测残差 (ey = y - yModel)
            vector_t ey_log = ey;
            for (int i = 0; i < 4; i++)
            {
                // 足端位置观测残差 (body系)
                Eigen::Vector3d foot_pos_residual = ey_log.segment<3>(3 * i);
                logger_->saveDataToCSV("y_pos_residual_foot_" + std::to_string(i) + ".csv", "ey_x, ey_y, ey_z", foot_pos_residual);

                // 足端速度观测残差 (body系)
                Eigen::Vector3d foot_vel_residual = ey_log.segment<3>(12 + 3 * i);
                logger_->saveDataToCSV("y_vel_residual_foot_" + std::to_string(i) + ".csv", "eyv_x, eyv_y, eyv_z", foot_vel_residual);
            }
            Eigen::Vector4d foot_height_residual;
            for (int i = 0; i < 4; i++)
            {
                foot_height_residual(i) = ey_log(24 + i);
            }
            logger_->saveDataToCSV("y_height02_residual.csv", "eyh_0, eyh_2, unused",
                                   Eigen::Vector3d(foot_height_residual(0), foot_height_residual(2), 0.0));
            logger_->saveDataToCSV("y_height13_residual.csv", "eyh_1, eyh_3, unused",
                                   Eigen::Vector3d(foot_height_residual(1), foot_height_residual(3), 0.0));

            // ==================== Ground Truth 对比日志 ====================
            if (ground_truth_enabled_ && sim_mode_)
            {
                // Ground Truth 姿态 (欧拉角 ZYX)
                Eigen::Vector3d gt_rot_zyx = quatToZyx(gt_orientation_);
                logger_->saveDataToCSV("gazeboTrue_rot_zyx.csv", "gt_yaw, gt_pitch, gt_roll", gt_rot_zyx);
                logger_->saveDataToCSV("gazeboTrue_pos.csv", "gt_pos_x, gt_pos_y, gt_pos_z", gt_position_);
                logger_->saveDataToCSV("gazeboTrue_vel.csv", "gt_vel_x, gt_vel_y, gt_vel_z", gt_linear_velocity_);
                logger_->saveDataToCSV("gazeboTrue_angvel.csv", "gt_wx, gt_wy, gt_wz", gt_angular_velocity_);
                // 位置误差 (估计值 - 真值)
                Eigen::Vector3d pos_error = state_pos - gt_position_;
                logger_->saveDataToCSV("posEstiTruthError.csv", "err_x, err_y, err_z", pos_error);
                // 速度误差 (估计值 - 真值)
                Eigen::Vector3d vel_error = state_vel - gt_linear_velocity_;
                logger_->saveDataToCSV("velEstiTruthError.csv", "err_vx, err_vy, err_vz", vel_error);
                // 姿态误差 (估计值 - 真值)
                Eigen::Vector3d rot_error = state_zyx - gt_rot_zyx;
                logger_->saveDataToCSV("rotEstiTruthError.csv", "err_yaw, err_pitch, err_roll", rot_error);
                // 角速度误差 (估计值 - 真值)
                Eigen::Vector3d angvel_error = angular_vel_local_ - gt_angular_velocity_;
                logger_->saveDataToCSV("angvelEstiTruthError.csv", "err_wx, err_wy, err_wz", angvel_error);
            }

            // ==================== 接触状态对比日志 ====================
            if (use_contact_estimator_ && contact_estimator_)
            {
                // 获取估计的接触标志和期望的接触标志
                auto estimated_contact = contact_estimator_->getContactFlags();
                auto expected_contact = contact_estimator_->getExpectedContact();
                auto contact_probs = contact_estimator_->getContactProbabilities();
                auto debug_info = contact_estimator_->getDebugInfo();

                // 记录估计的接触状态 (0=摆动, 1=接触)
                Eigen::Vector4d est_contact_vec;
                for (int i = 0; i < 4; ++i)
                {
                    est_contact_vec(i) = estimated_contact[i] ? 1.0 : 0.0;
                }
                logger_->saveDataToCSV(
                    "contact_estimated.csv",
                    "est_FL, est_FR, est_RL, est_RR",
                    {est_contact_vec(0), est_contact_vec(1), est_contact_vec(2), est_contact_vec(3)});

                // 记录期望的接触状态 (来自 MPC)
                Eigen::Vector4d exp_contact_vec;
                for (int i = 0; i < 4; ++i)
                {
                    exp_contact_vec(i) = expected_contact[i] ? 1.0 : 0.0;
                }
                logger_->saveDataToCSV(
                    "contact_expected.csv",
                    "exp_FL, exp_FR, exp_RL, exp_RR",
                    {exp_contact_vec(0), exp_contact_vec(1), exp_contact_vec(2), exp_contact_vec(3)});

                // 记录接触状态误差 (估计值 - 期望值)
                Eigen::Vector4d contact_error;
                for (int i = 0; i < 4; ++i)
                {
                    contact_error(i) = est_contact_vec(i) - exp_contact_vec(i);
                }
                logger_->saveDataToCSV(
                    "contact_error.csv",
                    "err_FL, err_FR, err_RL, err_RR, mismatch_count",
                    {contact_error(0), contact_error(1), contact_error(2), contact_error(3),
                     static_cast<double>(contact_estimator_->getContactMismatchCount())});

                // 记录接触概率 (卡尔曼滤波输出)
                logger_->saveDataToCSV(
                    "contact_prob.csv",
                    "prob_FL, prob_FR, prob_RL, prob_RR",
                    {contact_probs[0], contact_probs[1], contact_probs[2], contact_probs[3]});

                // 记录三个观测概率 P_gait, P_height, P_force (用于调试)
                logger_->saveDataToCSV(
                    "P_gait.csv",
                    "P_gait_FL, P_gait_FR, P_gait_RL, P_gait_RR",
                    {debug_info.P_gait[0], debug_info.P_gait[1], debug_info.P_gait[2], debug_info.P_gait[3]});
                logger_->saveDataToCSV(
                    "P_height.csv",
                    "P_h_FL, P_h_FR, P_h_RL, P_h_RR",
                    {debug_info.P_position[0], debug_info.P_position[1], debug_info.P_position[2], debug_info.P_position[3]});
                logger_->saveDataToCSV(
                    "P_force.csv",
                    "P_f_FL, P_f_FR, P_f_RL, P_f_RR",
                    {debug_info.P_force[0], debug_info.P_force[1], debug_info.P_force[2], debug_info.P_force[3]});
            }
        }

        updateLinear(xHat_.segment<3>(0), xHat_.segment<3>(3));

        auto odom = getOdomMsg();
        odom.header.stamp = time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base";
        publishMsgs(odom);

        return rbd_state_;
    }

    void KalmanFilterEstimate::computeForwardKinematics()
    {
        const auto &model = pinocchio_interface_.getModel();
        auto &data = pinocchio_interface_.getData();
        size_t actuatedDofNum = info_.actuatedDofNum;

        vector_t qPino(info_.generalizedCoordinatesNum);
        vector_t vPino(info_.generalizedCoordinatesNum);
        qPino.setZero();
        qPino.segment<3>(3) = rbd_state_.head<3>(); // Only set orientation, let position in origin.
        qPino.tail(actuatedDofNum) = rbd_state_.segment(6, actuatedDofNum);

        vPino.setZero();
        vPino.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            qPino.segment<3>(3),
            rbd_state_.segment<3>(info_.generalizedCoordinatesNum));
        // Only set angular velocity, let linear velocity be zero
        vPino.tail(actuatedDofNum) = rbd_state_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum);

        forwardKinematics(model, data, qPino, vPino);
        updateFramePlacements(model, data);
    }

    void KalmanFilterEstimate::updateGroundTruth()
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

    void KalmanFilterEstimate::publishGroundTruth(const rclcpp::Time &time)
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

    nav_msgs::msg::Odometry KalmanFilterEstimate::getOdomMsg()
    {
        nav_msgs::msg::Odometry odom;
        odom.pose.pose.position.x = xHat_.segment<3>(0)(0);
        odom.pose.pose.position.y = xHat_.segment<3>(0)(1);
        odom.pose.pose.position.z = xHat_.segment<3>(0)(2);
        odom.pose.pose.orientation.x = quat_.x();
        odom.pose.pose.orientation.y = quat_.y();
        odom.pose.pose.orientation.z = quat_.z();
        odom.pose.pose.orientation.w = quat_.w();
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                odom.pose.covariance[i * 6 + j] = p_(i, j);
                odom.pose.covariance[6 * (3 + i) + (3 + j)] = orientationCovariance_(i * 3 + j);
            }
        }
        //  The twist in this message should be specified in the coordinate frame given by the child_frame_id: "base"
        vector_t twist = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)).transpose() * xHat_.segment<3>(3);
        odom.twist.twist.linear.x = twist.x();
        odom.twist.twist.linear.y = twist.y();
        odom.twist.twist.linear.z = twist.z();
        odom.twist.twist.angular.x = angular_vel_local_.x();
        odom.twist.twist.angular.y = angular_vel_local_.y();
        odom.twist.twist.angular.z = angular_vel_local_.z();
        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                odom.twist.covariance[i * 6 + j] = p_.block<3, 3>(3, 3)(i, j);
                odom.twist.covariance[6 * (3 + i) + (3 + j)] = angularVelCovariance_(i * 3 + j);
            }
        }
        return odom;
    }

    void KalmanFilterEstimate::loadSettings(const std::string &task_file, const bool verbose)
    {
        boost::property_tree::ptree pt;
        read_info(task_file, pt);
        const std::string prefix = "kalmanFilter.";
        if (verbose)
        {
            std::cerr << "\n #### Kalman Filter Noise:";
            std::cerr << "\n #### =============================================================================\n";
        }

        loadData::loadPtreeValue(pt, foot_radius_, prefix + "footRadius", verbose);
        loadData::loadPtreeValue(pt, sim_mode_, prefix + "sim_mode", verbose);
        loadData::loadPtreeValue(pt, imu_process_noise_position_, prefix + "imuProcessNoisePosition", verbose);
        loadData::loadPtreeValue(pt, imu_process_noise_velocity_, prefix + "imuProcessNoiseVelocity", verbose);
        loadData::loadPtreeValue(pt, footProcessNoisePosition_, prefix + "footProcessNoisePosition", verbose);
        loadData::loadPtreeValue(pt, footSensorNoisePosition_, prefix + "footSensorNoisePosition", verbose);
        loadData::loadPtreeValue(pt, footSensorNoiseVelocity_, prefix + "footSensorNoiseVelocity", verbose);
        loadData::loadPtreeValue(pt, footHeightSensorNoise_, prefix + "footHeightSensorNoise", verbose);
        loadData::loadPtreeValue(pt, high_suspect_number_, prefix + "highSuspectNumber", verbose);

        // 同步加载 ContactEstimator 相关参数（contactEstimator.*）
        if (use_contact_estimator_ && contact_estimator_)
        {
            contact_estimator_->loadSettings(task_file, verbose);
        }
    }

    void KalmanFilterEstimate::getGaitPhaseFromMPC(scalar_t current_time,
                                                   std::array<scalar_t, 4> &contact_phase,
                                                   std::array<scalar_t, 4> &swing_phase)
    {
        // 默认值：假设所有腿在支撑相中间
        // 使用 -1.0 表示"不在该相位"
        contact_phase.fill(0.5); // 支撑相中间值
        swing_phase.fill(-1.0);  // -1.0 表示不在摆动相

        // 如果没有设置 ReferenceManager，使用默认值
        if (!reference_manager_ptr_)
        {
            return;
        }

        // 从 MPC 获取 ModeSchedule
        const auto &modeSchedule = reference_manager_ptr_->getModeSchedule();

        // 获取每条腿的接触相位
        const auto contactPhasePerLeg = getContactPhasePerLeg(current_time, modeSchedule);

        // 获取每条腿的摆动相位
        const auto swingPhasePerLeg = getSwingPhasePerLeg(current_time, modeSchedule);

        // 转换为输出格式
        for (size_t i = 0; i < 4; ++i)
        {
            // 直接传递原始值，保留 -1.0 语义
            contact_phase[i] = contactPhasePerLeg[i].phase;
            swing_phase[i] = swingPhasePerLeg[i].phase;
        }
    }

} // namespace legged