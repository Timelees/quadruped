#include <pinocchio/fwd.hpp>

#include "ocs2_quadruped_controller/estimator/ErrorStateKalmanFilter.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <utility>
#include <chrono>
#include <thread>
#include <limits>
#include <algorithm>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

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

        // 初始化腿部运动学模型
        // leg_kinematics_ = std::make_unique<QuadrupedRobotModel>();

        // 初始化滤波器状态
        init();

        // 初始化话题发布
        initPublishers();
    }

    ErrorStateKalmanFilter::~ErrorStateKalmanFilter()
    {
    }

    void ErrorStateKalmanFilter::init()
    {
        // initialize orientation from IMU if available; otherwise identity.
        rotation_matrix_.setIdentity();
        if (std::isfinite(quat_.w()) && std::isfinite(quat_.x()) && std::isfinite(quat_.y()) && std::isfinite(quat_.z()))
        {
            rotation_matrix_ = quat_.toRotationMatrix();
        }

        // 初始化名义状态
        constexpr double initial_body_height = 0.0562; // 初始高度0.05
        constexpr double foot_radius = 0.0207;
        xHat_.p = Eigen::Vector3d(0.0, 0.0, initial_body_height);
        xHat_.v = Eigen::Vector3d::Zero();
        xHat_.q = Eigen::Quaterniond(rotation_matrix_);

        // 足端初始位置：假设在地面上（Z = foot_radius）
        xHat_.f[0] = Eigen::Vector3d(0.23, 0.15, foot_radius);
        xHat_.f[1] = Eigen::Vector3d(0.23, -0.15, foot_radius);
        xHat_.f[2] = Eigen::Vector3d(-0.096, 0.15, foot_radius);
        xHat_.f[3] = Eigen::Vector3d(-0.096, -0.15, foot_radius);

        // 从 IMU 采样初始化偏置
        const Eigen::Vector3d acc_mean_body = imu_acc_mean_.cast<double>();
        // Eigen::Vector3d acc_mean_body = Eigen::Vector3d::Zero();
        const Eigen::Vector3d g_world(0.0, 0.0, -9.81);
        const Eigen::Vector3d g_body = rotation_matrix_.transpose() * g_world;

        // 加速度计偏置: ba = acc_mean - g_body (静止时应测到-g)
        Eigen::Vector3d ba_init = acc_mean_body + g_body;

        // 陀螺仪偏置
        Eigen::Vector3d bg_init = imu_gyro_mean_.cast<double>();

        xHat_.ba = ba_init;
        xHat_.bg = bg_init;

        // 初始化状态误差
        dx.setZero();

        // 初始化先验误差协方差矩阵P
        P.block<3, 3>(ESKFNominalState::POS_IDX, ESKFNominalState::POS_IDX) *= (init_pos_sigma_ * init_pos_sigma_);
        P.block<3, 3>(ESKFNominalState::VEL_IDX, ESKFNominalState::VEL_IDX) *= (init_vel_sigma_ * init_vel_sigma_);
        P.block<3, 3>(ESKFNominalState::ROT_IDX, ESKFNominalState::ROT_IDX) *= (init_rot_sigma_ * init_rot_sigma_);
        P.block<3, 3>(ESKFNominalState::F1_IDX, ESKFNominalState::F1_IDX) *= (init_fpos_sigma_ * init_fpos_sigma_);
        P.block<3, 3>(ESKFNominalState::F2_IDX, ESKFNominalState::F2_IDX) *= (init_fpos_sigma_ * init_fpos_sigma_);
        P.block<3, 3>(ESKFNominalState::F3_IDX, ESKFNominalState::F3_IDX) *= (init_fpos_sigma_ * init_fpos_sigma_);
        P.block<3, 3>(ESKFNominalState::F4_IDX, ESKFNominalState::F4_IDX) *= (init_fpos_sigma_ * init_fpos_sigma_);
        P.block<3, 3>(ESKFNominalState::BA_IDX, ESKFNominalState::BA_IDX) *= (init_ba_sigma_ * init_ba_sigma_);
        P.block<3, 3>(ESKFNominalState::BG_IDX, ESKFNominalState::BG_IDX) *= (init_bg_sigma_ * init_bg_sigma_);
    }

    vector_t ErrorStateKalmanFilter::update(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        // 更新基础状态（关节、接触、IMU）
        updateJointStates();
        updateContact();
        updateImu();

        double dt = period.seconds();

        // ====================== 运动学计算 ====================
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
        // 关节角度
        qPino.tail(actuatedDofNum) = rbd_state_.segment(6, actuatedDofNum);
        vPino.setZero();
        // 关节速度
        vPino.tail(actuatedDofNum) = rbd_state_.segment(6 + info_.generalizedCoordinatesNum, actuatedDofNum);
        // Pinocchio 正运动学计算
        forwardKinematics(model, data, qPino, vPino);
        computeJointJacobians(model, data);
        updateFramePlacements(model, data);
        // 获取足端位置和速度（相对于 body frame，因为 qPino 中位置和姿态都是0）
        foot_pos_ = ee_kinematics_->getPosition(vector_t());
        foot_vel_ = ee_kinematics_->getVelocity(vector_t(), vector_t());

        // 初始化足端世界坐标
        if (!feet_world_initialized_)
        {
            Eigen::Matrix3d R_wb = xHat_.q.toRotationMatrix();
            for (int i = 0; i < 4; ++i)
            {
                // 初始化足端世界坐标: f_world = p_body + R_wb * f_body
                xHat_.f[i] = xHat_.p + R_wb * foot_pos_[i];
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

        // 日志记录（每隔50次记录一次）
        static int log_counter = 0;
        log_counter++;
        if (is_logger_enabled_ && (log_counter % 50 == 0))
        {
            ESKFNominalState x_nominal_after_update = xHat_;   // 观测更新后状态，名义状态+误差状态
            Eigen::Vector3d a_unbias = in.acc - xHat_.ba;      // 滤波后的加速度（body系）
            Eigen::Vector3d omega_unbias = in.gyro - xHat_.bg; // 滤波后的陀螺仪角速度（body系）
            // 日志输出
            // 状态估计值
            Eigen::Vector3d state_pos = x_nominal_after_update.p;                // 位置估计值
            Eigen::Vector3d state_vel = x_nominal_after_update.v;                // 速度估计值
            Eigen::Vector3d state_rot_zyx = quatToZyx(x_nominal_after_update.q); // 姿态估计值（ZYX欧拉角）
            Eigen::Vector3d state_fpos1 = x_nominal_after_update.f[0];           // 足端1位置估计值
            Eigen::Vector3d state_fpos2 = x_nominal_after_update.f[1];           // 足端2位置估计值
            Eigen::Vector3d state_fpos3 = x_nominal_after_update.f[2];           // 足端3位置估计值
            Eigen::Vector3d state_fpos4 = x_nominal_after_update.f[3];           // 足端4位置估计值
            Eigen::Vector3d state_ba = x_nominal_after_update.ba;                // 加速度计偏置估计值
            Eigen::Vector3d state_bg = x_nominal_after_update.bg;                // 陀螺仪偏置估计值
            logger_->saveDataToCSV("state_pos.csv", "pos_x, pos_y, pos_z", state_pos);
            logger_->saveDataToCSV("state_vel.csv", "vel_x, vel_y, vel_z", state_vel);
            logger_->saveDataToCSV("state_rot_zyx.csv", "yaw, pitch, roll", state_rot_zyx);
            logger_->saveDataToCSV("state_fpos1.csv", "fpos1_x, fpos1_y, fpos1_z", state_fpos1);
            logger_->saveDataToCSV("state_fpos2.csv", "fpos2_x, fpos2_y, fpos2_z", state_fpos2);
            logger_->saveDataToCSV("state_fpos3.csv", "fpos3_x, fpos3_y, fpos3_z", state_fpos3);
            logger_->saveDataToCSV("state_fpos4.csv", "fpos4_x, fpos4_y, fpos4_z", state_fpos4);
            logger_->saveDataToCSV("state_ba.csv", "ba_x, ba_y, ba_z", state_ba);
            logger_->saveDataToCSV("state_bg.csv", "bg_x, bg_y, bg_z", state_bg);

            // ==================== 记录足端世界系速度（用于滑动检测分析）====================
            // 使用更新后的状态计算足端在世界系下的速度
            // 仅记录支撑相的足端速度，理想情况下应为0
            // fvel_world = v_body + R_wb * (omega × fpos_body + fvel_body)
            Eigen::Matrix3d R_wb_updated = x_nominal_after_update.q.toRotationMatrix();
            Eigen::Vector3d omega_body_updated = omega_unbias;

            for (int i = 0; i < 4; ++i)
            {
                // 仅记录支撑相的足端速度
                if (contact_flag_[i])
                {
                    // body系下的足端速度（来自运动学）
                    Eigen::Vector3d fvel_body = foot_vel_[i];
                    Eigen::Vector3d fpos_body = foot_pos_[i];

                    // body系下足端总速度（包含角速度引起的附加速度）
                    Eigen::Vector3d omega_cross_r = omega_body_updated.cross(fpos_body);
                    Eigen::Vector3d fvel_body_total = omega_cross_r + fvel_body;

                    // 世界系下的足端速度
                    Eigen::Vector3d fvel_world = x_nominal_after_update.v + R_wb_updated * fvel_body_total;

                    // 记录支撑相足端世界系速度
                    logger_->saveDataToCSV("footvel_world_" + std::to_string(i) + ".csv",
                                           "fvel_x, fvel_y, fvel_z", fvel_world);
                }
                // 摆动相不记录（足端在移动，速度没有参考意义）
            }

            // 加速度和角速度与真值对比
            Eigen::Vector3d IMU_acc_x(in.acc(0), a_unbias(0), 0.0);
            Eigen::Vector3d IMU_acc_y(in.acc(1), a_unbias(1), 0.0);
            Eigen::Vector3d IMU_acc_z(in.acc(2), a_unbias(2), 0.0);
            Eigen::Vector3d IMU_gyro_x(in.gyro(0), omega_unbias(0), 0.0);
            Eigen::Vector3d IMU_gyro_y(in.gyro(1), omega_unbias(1), 0.0);
            Eigen::Vector3d IMU_gyro_z(in.gyro(2), omega_unbias(2), 0.0);
            logger_->saveDataToCSV("IMU_acc_x.csv", "measured_acc_x, unbias_acc_x, unused", IMU_acc_x);
            logger_->saveDataToCSV("IMU_acc_y.csv", "measured_acc_y, unbias_acc_y, unused", IMU_acc_y);
            logger_->saveDataToCSV("IMU_acc_z.csv", "measured_acc_z, unbias_acc_z, unused", IMU_acc_z);
            logger_->saveDataToCSV("IMU_gyro_x.csv", "measured_gyro_x, unbias_gyro_x, unused", IMU_gyro_x);
            logger_->saveDataToCSV("IMU_gyro_y.csv", "measured_gyro_y, unbias_gyro_y, unused", IMU_gyro_y);
            logger_->saveDataToCSV("IMU_gyro_z.csv", "measured_gyro_z, unbias_gyro_z, unused", IMU_gyro_z);

            // 误差协方差矩阵P
            StateCov P_post = P;
            auto safe_sqrt = [](double v) -> double
            {
                if (!std::isfinite(v))
                    return std::numeric_limits<double>::quiet_NaN();
                return std::sqrt(std::max(0.0, v));
            };
            // 对角线协方差
            Eigen::Vector3d P_diag_pos(safe_sqrt(P_post(0, 0)), safe_sqrt(P_post(1, 1)), safe_sqrt(P_post(2, 2)));
            logger_->saveDataToCSV("P_diag_position.csv", "P_pos_x, P_pos_y, P_pos_z", P_diag_pos);
            Eigen::Vector3d P_diag_vel(safe_sqrt(P_post(3, 3)), safe_sqrt(P_post(4, 4)), safe_sqrt(P_post(5, 5)));
            logger_->saveDataToCSV("P_diag_velocity.csv", "P_vel_x, P_vel_y, P_vel_z", P_diag_vel);
            Eigen::Vector3d P_diag_rot(safe_sqrt(P_post(6, 6)), safe_sqrt(P_post(7, 7)), safe_sqrt(P_post(8, 8)));
            logger_->saveDataToCSV("P_diag_rotation.csv", "P_rot_x, P_rot_y, P_rot_z", P_diag_rot);
            Eigen::Vector3d P_diag_foot1(safe_sqrt(P_post(9, 9)), safe_sqrt(P_post(10, 10)), safe_sqrt(P_post(11, 11)));
            logger_->saveDataToCSV("P_diag_foot1_pos.csv", "P_f1_x, P_f1_y, P_f1_z", P_diag_foot1);
            Eigen::Vector3d P_diag_foot2(safe_sqrt(P_post(12, 12)), safe_sqrt(P_post(13, 13)), safe_sqrt(P_post(14, 14)));
            logger_->saveDataToCSV("P_diag_foot2_pos.csv", "P_f2_x, P_f2_y, P_f2_z", P_diag_foot2);
            Eigen::Vector3d P_diag_foot3(safe_sqrt(P_post(15, 15)), safe_sqrt(P_post(16, 16)), safe_sqrt(P_post(17, 17)));
            logger_->saveDataToCSV("P_diag_foot3_pos.csv", "P_f3_x, P_f3_y, P_f3_z", P_diag_foot3);
            Eigen::Vector3d P_diag_foot4(safe_sqrt(P_post(18, 18)), safe_sqrt(P_post(19, 19)), safe_sqrt(P_post(20, 20)));
            logger_->saveDataToCSV("P_diag_foot4_pos.csv", "P_f4_x, P_f4_y, P_f4_z", P_diag_foot4);
            Eigen::Vector3d P_diag_ba(safe_sqrt(P_post(21, 21)), safe_sqrt(P_post(22, 22)), safe_sqrt(P_post(23, 23)));
            logger_->saveDataToCSV("P_diag_acc_ba.csv", "P_ba_x, P_ba_y, P_ba_z", P_diag_ba);
            Eigen::Vector3d P_diag_bg(safe_sqrt(P_post(24, 24)), safe_sqrt(P_post(25, 25)), safe_sqrt(P_post(26, 26)));
            logger_->saveDataToCSV("P_diag_gyro_bg.csv", "P_bg_x, P_bg_y, P_bg_z", P_diag_bg);
            // 位置和其他的交叉协方差
            Eigen::Vector3d P_pos_vel_cov(safe_sqrt(P_post(0, 3)), safe_sqrt(P_post(1, 4)), safe_sqrt(P_post(2, 5)));
            logger_->saveDataToCSV("P_pos_vel_cov.csv", "P_pos_vel_x, P_pos_vel_y, P_pos_vel_z", P_pos_vel_cov);
            Eigen::Vector3d P_pos_rot_cov(safe_sqrt(P_post(0, 6)), safe_sqrt(P_post(1, 7)), safe_sqrt(P_post(2, 8)));
            logger_->saveDataToCSV("P_pos_rot_cov.csv", "P_pos_rot_x, P_pos_rot_y, P_pos_rot_z", P_pos_rot_cov);
            Eigen::Vector3d P_pos_foot1_cov(safe_sqrt(P_post(0, 9)), safe_sqrt(P_post(1, 10)), safe_sqrt(P_post(2, 11)));
            logger_->saveDataToCSV("P_pos_foot1_cov.csv", "P_pos_foot1_x, P_pos_foot1_y, P_pos_foot1_z", P_pos_foot1_cov);
            Eigen::Vector3d P_pos_foot2_cov(safe_sqrt(P_post(0, 12)), safe_sqrt(P_post(1, 13)), safe_sqrt(P_post(2, 14)));
            logger_->saveDataToCSV("P_pos_foot2_cov.csv", "P_pos_foot2_x, P_pos_foot2_y, P_pos_foot2_z", P_pos_foot2_cov);
            Eigen::Vector3d P_pos_foot3_cov(safe_sqrt(P_post(0, 15)), safe_sqrt(P_post(1, 16)), safe_sqrt(P_post(2, 17)));
            logger_->saveDataToCSV("P_pos_foot3_cov.csv", "P_pos_foot3_x, P_pos_foot3_y, P_pos_foot3_z", P_pos_foot3_cov);
            Eigen::Vector3d P_pos_foot4_cov(safe_sqrt(P_post(0, 18)), safe_sqrt(P_post(1, 19)), safe_sqrt(P_post(2, 20)));
            logger_->saveDataToCSV("P_pos_foot4_cov.csv", "P_pos_foot4_x, P_pos_foot4_y, P_pos_foot4_z", P_pos_foot4_cov);
            Eigen::Vector3d P_pos_ba_cov(safe_sqrt(P_post(0, 21)), safe_sqrt(P_post(1, 22)), safe_sqrt(P_post(2, 23)));
            logger_->saveDataToCSV("P_pos_ba_cov.csv", "P_pos_ba_x, P_pos_ba_y, P_pos_ba_z", P_pos_ba_cov);
            Eigen::Vector3d P_pos_bg_cov(safe_sqrt(P_post(0, 24)), safe_sqrt(P_post(1, 25)), safe_sqrt(P_post(2, 26)));
            logger_->saveDataToCSV("P_pos_bg_cov.csv", "P_pos_bg_x, P_pos_bg_y, P_pos_bg_z", P_pos_bg_cov);

            // 观测
            MeasVec y_log = y;
            for (int i = 0; i < 4; i++)
            {
                // 足端位置残差 (body系)
                Eigen::Vector3d foot_pos_residual = y_log.segment<3>(ESKFNominalState::MEAS_FPOS_IDX + 3 * i);
                logger_->saveDataToCSV("y_pos_residual_foot_" + std::to_string(i) + ".csv", "y_x, y_y, y_z", foot_pos_residual);

                // 足端速度残差 (世界系，支撑相应为0)
                Eigen::Vector3d foot_vel_residual = y_log.segment<3>(ESKFNominalState::MEAS_FVEL_IDX + 3 * i);
                logger_->saveDataToCSV("y_vel_residual_foot_" + std::to_string(i) + ".csv", "yv_x, yv_y, yv_z", foot_vel_residual);
            }
            // 足端高度残差
            Eigen::Vector4d foot_height_residual;
            for (int i = 0; i < 4; i++)
            {
                foot_height_residual(i) = y_log(ESKFNominalState::MEAS_FHEIGHT_IDX + i);
            }
            logger_->saveDataToCSV("y_height02_residual.csv", "yh_0, yh_2, unused",
                                   Eigen::Vector3d(foot_height_residual(0), foot_height_residual(2), 0.0));
            logger_->saveDataToCSV("y_height13_residual.csv", "yh_1, yh_3, unused",
                                   Eigen::Vector3d(foot_height_residual(1), foot_height_residual(3), 0.0));

            // 记录接触状态 (4条腿的接触标志: 1.0=支撑相, 0.0=摆动相)
            // 由于saveDataToCSV只支持Vector3d，分两次记录
            Eigen::Vector3d contact_state_0(contact_flag_[0] ? 1.0 : 0.0, 0.0, 0.0);
            Eigen::Vector3d contact_state_1(contact_flag_[1] ? 1.0 : 0.0, 0.0, 0.0);
            Eigen::Vector3d contact_state_2(contact_flag_[2] ? 1.0 : 0.0, 0.0, 0.0);
            Eigen::Vector3d contact_state_3(contact_flag_[3] ? 1.0 : 0.0, 0.0, 0.0);
            logger_->saveDataToCSV("contact_state_0.csv", "c0, unused, unused", contact_state_0);
            logger_->saveDataToCSV("contact_state_1.csv", "c1, unused, unused", contact_state_1);
            logger_->saveDataToCSV("contact_state_2.csv", "c2, unused, unused", contact_state_2);
            logger_->saveDataToCSV("contact_state_3.csv", "c3, unused, unused", contact_state_3);
        }

        // 更新输出状态
        vector3_t zyx = quatToZyx(xHat_.q);
        rbd_state_.segment<3>(0) = zyx;
        rbd_state_.segment<3>(3) = xHat_.p;
        rbd_state_.segment<3>(18) = in.gyro - xHat_.bg;
        rbd_state_.segment<3>(21) = xHat_.v;
        quat_ = xHat_.q;

        auto odom = getOdomMsg();
        odom.header.stamp = time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base";
        publishMsgs(odom);

        return rbd_state_;
    }

    void ErrorStateKalmanFilter::eskfPrediction(double dt, const ESKFInput &u)
    {
        // 设置过程噪声 Q
        setCovarianceQ(Q, dt);

        Eigen::Vector3d a_unbias = u.acc - xHat_.ba;      // 去偏置后的加速度（body系）
        Eigen::Vector3d omega_unbias = u.gyro - xHat_.bg; // 去偏置后的陀螺仪角速度（body系）
        Eigen::Matrix3d R = xHat_.q.toRotationMatrix();   // 当前姿态旋转矩阵
        Eigen::Vector3d g(0.0, 0.0, -9.81);               // 重力加速度 (世界系)
        Eigen::Vector3d a_world = R * a_unbias + g;       // 世界系下的加速度

        // =================== 名义状态传播 ===================
        xHat_.p = xHat_.p + xHat_.v * dt + 0.5 * a_world * dt * dt; // 位置更新: p_k = p_{k-1} + v * dt
        xHat_.v = xHat_.v + a_world * dt;                           // 速度更新: v_k = v_{k-1} + a * dt
        // 姿态更新: R_k = R_{k-1} * Exp(ω * dt)
        Eigen::Vector3d delta_theta = omega_unbias * dt; // 旋转角度
        Eigen::Quaterniond dq = ExpSO3Quat(delta_theta);
        xHat_.q = xHat_.q * dq;
        xHat_.q.normalize(); // 保持单位四元数
        // 足端位置、偏置保持不变 (由过程噪声建模)
        for (int i = 0; i < 4; i++)
        {
            xHat_.f[i] = xHat_.f[i];
        }
        xHat_.ba = xHat_.ba;
        xHat_.bg = xHat_.bg;

        // =================== 误差状态协方差传播 ===================
        // 计算离散化的状态转移矩阵 A 和噪声雅可比 B
        // 误差状态布局: [δp, δv, δθ, δf1, δf2, δf3, δf4, δba, δbg]
        A = StateCov::Identity();
        A.block<3, 3>(ESKFNominalState::POS_IDX, ESKFNominalState::VEL_IDX) = Eigen::Matrix3d::Identity() * dt; // ∂δp_dot/∂δv = I
        A.block<3, 3>(ESKFNominalState::VEL_IDX, ESKFNominalState::ROT_IDX) = -R * skew(a_unbias) * dt;         // ∂δv_dot/∂δθ = -R * [a_unbias]_x * dt
        A.block<3, 3>(ESKFNominalState::VEL_IDX, ESKFNominalState::BA_IDX) = -R * dt;                           // ∂δv_dot/∂δba = -R * dt
        A.block<3, 3>(ESKFNominalState::ROT_IDX, ESKFNominalState::ROT_IDX) = ExpSO3(-delta_theta);             // ∂δθ_dot/∂δθ = I - [ω_unbias]_x * dt ≈ Exp(-ω * dt)
        A.block<3, 3>(ESKFNominalState::ROT_IDX, ESKFNominalState::BG_IDX) = -Eigen::Matrix3d::Identity() * dt; // ∂δθ_dot/∂δbg = -I * dt

        // 噪声雅可比 B     噪声布局: [n_g(3), n_a(3), n_f1(3), n_f2(3), n_f3(3), n_f4(3), n_ba(3), n_bg(3)]
        B.setZero();
        B.block<3, 3>(ESKFNominalState::ROT_IDX, 0) = -ExpSO3(delta_theta) * dt;         // 陀螺仪噪声 -> 姿态误差
        B.block<3, 3>(ESKFNominalState::VEL_IDX, 3) = -R * dt;                           // 加速度噪声 -> 速度误差
        // 足端过程噪声 -> 足端位置误差 (世界系下的随机游走)
        B.block<3, 3>(ESKFNominalState::F1_IDX, 6) = R * dt;
        B.block<3, 3>(ESKFNominalState::F2_IDX, 9) = R * dt;
        B.block<3, 3>(ESKFNominalState::F3_IDX, 12) = R * dt;
        B.block<3, 3>(ESKFNominalState::F4_IDX, 15) = R * dt;
        // 偏置随机游走噪声
        B.block<3, 3>(ESKFNominalState::BA_IDX, 18) = Eigen::Matrix3d::Identity() * dt;
        B.block<3, 3>(ESKFNominalState::BG_IDX, 21) = Eigen::Matrix3d::Identity() * dt;

        // 先验误差状态更新
        // dx = A * dx; // 噪声omega_k通过协方差传播

        // 协方差传播: P_k = A * P_{k-1} * A^T + B * Q * B^T
        P = A * P * A.transpose() + B * Q * B.transpose();
    }

    void ErrorStateKalmanFilter::eskfUpdate(const MeasVec &y, const MeasJacobian &C, const MeasCov &R)
    {
        // 1. 计算卡尔曼增益
        MeasCov S = C * P * C.transpose() + R; // 计算残差协方差

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
        P = I_KH * P * I_KH.transpose() + K * R * K.transpose(); // 非最优卡尔曼增益下会引入观测噪声项

        // 5. 误差状态重置 (姿态部分)  这里是否有影响？？？？？？？
        resetErrorState(dx);

        // 确保对称性
        P = 0.5 * (P + P.transpose());
    }

    void ErrorStateKalmanFilter::setCovarianceQ(ProcessNoiseCov &Q, double dt)
    {
        Q.setZero();
        if (!std::isfinite(dt) || dt <= 0.0)
        {
            return;
        }

        // 过程噪声协方差 (连续时间噪声 × dt)
        // [ n_g(3), n_a(3), n_f1(3), n_f2(3), n_f3(3), n_f4(3), n_ba(3), n_bg(3) ]
        // =================== 1. 角速度噪声（影响姿态积分）===================
        double gyro_variance = (proc_noise_gyro_ * proc_noise_gyro_);
        Eigen::Matrix3d gyro_Q = gyro_variance * Eigen::Matrix3d::Identity();
        // 降低 yaw(z轴) 方向的过程噪声，因为 yaw 不可观，需要更信任陀螺仪
        gyro_Q(2, 2) *= 0.01;  // yaw 过程噪声降为 1/100
        Q.block<3, 3>(0, 0) = gyro_Q;
        // =================== 2. 加速度噪声（影响速度积分）===================
        double acc_variance = (proc_noise_acc_ * proc_noise_acc_) ;
        Q.block<3, 3>(3, 3) = acc_variance * Eigen::Matrix3d::Identity();
        // =================== 3. 足端过程噪声（基于接触置信度）===================
        double high_suspect_number = 100.0;
        for (int i = 0; i < 4; ++i)
        {
            double estimated_contact = 0.0;
            if (contact_flag_[i])
            {
                // 触地时，置信度为1
                estimated_contact = 1.0;
            }
            // 基础足端过程噪声
            // double base_variance = dt * proc_noise_fpos_ * proc_noise_fpos_;
            double base_variance = (proc_noise_fpos_ * proc_noise_fpos_);
            // 根据接触置信度放大噪声
            // contact=1 时: variance = base_variance
            // contact=0 时: variance = (1 + 1000) * base_variance = 1001 * base_variance
            double foot_variance = (1.0 + (1.0 - estimated_contact) * high_suspect_number) * base_variance;

            // 对每条腿的 xyz 三个轴都设置相同的过程噪声
            Q.block<3, 3>(6 + 3 * i, 6 + 3 * i) = foot_variance * Eigen::Matrix3d::Identity();
        }
        // =================== 4. 偏置过程噪声（随时间漂移）===================
        // 偏置变化较慢，噪声较小，与 dt 成正比
        double ba_variance = (proc_noise_ba_ * proc_noise_ba_) ;
        double bg_variance = (proc_noise_bg_ * proc_noise_bg_) ;
        Q.block<3, 3>(18, 18) = ba_variance * Eigen::Matrix3d::Identity();
        Eigen::Matrix3d bg_Q = bg_variance * Eigen::Matrix3d::Identity();
        // 降低 bg_z 的过程噪声，因为它难以被观测到（弱可观），需要假设它变化更慢
        bg_Q(2, 2) *= 0.01;  // bg_z 过程噪声降为 1/10
        Q.block<3, 3>(21, 21) = bg_Q;
    }

    void ErrorStateKalmanFilter::measurementModelKinematics(const ESKFNominalState &x, const ESKFInput &in, MeasVec &y, MeasJacobian &C, MeasCov &R)
    {
        // 初始化输出
        y.setZero();
        C.setZero();
        R.setZero();

        // ========================================================================
        // 扩展观测模型：包含足端位置、速度和高度
        // 观测向量布局: [foot_pos(12), foot_vel(12), foot_height(4)]
        // 观测方程:
        // 1. 足端位置: h_pos(x) = R^T * (f_world - p_body), 测量值为body系下足端位置
        // 2. 足端速度: h_vel(x) = v_body + ω × f_body + R^T * df/dt
        //              对于支撑相: 足端世界系速度应为0, 即 v + ω × (f - p) ≈ 0
        // 3. 足端高度: h_height(x) = f_world_z, 测量值为foot_radius (支撑相)
        // ========================================================================

        const Eigen::Matrix3d R_fpos = Eigen::Matrix3d::Identity() * (meas_noise_fpos_ * meas_noise_fpos_);
        const Eigen::Matrix3d R_fvel = Eigen::Matrix3d::Identity() * (meas_noise_fvel_ * meas_noise_fvel_);
        const double R_fheight = meas_noise_fheight_ * meas_noise_fheight_;
        const Eigen::Matrix3d R_phi = Eigen::Matrix3d::Identity() * (meas_noise_encoder_ * meas_noise_encoder_);

        // 获取pinocchio雅可比用于编码器噪声传播
        std::vector<VectorFunctionLinearApproximation> eeLinApprox;
        bool haveJacobian = false;
        try
        {
            eeLinApprox = ee_kinematics_->getPositionLinearApproximation(rbd_state_);
            haveJacobian = (eeLinApprox.size() >= 4);
        }
        catch (...)
        {
            haveJacobian = false;
        }

        // 获取当前状态估计值
        Eigen::Matrix3d R_wb = x.q.toRotationMatrix(); // body->world
        Eigen::Vector3d p_k = x.p;                     // body位置 (世界系)
        Eigen::Vector3d v_k = x.v;                     // body速度 (世界系)

        for (int i = 0; i < 4; ++i)
        {
            const bool isContact = contact_flag_[i];
            const int foot_state_idx = ESKFNominalState::F1_IDX + 3 * i;

            // ==================== 1. 足端位置观测 (body系) ====================
            // 测量: z_pos = f_body_meas (来自运动学)
            // 预测: h_pos(x) = R^T * (f_world - p)
            // 残差: y_pos = z_pos - h_pos(x)
            Eigen::Vector3d fpos_body_meas = foot_pos_[i];
            Eigen::Vector3d fpos_world_est = x.f[i];
            Eigen::Vector3d fpos_body_pred = R_wb.transpose() * (fpos_world_est - p_k);
            // 足端位置是否使用足端半径补偿？？？？？？
            Eigen::Vector3d y_pos = fpos_body_meas - fpos_body_pred;
            y.segment<3>(ESKFNominalState::MEAS_FPOS_IDX + 3 * i) = y_pos;

            // 足端位置雅可比
            // ∂h_pos/∂p = -R^T
            // ∂h_pos/∂θ = [R^T * (f_world - p)]_× = [f_body_pred]_×
            // ∂h_pos/∂f = R^T
            Eigen::Matrix3d dh_pos_dp = -R_wb.transpose();
            Eigen::Matrix3d dh_pos_dtheta = lie_group::skew(fpos_body_pred);
            Eigen::Matrix3d dh_pos_df = R_wb.transpose();

            C.block<3, 3>(ESKFNominalState::MEAS_FPOS_IDX + 3 * i, ESKFNominalState::POS_IDX) = dh_pos_dp;
            C.block<3, 3>(ESKFNominalState::MEAS_FPOS_IDX + 3 * i, ESKFNominalState::ROT_IDX) = dh_pos_dtheta;
            C.block<3, 3>(ESKFNominalState::MEAS_FPOS_IDX + 3 * i, foot_state_idx) = dh_pos_df;

            // 位置观测噪声 (包含编码器噪声传播)
            Eigen::Matrix3d R_pos_i = R_fpos;
            if (haveJacobian)
            {
                const auto &dfdx = eeLinApprox[i].dfdx;
                if (dfdx.cols() >= 18)
                {
                    const Eigen::Matrix<double, 3, 3> J_leg = dfdx.block<3, 3>(0, 6 + 3 * i);
                    if (J_leg.allFinite())
                    {
                        R_pos_i += J_leg * R_phi * J_leg.transpose();
                    }
                }
            }

            // ==================== 2. 足端速度观测 (世界系) ====================
            // TODO: fvel_body这个地方的速度是使用运动学算的速度还是估计的足端位置求导来做？
            // 完整的足端世界系速度: fvel_world = v_body + R * (ω_body × f_body + fvel_body)
            // 其中:
            // - v_body: 机体在世界系下的速度
            // - ω_body: 机体角速度 (body系, 已去偏)
            // - f_body: 足端在body系下的位置 (foot_pos_[i])
            // - fvel_body: 足端在body系下的速度 (foot_vel_[i], 来自关节运动学)
            // 支撑相约束: 足端在世界系下应该静止 (fvel_world = 0)
            // 残差: y_vel = z(=0) - h(x)
            Eigen::Matrix3d R_vel_i = R_fvel;
            fpos_body_meas = foot_pos_[i];
            const Eigen::Vector3d fvel_body_meas = foot_vel_[i];
            const Eigen::Vector3d omega_body = in.gyro - x.bg;                                         // body
            const Eigen::Vector3d fvel_body_total = omega_body.cross(fpos_body_meas) + fvel_body_meas; // body
            const Eigen::Vector3d fvel_world_pred = v_k + R_wb * fvel_body_total;                      // world

            if (isContact)
            {
                y.segment<3>(ESKFNominalState::MEAS_FVEL_IDX + 3 * i) = -fvel_world_pred;

                C.block<3, 3>(ESKFNominalState::MEAS_FVEL_IDX + 3 * i, ESKFNominalState::VEL_IDX) =
                    Eigen::Matrix3d::Identity();
                C.block<3, 3>(ESKFNominalState::MEAS_FVEL_IDX + 3 * i, ESKFNominalState::ROT_IDX) =
                    -R_wb * lie_group::skew(fvel_body_total);
                C.block<3, 3>(ESKFNominalState::MEAS_FVEL_IDX + 3 * i, ESKFNominalState::BG_IDX) =
                    R_wb * lie_group::skew(fpos_body_meas);
            }
            else
            {
                // 摆动相：禁用该观测
                y.segment<3>(ESKFNominalState::MEAS_FVEL_IDX + 3 * i).setZero();
                C.block<3, ESKFNominalState::STATE_DIM>(ESKFNominalState::MEAS_FVEL_IDX + 3 * i, 0).setZero();
                R_vel_i *= swing_inflate_;
            }

            // ==================== 3. 足端高度观测 (世界系) ====================
            // 这里采用最直接的贴地约束：支撑相 f_world_z ~= foot_radius_
            double R_height_i = R_fheight;
            if (isContact)
            {
                const double fheight_pred = x.f[i].z();
                y(ESKFNominalState::MEAS_FHEIGHT_IDX + i) = (foot_radius_ - fheight_pred);
                // y = z - h, dy/dfz = -1
                C(ESKFNominalState::MEAS_FHEIGHT_IDX + i, foot_state_idx + 2) = 1.0;
            }
            else
            {
                y(ESKFNominalState::MEAS_FHEIGHT_IDX + i) = 0.0;
                C.row(ESKFNominalState::MEAS_FHEIGHT_IDX + i).setZero();
                R_height_i *= swing_inflate_;
            }
            // ==================== 4. 根据接触相位置信度调整噪声 ====================

            if (!isContact)
            {
                // 摆动相: 大幅增加噪声，降低观测信任度
                R_pos_i *= swing_inflate_;
                R_vel_i *= swing_inflate_;
                R_height_i *= swing_inflate_;
            }

            // ==================== 5. 观测矩阵更新 ====================
            // 足端位置噪声矩阵
            R.block<3, 3>(ESKFNominalState::MEAS_FPOS_IDX + 3 * i, ESKFNominalState::MEAS_FPOS_IDX + 3 * i) = R_pos_i;

            // 速度噪声矩阵
            R.block<3, 3>(ESKFNominalState::MEAS_FVEL_IDX + 3 * i, ESKFNominalState::MEAS_FVEL_IDX + 3 * i) = R_vel_i;

            // 高度噪声标量
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
        Eigen::Quaterniond dq = ExpSO3Quat(delta_theta);
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
    }

    void ErrorStateKalmanFilter::resetErrorState(const StateVec &dx)
    {
        Eigen::Vector3d delta_theta = dx.segment<3>(ESKFNominalState::ROT_IDX);

        // 构造重置雅可比 (只有姿态部分需要修正)
        StateCov G_reset = StateCov::Identity();
        G_reset.block<3, 3>(ESKFNominalState::ROT_IDX, ESKFNominalState::ROT_IDX) = Eigen::Matrix3d::Identity() - 0.5 * skew(delta_theta);

        // 更新协方差
        P = G_reset * P * G_reset.transpose();
    }

    nav_msgs::msg::Odometry ErrorStateKalmanFilter::getOdomMsg()
    {
        nav_msgs::msg::Odometry odom;

        odom.pose.covariance.fill(0.0);
        odom.twist.covariance.fill(0.0);

        odom.pose.pose.position.x = xHat_.p.x();
        odom.pose.pose.position.y = xHat_.p.y();
        odom.pose.pose.position.z = xHat_.p.z();

        odom.pose.pose.orientation.x = quat_.x();
        odom.pose.pose.orientation.y = quat_.y();
        odom.pose.pose.orientation.z = quat_.z();
        odom.pose.pose.orientation.w = quat_.w();

        for (int i = 0; i < 3; ++i)
        {
            for (int j = 0; j < 3; ++j)
            {
                odom.pose.covariance[6 * i + j] = P(ESKFNominalState::POS_IDX + i, ESKFNominalState::POS_IDX + j);
                odom.pose.covariance[6 * (3 + i) + (3 + j)] = P(ESKFNominalState::ROT_IDX + i, ESKFNominalState::ROT_IDX + j);
            }
        }

        odom.twist.twist.linear.x = xHat_.v.x();
        odom.twist.twist.linear.y = xHat_.v.y();
        odom.twist.twist.linear.z = xHat_.v.z();

        odom.twist.twist.angular.x = angular_vel_local_.x() - xHat_.bg.x();
        odom.twist.twist.angular.y = angular_vel_local_.y() - xHat_.bg.y();
        odom.twist.twist.angular.z = angular_vel_local_.z() - xHat_.bg.z();

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

        loadData::loadPtreeValue(pt, slip_threshold_, prefix + "SlipThreshold", verbose);
        loadData::loadPtreeValue(pt, slip_inflate_, prefix + "SlipInflate", verbose);
        loadData::loadPtreeValue(pt, swing_inflate_, prefix + "SwingInflate", verbose);

        loadData::loadPtreeValue(pt, init_pos_sigma_, prefix + "InitPosSigma", verbose);
        loadData::loadPtreeValue(pt, init_vel_sigma_, prefix + "InitVelSigma", verbose);
        loadData::loadPtreeValue(pt, init_rot_sigma_, prefix + "InitRotSigma", verbose);
        loadData::loadPtreeValue(pt, init_fpos_sigma_, prefix + "InitFootPosSigma", verbose);
        loadData::loadPtreeValue(pt, init_bg_sigma_, prefix + "InitBiasGyroSigma", verbose);
        loadData::loadPtreeValue(pt, init_ba_sigma_, prefix + "InitBiasAccelSigma", verbose);

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
        loadData::loadPtreeValue(pt, foot_radius_, prefix + "FootRadius", verbose);
    }
}
