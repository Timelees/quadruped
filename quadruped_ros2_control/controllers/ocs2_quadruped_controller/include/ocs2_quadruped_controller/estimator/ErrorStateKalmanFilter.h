#pragma once

#include "StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <rclcpp/rclcpp.hpp>
#include <ocs2_core/misc/LoadData.h>
#include <nav_msgs/msg/odometry.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <memory>
#include <vector>
#include <array>
#include <optional>

#include <Eigen/Dense>

// 基于李群李代数的 ESKF 实现
// #include "LieGroupESKF.h"
#include "LieGroupUtils.h"
#include <sophus/so3.hpp>
#include "STAObserver.h"

#include "log_save.h"

// 腿部运动学计算
#include "controller_common/common/leg.h"

namespace ocs2::legged_robot
{
    using namespace lie_group;

    // 名义状态定义
    struct ESKFNominalState
    {
        Eigen::Vector3d p;    // body位置 (世界系)
        Eigen::Vector3d v;    // body速度 (世界系)
        Eigen::Quaterniond q; // body姿态 (机体系 -> 世界系 )
        Eigen::Vector3d f[4]; // 4个足端位置 (世界系)
        Eigen::Vector3d ba;   // 加速度计偏置
        Eigen::Vector3d bg;   // 陀螺仪偏置

        ESKFNominalState()
        {
            p.setZero();
            v.setZero();
            q = Eigen::Quaterniond::Identity();
            for (int i = 0; i < 4; ++i)
                f[i].setZero();
            ba.setZero();
            bg.setZero();
        }

        // 状态维度定义
        static constexpr int POS_IDX = 0;
        static constexpr int VEL_IDX = 3;
        static constexpr int ROT_IDX = 6; // 误差状态中姿态转为旋转矩阵， 3*3 维
        static constexpr int F1_IDX = 9;
        static constexpr int F2_IDX = 12;
        static constexpr int F3_IDX = 15;
        static constexpr int F4_IDX = 18;
        static constexpr int BA_IDX = 21;
        static constexpr int BG_IDX = 24;

        static constexpr int STATE_DIM = 27;         // 误差状态维度 【p(3), v(3), G(3), f(12), ba(3), bg(3)】
        static constexpr int NOMINAL_DIM = 28;       // 名义状态维度 (四元数占4维)
        static constexpr int PROCESS_NOISE_DIM = 24; // 过程噪声维度 [n_g(3), n_a(3), n_f(12), n_ba(3), n_bg(3)]

        // 观测维度定义  [foot_pos(12), foot_vel(12), foot_height(4)]
        static constexpr int MEAS_FOOT_POS_DIM = 12;   // 足端位置观测维度
        static constexpr int MEAS_FOOT_VEL_DIM = 12;   // 足端速度观测维度
        static constexpr int MEAS_FOOT_HEIGHT_DIM = 4; // 足端高度观测维度
        static constexpr int MEAS_DIM = 28;            // 总观测维度 (12 + 12 + 4 = 28)
       

        // 观测向量索引
        static constexpr int MEAS_FPOS_IDX = 0;     // 足端位置起始索引
        static constexpr int MEAS_FVEL_IDX = 12;    // 足端速度起始索引
        static constexpr int MEAS_FHEIGHT_IDX = 24; // 足端高度起始索引
    };

    // ESKF输入结构体 (IMU 测量值 + STA 扰动)
    struct ESKFInput
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Eigen::Vector3d acc;      // 加速度计测量 (机体系)
        Eigen::Vector3d gyro;     // 陀螺仪测量 (机体系)
        Eigen::Vector3d dist_vel; // 速度扰动 (STA)
        Eigen::Vector3d dist_acc; // 加速度扰动 (STA)
        Eigen::Vector3d dist_rot; // 姿态扰动 (STA)

        ESKFInput()
        {
            acc.setZero();
            gyro.setZero();
            dist_vel.setZero();
            dist_acc.setZero();
            dist_rot.setZero();
        }
    };

    using StateVec = Eigen::Matrix<double, ESKFNominalState::STATE_DIM, 1>;
    using StateCov = Eigen::Matrix<double, ESKFNominalState::STATE_DIM, ESKFNominalState::STATE_DIM>;
    using ProcessNoiseCov = Eigen::Matrix<double, ESKFNominalState::PROCESS_NOISE_DIM, ESKFNominalState::PROCESS_NOISE_DIM>;
    using StateNoiseJacobian = Eigen::Matrix<double, ESKFNominalState::STATE_DIM, ESKFNominalState::PROCESS_NOISE_DIM>;
    using MeasVec = Eigen::Matrix<double, ESKFNominalState::MEAS_DIM, 1>;
    using MeasCov = Eigen::Matrix<double, ESKFNominalState::MEAS_DIM, ESKFNominalState::MEAS_DIM>;
    using MeasJacobian = Eigen::Matrix<double, ESKFNominalState::MEAS_DIM, ESKFNominalState::STATE_DIM>;

    /**
     * @brief 基于李群李代数的 Error-State Kalman Filter
     */
    class ErrorStateKalmanFilter final : public StateEstimateBase
    {
    public:
        ErrorStateKalmanFilter(PinocchioInterface pinocchio_interface, CentroidalModelInfo info,
                               const PinocchioEndEffectorKinematics &ee_kinematics,
                               CtrlInterfaces &ctrl_component,
                               const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);
        ~ErrorStateKalmanFilter() override;

        vector_t update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        void init();

        void loadSettings(const std::string &task_file, const bool verbose);

    private:
        // 基于李群李代数的 ESKF 实现
        // std::shared_ptr<LieGroupESKF> kf_;

        PinocchioInterface pinocchio_interface_;
        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_; // 使用pinocchio接口计算运动学

        // 腿部运动学模型 (使用解析方法替代 Pinocchio)
        std::unique_ptr<QuadrupedRobotModel> leg_kinematics_;

        // 运动学计算的足端位置和速度（body frame）
        std::vector<vector3_t> foot_pos_;
        std::vector<vector3_t> foot_vel_;

        // body->world的姿态旋转矩阵
        Eigen::Matrix3d rotation_matrix_ = Eigen::Matrix3d::Identity();

        // imu_link->body 的变换矩阵（只有平移，没有旋转）
        Eigen::Vector3d imu_to_body_translation_{-0.033, 0.017, -0.051};        // cyberdog的imu和body的平移
        Eigen::Quaterniond imu_to_body_rotation_ = Eigen::Quaterniond::Identity();
        Eigen::Matrix3d R_imu_to_body_ = imu_to_body_rotation_.toRotationMatrix();

        // contact edge detection
        contact_flag_t prev_contact_flag_{};
        bool feet_world_initialized_ = false;

       

        // 初始化误差协方差矩阵P的参数
        double init_pos_sigma_ = 0.01;
        double init_vel_sigma_ = 0.01;
        double init_rot_sigma_ = 0.01;
        double init_fpos_sigma_ = 0.01;
        double init_bg_sigma_ = 0.01; // 增大以适应较大的陀螺仪偏置不确定性
        double init_ba_sigma_ = 0.01;


        // imu_sanity_check 测量的IMU偏置
        Eigen::Vector3d imu_acc_mean_ = Eigen::Vector3d(0.0, 0.0, 9.790); // ba   (-0.011, 0.011, 9.790)
        Eigen::Vector3d imu_gyro_mean_ = Eigen::Vector3d(0.0, 0.0, 0.0);  // bg   (-0.0001, -0.0001, -0.52)

        // ==================== 启动静止期IMU偏置采样 ====================
        static constexpr int BIAS_INIT_SAMPLES = 200;     // 采样帧数（约0.2秒@1kHz）
        int bias_init_sample_count_ = 0;                  // 当前采样计数
        Eigen::Vector3d gyro_sum_ = Eigen::Vector3d::Zero();  // 陀螺仪累加
        Eigen::Vector3d acc_sum_ = Eigen::Vector3d::Zero();   // 加速度计累加
        bool bias_initialized_ = false;                   // 偏置是否已初始化

        // 加速度过程噪声
        double proc_noise_acc_x_ = 0.017;
        double proc_noise_acc_y_ = 0.017;
        double proc_noise_acc_z_ = 0.017;
        // 角速度过程噪声
        double proc_noise_gyro_x_ = 0.002;
        double proc_noise_gyro_y_ = 0.002;
        double proc_noise_gyro_z_ = 0.5211;
        // 足端位置过程噪声
        double proc_noise_fpos_ = 0.05;
        // 加速度偏置过程噪声
        double proc_noise_ba_x_ = 0.0001;
        double proc_noise_ba_y_ = 0.0001;
        double proc_noise_ba_z_ = 0.0001;
        // 陀螺仪偏置过程噪声
        double proc_noise_bg_x_ = 0.001;
        double proc_noise_bg_y_ = 0.001;
        double proc_noise_bg_z_ = 0.5211;

        double meas_noise_fvel_ = 0.1;     // 足端速度测量噪声
        double meas_noise_fpos_ = 0.1;     // 足端位置测量噪声
        double meas_noise_fheight_ = 0.01; // 足端高度测量噪声
        double meas_noise_ba_ = 0.0001;    // 加速度偏置测量噪声
        double meas_noise_bg_ = 0.0001;    // 陀螺仪偏置测量噪声
        double meas_noise_encoder_ = 0.02; // 编码器测量噪声

        double high_suspect_number = 100.0;

        // 足端参数
        double foot_radius_ = 0.019; // 足端半径，用于高度约束

        double slip_threshold_ = 0.05; // 滑移阈值
        double slip_inflate_ = 10.0;   // 支撑相滑移膨胀系数
        double swing_inflate_ = 100.0; // 摆动相膨胀系数
        
        // 滑动检测状态（用于日志和调试）
        std::array<bool, 4> slip_flag_ = {false, false, false, false};
        std::array<double, 4> foot_world_speed_ = {0.0, 0.0, 0.0, 0.0};

        ESKFNominalState xHat_;                            // 滤波器状态估计值
        StateVec dx;                                       // 误差状态
        StateCov P = StateCov::Identity();                 // 误差协方差矩阵
        ProcessNoiseCov Q = ProcessNoiseCov::Identity();   // 过程噪声协方差
        MeasVec y = MeasVec::Zero();                       // 观测残差
        MeasJacobian C = MeasJacobian::Zero();             // 观测雅可比
        MeasCov R = MeasCov::Zero();                       // 观测噪声协方差
        StateCov A = StateCov::Identity();                 // 状态转移矩阵
        StateNoiseJacobian B = StateNoiseJacobian::Zero(); // 噪声雅可比矩阵 (27x24)


        void imuCorrect();  // IMU校正
        Eigen::Vector3d acc_correct;
        Eigen::Vector3d w_correct; // 角速度直接转换，无需补偿

        // 设置过程噪声协方差矩阵 Q
        void setCovarianceQ(ProcessNoiseCov &Q, double dt);

        void eskfPrediction(double dt, const ESKFInput &u);

        void eskfUpdate(const MeasVec &y, const MeasJacobian &C, const MeasCov &R);

        void injectErrorState(const StateVec &dx);

        void resetErrorState(const StateVec &dx);

        /**基于运动学的观测模型
         * 计算观测残差、雅可比和噪声协方差
         * @param x 当前名义状态
         * @param y [out] 观测残差
         * @param C [out] 观测雅可比
         * @param R [out] 观测噪声协方差
         */
        void measurementModelKinematics(const ESKFNominalState &x,
                                        const ESKFInput &in,
                                        MeasVec &y,
                                        MeasJacobian &C,
                                        MeasCov &R);

        nav_msgs::msg::Odometry getOdomMsg();

        // Ground Truth (从Gazebo获取真实位姿用于对比)
        void updateGroundTruth();
        void publishGroundTruth(const rclcpp::Time &time);
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
        Eigen::Vector3d gt_position_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d gt_linear_velocity_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond gt_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d gt_angular_velocity_ = Eigen::Vector3d::Zero();
        bool ground_truth_enabled_ = true; // 是否启用Ground Truth对比

        // STA Observer
        STAObserver sta_observer_;

        // 日志保存
        std::unique_ptr<quadruped::logs::LogSave> logger_;
        bool is_logger_enabled_ = true;
    };
}
