#pragma once

#include "StateEstimateBase.h"
#include "ContactEstimator.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_legged_robot/gait/LegLogic.h>

#include <rclcpp/rclcpp.hpp>
#include <ocs2_core/misc/LoadData.h>
#include <nav_msgs/msg/odometry.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <memory>
#include <vector>
#include <array>
#include <optional>
#include <cstdint>
#include <string>

#include <Eigen/Dense>


#include <sophus/so3.hpp>
#include "STAObserver.h"

#include "log_save.h"
#include <rclcpp/rclcpp.hpp>

// 前向声明
namespace ocs2::legged_robot {
    class SwitchedModelReferenceManager;
}

namespace ocs2::legged_robot
{

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
        static constexpr int MEAS_DIM = 28;            // 总观测维度 
       

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
        Eigen::Vector3d dist_pos; // 位置扰动 (STA)
        Eigen::Vector3d dist_vel; // 速度扰动 (STA)
        Eigen::Vector3d dist_rot; // 姿态扰动 (STA)

        ESKFInput()
        {
            acc.setZero();
            gyro.setZero();
            dist_pos.setZero();
            dist_vel.setZero();
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

        void setReferenceManager(std::shared_ptr<SwitchedModelReferenceManager> reference_manager_ptr) {
            reference_manager_ptr_ = reference_manager_ptr;
        }

    private:
        PinocchioInterface pinocchio_interface_;
        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_; // 使用pinocchio接口计算运动学
        // 足端帧名称
        const std::array<std::string, 4> foot_frame_names_ = {
            "FL_foot", "FR_foot", "RL_foot", "RR_foot"
        };
        std::array<std::size_t, 4> ee_frame_ids_;  // 足端帧索引（缓存，避免在头文件包含 pinocchio）
        Eigen::Matrix<double, 6, Eigen::Dynamic> J_full_;  // 完整雅可比矩阵 (6×nv)
        std::vector<Eigen::Matrix<double, 3, Eigen::Dynamic>> Jc;                    // 足端接触雅可比矩阵 (3×nv)

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
        bool feet_world_initialized_ = false;
        // 接触估计器
        std::unique_ptr<ContactEstimator> contact_estimator_;
        bool use_contact_estimator_ = true;  // 是否使用接触估计器
        std::array<double, 4> contact_prob_ = {0.0, 0.0, 0.0, 0.0}; // 接触概率
        // 步态相位信息获取
        std::shared_ptr<SwitchedModelReferenceManager> reference_manager_ptr_;

        // 与 MPC/ModeSchedule 一致的控制时间
        double controller_time_ = 0.0;


        double proc_noise_acc_ = 0.017; // 加速度过程噪声
        double proc_noise_gyro_ = 0.002; // 角速度过程噪声
        double proc_noise_fpos_ = 0.05; // 足端位置过程噪声
        double proc_noise_ba_ = 0.0001; // 加速度偏置过程噪声
        double proc_noise_bg_ = 0.001; // 陀螺仪偏置过程噪声

        double meas_noise_fvel_ = 0.1;     // 足端速度测量噪声
        double meas_noise_fpos_ = 0.1;     // 足端位置测量噪声
        double meas_noise_fheight_ = 0.01; // 足端高度测量噪声
        double meas_noise_ba_ = 0.0001;    // 加速度偏置测量噪声
        double meas_noise_bg_ = 0.0001;    // 陀螺仪偏置测量噪声
        double meas_noise_encoder_ = 0.02; // 编码器测量噪声
        double meas_noise_attitude_ = 0.01; // IMU姿态软约束噪声 (小值=强约束, 大值=弱约束)
        double meas_noise_attitude_yaw_ = 0.01; // IMU姿态软约束噪声 (小值=强约束, 大值=弱约束)
        double high_suspect_number = 100.0;

        // 足端参数
        double foot_radius_ = 0.019; // 足端半径，用于高度约束
        double min_contact_trust_ = 0.05; // 接触概率下限，避免观测被完全关闭
        double swing_inflate_ = 100.0; // 摆动相膨胀系数

        // STA Observer parameters  
        bool use_sta_observer_ = true; // 是否使用STA Observer
        double sta_bandwidth_ = 5.0; // STA Observer带宽
        int sta_qd_window_ = 20; // STA Observer协方差窗口大小
        double sta_epsilon_ = 1e-6; // STA Observer误差阈值
        double sta_qd_regularization_ = 1e-6; // STA Observer协方差正则化参数
        double sta_qd_max_ = 10.0; // STA Observer最大协方差
        double sta_max_disturbance_ = 50.0; // STA Observer最大扰动
        

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

        /** IMU 姿态软约束：独立卡尔曼更新，将 ESKF 旋转柔性约束到 IMU 输出 */
        void fuseImuAttitude();

        /**基于运动学的观测模型
         * 根据当前名义状态x和输入in，计算观测残差y、雅可比C和噪声协方差R
         */
        void measurementModelKinematics(const ESKFNominalState &x,const ESKFInput &in,
                                        MeasVec &y, MeasJacobian &C, MeasCov &R);

        nav_msgs::msg::Odometry getOdomMsg();

        void footStateEstimation(const rclcpp::Time &time, const rclcpp::Duration &period);     // 足端力和接触状态估计
        



        // Ground Truth (从Gazebo获取真实位姿用于对比)
        void updateGroundTruth();
        void publishGroundTruth(const rclcpp::Time &time);
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
        Eigen::Vector3d gt_position_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d gt_linear_velocity_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond gt_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d gt_angular_velocity_ = Eigen::Vector3d::Zero();
        bool ground_truth_enabled_ = true; // 是否启用Ground Truth对比
        bool sim_mode_ = true;             // true:仿真，false:实机

        // STA Observer
        STAObserver sta_observer_;

        // 日志保存
        std::unique_ptr<quadruped::logs::LogSave> logger_;
        bool is_logger_enabled_ = true;

    };
}
