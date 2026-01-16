#pragma once

#include "StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <tf2_ros/transform_broadcaster.h>

#include "log_save.h"


namespace ocs2::legged_robot { 
    class ExtendKalmanFilter final : public StateEstimateBase { 
    public:
        ExtendKalmanFilter(PinocchioInterface pinocchio_interface, 
                            CentroidalModelInfo info,
                            const PinocchioEndEffectorKinematics &ee_kinematics,
                            CtrlInterfaces &ctrl_component,
                            const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);
        void init_state();
        vector_t update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        void loadSettings(const std::string &task_file, bool verbose);

    protected:
        nav_msgs::msg::Odometry getOdomMsg();
        
        PinocchioInterface pinocchio_interface_;        // 机器人运动学/动力学模型
        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_;     // 脚部运动学计算器

        vector_t feet_heights_;     // 脚部高度测量值

        // Config
        scalar_t foot_radius_ = 0.02;
        scalar_t imu_process_noise_position_ = 0.01;        // 位置预测协方差
        scalar_t imu_process_noise_velocity_ = 0.01;        // 速度预测协方差
        scalar_t footProcessNoisePosition_ = 0.002;         // 脚部位置预测协方差
        scalar_t footSensorNoisePosition_ = 0.005;          // 足端位置测量协方差
        scalar_t footSensorNoiseVelocity_ = 0.1;            // 足端速度测量协方差
        scalar_t footHeightSensorNoise_ = 0.01;             // 足端高度测量协方差
        scalar_t high_suspect_number_ = 100.0;              // 置信度系数

    private:
        // 接触点数量（腿部数量4）、接触点总维度（关节数量12）
        size_t numContacts_, dimContacts_;
        size_t numState_;           // 状态向量维度【world_frame】（base_position[3]+base_vec[3]+足端位置[12]：18）
        size_t numObserve_;         // 观测向量维度【world_frame】（body_position-foot_position[12] + foot_vec[12] + foot_position_z[4]：28）

        // 状态值
        // 状态向量：base_position[3]+base_vec[3]+足端位置[12]
        vector_t x_pri;         // 状态预测值，先验估计值
        vector_t x_post;        // 状态更新值，后验估计值
        // 状态误差协方差
        matrix_t P_pri;         // 先验误差协方差矩阵
        matrix_t P_post;        // 后验误差协方差矩阵
        // 系统模型
        matrix_t A;             // 状态转移矩阵（动力学模型）
        matrix_t B;             // 控制输入矩阵（加速度计）
        matrix_t Q;             // 预测噪声协方差矩阵

        // 观测值
        // 0 1 2   FL pos residual
        // 3 4 5   FR pos residual
        // 6 7 8   RL pos residual
        // 9 10 11 RR pos residual
        // 12 13 14 vel from FL
        // 15 16 17 vel from FR
        // 18 19 20 vel from RL
        // 21 22 23 vel from RR
        // 24 25 26 27 foot height
        vector_t y_actual;              // 观测向量，实际测量值
        vector_t y_estimate;          // 观测估计值，观测矩阵*先验估计状态值
        vector_t error_y;        // 观测误差，实际测量值-观测估计值
        vector_t Serror_y;       // S^-1*error_y
        matrix_t C;              // 观测矩阵
        matrix_t SC;             // S^-1*C
        matrix_t R;              // 观测噪声协方差矩阵

        matrix_t eye3;           // 3维单位矩阵
        matrix_t S;               // 观测噪声协方差矩阵的逆矩阵
        matrix_t K;               // 卡尔曼增益

        // 足端状态   脚部位置观测值、脚部速度观测值
        vector_t feet_positions_, feet_velocities_;

        std::unique_ptr<quadruped::logs::LogSave> logger_;
        bool is_logger_enabled_ = true;
    };
};


