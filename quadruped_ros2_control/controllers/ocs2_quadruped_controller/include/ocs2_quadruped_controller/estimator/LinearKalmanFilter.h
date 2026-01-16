//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include "StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <tf2_ros/transform_broadcaster.h>

#include "log_save.h"
#include <rclcpp/rclcpp.hpp>


namespace ocs2::legged_robot
{
    class KalmanFilterEstimate final : public StateEstimateBase
    {
    public:
        KalmanFilterEstimate(PinocchioInterface pinocchio_interface, CentroidalModelInfo info,
                             const PinocchioEndEffectorKinematics &ee_kinematics,
                             CtrlInterfaces &ctrl_component,
                             const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);

        vector_t update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        void loadSettings(const std::string &task_file, bool verbose);

    protected:
        /**
         * @brief 零速度更新 (ZUPT - Zero Velocity Update)
         * 当检测到机器人处于静止或稳定接触状态时，约束速度估计以防止积分漂移
         */
        void applyZeroVelocityUpdate();

        /**
         * @brief 约束协方差矩阵，防止不确定性无限增长
         */
        void constrainCovariance();

        /**
         * @brief 检测四足站立状态并校准位置
         */
        void checkFourLegStand(const std::vector<vector3_t>& eePos);

        /**
         * @brief 计算基于相位的信任度
         * @param phase 接触相位 [0, 1]
         * @param trust_window 信任窗口
         * @return 信任度 [0, 1]
         */
        scalar_t computeTrust(scalar_t phase, scalar_t trust_window);

        nav_msgs::msg::Odometry getOdomMsg();

        PinocchioInterface pinocchio_interface_;                        // 机器人运动学/动力学模型
        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_; // 脚部运动学计算器

        vector_t feet_heights_; // 脚部高度测量值

        // Kalman Filter 参数
        scalar_t foot_radius_ = 0.02;
        scalar_t imu_process_noise_position_ = 0.02;
        scalar_t imu_process_noise_velocity_ = 0.02;
        scalar_t footProcessNoisePosition_ = 0.002;
        scalar_t footSensorNoisePosition_ = 0.005;
        scalar_t footSensorNoiseVelocity_ = 0.1;
        scalar_t footHeightSensorNoise_ = 0.01;
        scalar_t high_suspect_number_ = 100.0;


        
        


    private:
        // 接触点数量（3D 接触 + 6D 接触：4）、接触点总维度（每足端 3D 位置：12）、状态向量维度（6D 基座状态 + 足端位置：18）、	观测向量维度（足端位置、速度和高度：28  ）
        size_t numContacts_, dimContacts_, numState_, numObserve_;
        // 状态转移矩阵（动力学模型）、控制输入矩阵（加速度计）、观测矩阵、过程噪声协方差、状态估计协方差、观测噪声协方差
        matrix_t a_, b_, c_, q_, p_, r_;
        // 状态估计向量（[位置(3), 速度(3), 接触点位置(3×N)]）、脚部位置观测值、脚部速度观测值
        vector_t xHat_, ps_, vs_;

        // 日志记录
        std::unique_ptr<quadruped::logs::LogSave> logger_;
        bool is_logger_enabled_ = true;
        // 控制日志写入频率：以秒为单位的间隔和上次写入时间
        double log_interval_sec_ = 0.1; // 默认每0.1s写一次
        rclcpp::Time last_log_time_;
    };
}
