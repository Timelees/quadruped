//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include "StateEstimateBase.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <tf2_ros/transform_broadcaster.h>

namespace ocs2::legged_robot {
    class KalmanFilterEstimate final : public StateEstimateBase {
    public:
        KalmanFilterEstimate(PinocchioInterface pinocchio_interface, CentroidalModelInfo info,
                             const PinocchioEndEffectorKinematics &ee_kinematics,
                             CtrlInterfaces &ctrl_component,
                             const rclcpp_lifecycle::LifecycleNode::SharedPtr &node);

        vector_t update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        void loadSettings(const std::string &task_file, bool verbose);

    protected:
        nav_msgs::msg::Odometry getOdomMsg();

        PinocchioInterface pinocchio_interface_;        // 机器人运动学/动力学模型
        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_;     // 脚部运动学计算器

        vector_t feet_heights_;     // 脚部高度测量值

        // Config
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
    };
}
