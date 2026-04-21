//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include "StateEstimateBase.h"
#include "ContactEstimator.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_legged_robot/gait/LegLogic.h>

#include <tf2_ros/transform_broadcaster.h>
#include <realtime_tools/realtime_tools/realtime_buffer.hpp>

#include "log_save.h"
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// 前向声明
namespace ocs2::legged_robot {
    class SwitchedModelReferenceManager;
}


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

        void computeForwardKinematics();

        void loadSettings(const std::string &task_file, bool verbose);

        // 设置 SwitchedModelReferenceManager 用于获取步态相位信息
        void setReferenceManager(std::shared_ptr<SwitchedModelReferenceManager> reference_manager_ptr) {
            reference_manager_ptr_ = reference_manager_ptr;
        }

    protected:
        nav_msgs::msg::Odometry getOdomMsg();
        nav_msgs::msg::Odometry getGroundTruthOdomMsg();

        PinocchioInterface pinocchio_interface_;                        // 机器人运动学/动力学模型
        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_; // 脚部运动学计算器

        vector_t feet_heights_; // 脚部高度测量值

        // Kalman Filter 参数
        scalar_t foot_radius_ = 0.019;
        scalar_t imu_process_noise_position_ = 0.02;
        scalar_t imu_process_noise_velocity_ = 0.02;
        scalar_t footProcessNoisePosition_ = 0.002;
        scalar_t footSensorNoisePosition_ = 0.005;
        scalar_t footSensorNoiseVelocity_ = 0.1;
        scalar_t footHeightSensorNoise_ = 0.01;
        scalar_t high_suspect_number_ = 10000.0;

    private:
        // 接触点数量（3D 接触 + 6D 接触：4）、接触点总维度（每足端 3D 位置：12）、状态向量维度（6D 基座状态 + 足端位置：18）、	观测向量维度（足端位置、速度和高度：28  ）
        size_t numContacts_, dimContacts_, numState_, numObserve_;
        // 状态转移矩阵（动力学模型）、控制输入矩阵（加速度计）、观测矩阵、过程噪声协方差、状态估计协方差、观测噪声协方差
        matrix_t a_, b_, c_, q_, p_, r_;
        // 状态估计向量（[位置(3), 速度(3), 接触点位置(3×N)]）、脚部位置观测值、脚部速度观测值
        vector_t xHat_, ps_, vs_;

        
        // 外部定位话题订阅 (异步接收, 通过 RealtimeBuffer 线程安全传递)
        void updateFromTopic();                     // 从外部定位话题更新位姿状态
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr localization_sub_;
        realtime_tools::RealtimeBuffer<nav_msgs::msg::Odometry> localization_buffer_;
        bool topicUpdated_ = false;                 // 是否有新的定位数据到达
        

        // Ground Truth (从Gazebo获取真实位姿用于对比)
        void updateGroundTruth();                   // 从 odom_state_interface_ 读取真值 (用于日志对比)
        void publishGroundTruth(const rclcpp::Time &time);
        // Ground Truth 发布
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
        // Ground Truth 缓存 (用于日志对比)
        Eigen::Vector3d gt_position_ = Eigen::Vector3d::Zero();
        Eigen::Vector3d gt_linear_velocity_ = Eigen::Vector3d::Zero();
        Eigen::Quaterniond gt_orientation_ = Eigen::Quaterniond::Identity();
        Eigen::Vector3d gt_angular_velocity_ = Eigen::Vector3d::Zero();
        bool ground_truth_enabled_ = true; // 是否启用Ground Truth对比
        bool is_initialized_ = false;      // 状态是否已初始化
        

        // 日志记录
        std::unique_ptr<quadruped::logs::LogSave> logger_;
        bool is_logger_enabled_ = true;
        bool sim_mode_ = true;      // true:仿真，false:实机

        // 接触估计器
        std::unique_ptr<ContactEstimator> contact_estimator_;
        bool use_contact_estimator_ = true;  // 是否使用接触估计器

        // 步态相位信息获取
        std::shared_ptr<SwitchedModelReferenceManager> reference_manager_ptr_;
   
    
        // 从 MPC 的 ModeSchedule 获取支撑相和摆动相
        void getGaitPhaseFromMPC(scalar_t current_time,
                                  std::array<scalar_t, 4>& contact_phase,
                                  std::array<scalar_t, 4>& swing_phase);

        // 点云话题订阅 (用于调试足端位置处的点云高度)
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        std::mutex pointcloud_mutex_;
        sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
        bool pointcloud_received_ = false;

        // TF2 变换 (用于点云坐标变换)
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    };
}
