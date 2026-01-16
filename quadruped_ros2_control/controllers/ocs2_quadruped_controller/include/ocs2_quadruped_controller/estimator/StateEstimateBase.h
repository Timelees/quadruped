//
// Created by qiayuan on 2021/11/15.
//
#pragma once

#include <array>
#include <controller_common/CtrlInterfaces.h>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_legged_robot/common/ModelSettings.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "control_input_msgs/msg/euler_angle.hpp"

namespace ocs2::legged_robot
{
    class StateEstimateBase
    {
    public:
        virtual ~StateEstimateBase() = default;

        StateEstimateBase(CentroidalModelInfo info, CtrlInterfaces &ctrl_component,
                          rclcpp_lifecycle::LifecycleNode::SharedPtr node);

        virtual void updateJointStates();

        virtual void updateContact();

        virtual void updateImu();

        virtual vector_t update(const rclcpp::Time &time, const rclcpp::Duration &period) = 0;

        [[nodiscard]] size_t getMode() const { return stanceLeg2ModeNumber(contact_flag_); }

    protected:
        void initPublishers();

        void updateAngular(const vector3_t &zyx, const vector_t &angularVel);

        void updateLinear(const vector_t &pos, const vector_t &linearVel);

        void publishMsgs(const nav_msgs::msg::Odometry &odom) const;

        void publishErrorMsgs(const geometry_msgs::msg::Vector3 &error_msg) const;

        void footPositionCorrection(std::vector<Eigen::Vector3d> &footPositions, int legNumber, const rclcpp::Time &time);    // 足端位置校正

        CtrlInterfaces &ctrl_component_;
        CentroidalModelInfo info_;

        contact_flag_t contact_flag_{}; // 脚部接触状态标志
        std::array<double, 4> contact_phase_{}; // 接触相位 [0,1]: 0=摆动开始, 1=支撑稳定
        double feet_force_threshold_ = 0.5;  // 降低阈值以检测更多接触（原值5.0过高）
        double footFallPositionRecord[4][3] = {0}; // 记录足端初始落脚点位置

        vector3_t zyx_offset_ = vector3_t::Zero();
        vector_t rbd_state_;                               // 状态数据 [ 姿态zyx(3) | base位置(3) | 关节位置(12) | base角速度(3) | base线速度(3) | 关节速度(12) ]
        Eigen::Quaternion<scalar_t> quat_;                 // IMU四元数姿态
        matrix3_t rotation_matrix;                           // IMU姿态对应的旋转矩阵（机体坐标系->世界坐标系）
        vector3_t angular_vel_local_, linear_accel_local_; // IMU线加速度、角加速度（机体坐标系）
        vector3_t angular_vel_world_, linear_accel_world_; // IMU线加速度、角加速度（世界坐标系）

        // 传感器在机体参考系下的位置偏移（默认零）。
        vector3_t imu_local_position_ = vector3_t(0.033, -0.017, 0.051); // imu_link相较于base_link的偏移位置

        // 经过坐标变换并补偿离心加速度后的线加速度（世界坐标系）
        vector3_t linear_accel_corrected_world_ = vector3_t::Zero();

        matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;



        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pose_pub_;
        rclcpp::Publisher<control_input_msgs::msg::EulerAngle>::SharedPtr eulerAngle_pub_;

        rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr error_pub_;

        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    };

    template <typename T>
    T square(T a)
    {
        return a * a;
    }

    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T> &q)
    {
        Eigen::Matrix<SCALAR_T, 3, 1> zyx;

        SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
        zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()),
                            square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
        zyx(1) = std::asin(as);
        zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()),
                            square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
        return zyx;
    }
} // namespace legged
