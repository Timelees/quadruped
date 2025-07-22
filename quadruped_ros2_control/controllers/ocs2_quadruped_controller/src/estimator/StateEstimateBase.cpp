//
// Created by qiayuan on 2021/11/15.
//

#include "ocs2_quadruped_controller/estimator/StateEstimateBase.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include <memory>
#include <utility>

namespace ocs2::legged_robot
{
    // 初始化状态估计模块，设置相关参数和对象
    StateEstimateBase::StateEstimateBase(CentroidalModelInfo info,
                                         CtrlInterfaces& ctrl_component,
                                         rclcpp_lifecycle::LifecycleNode::SharedPtr node)
        : ctrl_component_(ctrl_component),
          info_(std::move(info)),
          rbd_state_(vector_t::Zero(2 * info_.generalizedCoordinatesNum)), node_(std::move(node))
    {
        if (!node_->has_parameter("feet_force_threshold")) {
            node_->declare_parameter("feet_force_threshold", feet_force_threshold_);
        }
        feet_force_threshold_ = node_->get_parameter("feet_force_threshold").as_double();


        // 协方差更新
        // Declare and retrieve covariance parameters
        std::vector<double> static_covariance_orientation(9, 0.0);
        std::vector<double> static_covariance_angular_velocity(9, 0.0);
        std::vector<double> static_covariance_linear_acceleration(9, 0.0);
        if (!node_->has_parameter("static_covariance_orientation")) {
            node_->declare_parameter("static_covariance_orientation", static_covariance_orientation);
        }
        if(!node_->has_parameter("static_covariance_angular_velocity")) {
            node_->declare_parameter("static_covariance_angular_velocity", static_covariance_angular_velocity);
        }
        if(!node_->has_parameter("static_covariance_linear_acceleration")) {
            node_->declare_parameter("static_covariance_linear_acceleration", static_covariance_linear_acceleration);
        }
        // Retrieve covariance parameters
        static_covariance_orientation = node_->get_parameter("static_covariance_orientation").as_double_array();
        static_covariance_angular_velocity = node_->get_parameter("static_covariance_angular_velocity").as_double_array();
        static_covariance_linear_acceleration = node_->get_parameter("static_covariance_linear_acceleration").as_double_array();
        
        orientationCovariance_ = Eigen::Map<const matrix3_t>(static_covariance_orientation.data());
        angularVelCovariance_ = Eigen::Map<const matrix3_t>(static_covariance_angular_velocity.data());
        linearAccelCovariance_ = Eigen::Map<const matrix3_t>(static_covariance_linear_acceleration.data());
    
    }
    // 更新机器人的关节状态信息
    void StateEstimateBase::updateJointStates()
    {
        const size_t size = ctrl_component_.joint_effort_state_interface_.size();
        vector_t joint_pos(size), joint_vel(size);

        for (int i = 0; i < size; i++)
        {
            joint_pos(i) = ctrl_component_.joint_position_state_interface_[i].get().get_value();
            joint_vel(i) = ctrl_component_.joint_velocity_state_interface_[i].get().get_value();
        }

        rbd_state_.segment(6, info_.actuatedDofNum) = joint_pos;
        rbd_state_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = joint_vel;
    }
    // 更新足部的接触状态
    void StateEstimateBase::updateContact()
    {
        const size_t size = ctrl_component_.foot_force_state_interface_.size();
        for (int i = 0; i < size; i++)
        {
            contact_flag_[i] = ctrl_component_.foot_force_state_interface_[i].get().get_value() >
                feet_force_threshold_;
        }
    }
    // 更新机器人的 IMU（惯性测量单元）信息
    void StateEstimateBase::updateImu()
    {
        quat_ = {
            ctrl_component_.imu_state_interface_[0].get().get_value(),
            ctrl_component_.imu_state_interface_[1].get().get_value(),
            ctrl_component_.imu_state_interface_[2].get().get_value(),
            ctrl_component_.imu_state_interface_[3].get().get_value()
        };

        angular_vel_local_ = {
            ctrl_component_.imu_state_interface_[4].get().get_value(),
            ctrl_component_.imu_state_interface_[5].get().get_value(),
            ctrl_component_.imu_state_interface_[6].get().get_value()
        };

        linear_accel_local_ = {
            ctrl_component_.imu_state_interface_[7].get().get_value(),
            ctrl_component_.imu_state_interface_[8].get().get_value(),
            ctrl_component_.imu_state_interface_[9].get().get_value()
        };

        // orientationCovariance_ = orientationCovariance;
        // angularVelCovariance_ = angularVelCovariance;
        // linearAccelCovariance_ = linearAccelCovariance;

        // RCLCPP_INFO_STREAM(node_->get_logger(), "orientationCovariance_:\n" << orientationCovariance_);
        // RCLCPP_INFO_STREAM(node_->get_logger(), "angularVelCovariance_:\n" << angularVelCovariance_);
        // RCLCPP_INFO_STREAM(node_->get_logger(), "linearAccelCovariance_:\n" << linearAccelCovariance_);


        const vector3_t zyx = quatToZyx(quat_) - zyx_offset_;
        const vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
            zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(quatToZyx(quat_), angular_vel_local_));

        updateAngular(zyx, angularVelGlobal);
    }

    void StateEstimateBase::initPublishers()
    {
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);   // 发布里程计消息
        pose_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("pose", 10); // 发布姿态消息
    }
    // 更新机器人的角状态信息
    void StateEstimateBase::updateAngular(const vector3_t& zyx, const vector_t& angularVel)
    {
        rbd_state_.segment<3>(0) = zyx;
        rbd_state_.segment<3>(info_.generalizedCoordinatesNum) = angularVel;
    }
    // 更新线速度信息
    void StateEstimateBase::updateLinear(const vector_t& pos, const vector_t& linearVel)
    {
        rbd_state_.segment<3>(3) = pos;
        rbd_state_.segment<3>(info_.generalizedCoordinatesNum + 3) = linearVel;
    }

    void StateEstimateBase::publishMsgs(const nav_msgs::msg::Odometry& odom) const
    {
        rclcpp::Time time = odom.header.stamp;
        odom_pub_->publish(odom);

        nav_msgs::msg::Odometry pose;
        pose.header = odom.header;
        pose.pose.pose = odom.pose.pose;
        pose.pose.covariance = odom.pose.covariance;
        
        pose_pub_->publish(pose);
    }
} // namespace legged
