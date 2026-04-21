//
// Created by qiayuan on 2021/11/15.
//

#include "ocs2_quadruped_controller/estimator/StateEstimateBase.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <grid_map_ros/GridMapRosConverter.hpp>

#include <memory>
#include <utility>

namespace ocs2::legged_robot
{
    StateEstimateBase::StateEstimateBase(CentroidalModelInfo info,
                                         CtrlInterfaces &ctrl_component,
                                         rclcpp_lifecycle::LifecycleNode::SharedPtr node)
        : ctrl_component_(ctrl_component),
          info_(std::move(info)),
          rbd_state_(vector_t::Zero(2 * info_.generalizedCoordinatesNum)), node_(std::move(node)),
          joint_state_(vector_t::Zero(3 * info_.actuatedDofNum))
    {
        if (!node_->has_parameter("feet_force_threshold"))
        {
            node_->declare_parameter("feet_force_threshold", feet_force_threshold_);
        }
        feet_force_threshold_ = node_->get_parameter("feet_force_threshold").as_double();

        if (!node_->has_parameter("terrain_map_topic"))
        {
            node_->declare_parameter("terrain_map_topic", terrain_map_topic_);
        }
        if (!node_->has_parameter("terrain_height_layer"))
        {
            node_->declare_parameter("terrain_height_layer", terrain_height_layer_);
        }
        terrain_map_topic_ = node_->get_parameter("terrain_map_topic").as_string();
        terrain_height_layer_ = node_->get_parameter("terrain_height_layer").as_string();

        initTerrainMapSubscriber();
    }

    void StateEstimateBase::initTerrainMapSubscriber()
    {
        terrain_map_sub_ = node_->create_subscription<grid_map_msgs::msg::GridMap>(
            terrain_map_topic_, rclcpp::QoS(1).best_effort(),
            [this](const grid_map_msgs::msg::GridMap::SharedPtr msg)
            {
                std::lock_guard<std::mutex> lock(terrain_map_mutex_);
                grid_map::GridMapRosConverter::fromMessage(*msg, terrain_map_);
                terrain_map_received_ = true;
            });

        RCLCPP_INFO(node_->get_logger(),
                    "[StateEstimateBase] Terrain map subscribed: topic=%s, layer=%s",
                    terrain_map_topic_.c_str(), terrain_height_layer_.c_str());
    }

    scalar_t StateEstimateBase::queryTerrainHeight(scalar_t worldX, scalar_t worldY, scalar_t defaultHeight) const
    {
        std::lock_guard<std::mutex> lock(terrain_map_mutex_);
        if (!terrain_map_received_)
        {
            return defaultHeight;
        }

        const grid_map::Position position(worldX, worldY);
        if (!terrain_map_.isInside(position))
        {
            return defaultHeight;
        }

        const std::array<std::string, 4> candidateLayers = {
            terrain_height_layer_, "elevation", "elevation_inpainted", "smooth_planar"};
        for (const auto& layer : candidateLayers)
        {
            if (!terrain_map_.exists(layer))
            {
                continue;
            }

            const float height = terrain_map_.atPosition(layer, position);
            if (std::isfinite(height))
            {
                return static_cast<scalar_t>(height);
            }
        }

        return defaultHeight;
    }

    scalar_t StateEstimateBase::queryFootTerrainHeight(const vector3_t& basePosition,
                                                       const vector3_t& footPositionInWorldAligned,
                                                       scalar_t defaultHeight) const
    {
        const vector3_t footWorld = basePosition + footPositionInWorldAligned;
        return queryTerrainHeight(footWorld.x(), footWorld.y(), defaultHeight);
    }

    void StateEstimateBase::updateJointStates()
    {
        const size_t size = ctrl_component_.joint_effort_state_interface_.size();
        vector_t joint_pos(size), joint_vel(size), joint_tau(size);

        for (int i = 0; i < size; i++)
        {
            joint_pos(i) = ctrl_component_.joint_position_state_interface_[i].get().get_value();
            joint_vel(i) = ctrl_component_.joint_velocity_state_interface_[i].get().get_value();
            joint_tau(i) = ctrl_component_.joint_effort_state_interface_[i].get().get_value();
        }

        rbd_state_.segment(6, info_.actuatedDofNum) = joint_pos;                                                          // 关节位置（6-17）
        rbd_state_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = joint_vel;                        // 关节速度（24-35）

        joint_state_.segment(0, info_.actuatedDofNum) = joint_pos;
        joint_state_.segment(info_.actuatedDofNum, info_.actuatedDofNum) = joint_vel;
        joint_state_.segment(2 * info_.actuatedDofNum, info_.actuatedDofNum) = joint_tau;
    }

    void StateEstimateBase::updateContact()
    {
    
        const size_t size = ctrl_component_.foot_force_state_interface_.size();
        for (int i = 0; i < size; i++)
        {
            contact_flag_[i] = ctrl_component_.foot_force_state_interface_[i].get().get_value() >
                feet_force_threshold_;
        }
    }

    void StateEstimateBase::updateContactPhase()
    {
    }

    void StateEstimateBase::updateImu()
    {
        quat_ = {
            ctrl_component_.imu_state_interface_[0].get().get_value(),
            ctrl_component_.imu_state_interface_[1].get().get_value(),
            ctrl_component_.imu_state_interface_[2].get().get_value(),
            ctrl_component_.imu_state_interface_[3].get().get_value()};

        angular_vel_local_ = {
            ctrl_component_.imu_state_interface_[4].get().get_value(),
            ctrl_component_.imu_state_interface_[5].get().get_value(),
            ctrl_component_.imu_state_interface_[6].get().get_value()};

        linear_accel_local_ = {
            ctrl_component_.imu_state_interface_[7].get().get_value(),
            ctrl_component_.imu_state_interface_[8].get().get_value(),
            ctrl_component_.imu_state_interface_[9].get().get_value()};

        // 从硬件接口读取协方差矩阵
        for (size_t i = 0; i < 9; ++i) {
            orientationCovariance_(i) = ctrl_component_.imu_state_interface_[10 + i].get().get_value();
            angularVelCovariance_(i) = ctrl_component_.imu_state_interface_[19 + i].get().get_value();
            linearAccelCovariance_(i) = ctrl_component_.imu_state_interface_[28 + i].get().get_value();
        }

        rotation_matrix = getRotationMatrixFromZyxEulerAngles(quatToZyx(quat_)); // body->world

        const vector3_t zyx = quatToZyx(quat_) - zyx_offset_;
        const vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
            zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(quatToZyx(quat_), angular_vel_local_));

        updateAngular(zyx, angularVelGlobal);
    }

    void StateEstimateBase::initPublishers()
    {
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        pose_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("pose", 10);
    }

    void StateEstimateBase::updateAngular(const vector3_t &zyx, const vector_t &angularVel)
    {
        rbd_state_.segment<3>(0) = zyx;
        rbd_state_.segment<3>(info_.generalizedCoordinatesNum) = angularVel;
    }

    void StateEstimateBase::updateLinear(const vector_t &pos, const vector_t &linearVel)
    {
        rbd_state_.segment<3>(3) = pos;
        rbd_state_.segment<3>(info_.generalizedCoordinatesNum + 3) = linearVel;
    }

    void StateEstimateBase::publishMsgs(const nav_msgs::msg::Odometry &odom) const
    {
        rclcpp::Time time = odom.header.stamp;
        odom_pub_->publish(odom);

        nav_msgs::msg::Odometry pose;
        pose.header = odom.header;
        pose.pose = odom.pose;
        pose_pub_->publish(pose);
    }
} // namespace legged