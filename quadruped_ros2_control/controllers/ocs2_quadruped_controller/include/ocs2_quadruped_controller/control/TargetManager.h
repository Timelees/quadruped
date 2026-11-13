//
// Created by tlab-uav on 24-9-30.
//

#ifndef TARGETMANAGER_H
#define TARGETMANAGER_H


#include <algorithm>
#include <memory>
#include <controller_common/CtrlInterfaces.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <realtime_tools/realtime_tools/realtime_buffer.hpp>

namespace ocs2::legged_robot
{
    class TargetManager
    {
    public:
        TargetManager(CtrlInterfaces& ctrl_component,
                      rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                      const std::shared_ptr<ReferenceManagerInterface>& referenceManagerPtr,
                      const std::string& task_file,
                      const std::string& reference_file);

        ~TargetManager() = default;

        void update(SystemObservation& observation);

    private:
        CtrlInterfaces& ctrl_component_;
        std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;

        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
        realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> buffer_;
        int twist_count = 0;

        vector_t default_joint_state_{};
        scalar_t command_height_{};
        scalar_t time_to_target_{};
        scalar_t target_displacement_velocity_{};
        scalar_t target_lateral_velocity_{};
        scalar_t target_rotation_velocity_{};

        Eigen::Vector4d filtered_command_ = Eigen::Vector4d::Zero();
        scalar_t previous_command_time_ = -1.0;
        // Bounded braking completes roughly within one trot swing at normal speed.
        scalar_t linear_acceleration_ = 0.6; // m/s^2
        scalar_t yaw_acceleration_ = 1.5; // rad/s^2

        // When yaw rate command is ~0, hold a fixed world yaw so MPC regulates heading (avoids passive drift in crab walk).
        bool yaw_hold_initialized_{false};
        bool had_yaw_rate_command_{false};
        scalar_t yaw_hold_value_{0};
        static constexpr scalar_t kPitchResponseTime = 0.25; // [s]
        static constexpr scalar_t kYawRateDeadzone = 0.02; // [rad/s], below = treat as no yaw command
    };
}

#endif //TARGETMANAGER_H
