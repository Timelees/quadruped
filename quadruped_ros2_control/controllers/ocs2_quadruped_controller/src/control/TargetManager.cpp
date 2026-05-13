//
// Created by tlab-uav on 24-9-30.
//

#include "ocs2_quadruped_controller/control/TargetManager.h"

#include <angles/angles.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace ocs2::legged_robot
{
    namespace
    {
        // Hold each /cmd_vel sample for this many control cycles (~0.2 s at 500 Hz -> 100 cycles if divisor 5).
        constexpr int kCmdVelHoldCycleDivisor = 5;
    }
    TargetManager::TargetManager(CtrlInterfaces& ctrl_component,
                                 rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                                 const std::shared_ptr<ReferenceManagerInterface>& referenceManagerPtr,
                                 const std::string& task_file,
                                 const std::string& reference_file)
        : ctrl_component_(ctrl_component),
          referenceManagerPtr_(referenceManagerPtr),
          node_(std::move(node))
    {
        default_joint_state_ = vector_t::Zero(12);
        // // 从配置文件加载参数
        loadData::loadCppDataType(reference_file, "comHeight", command_height_);
        loadData::loadEigenMatrix(reference_file, "defaultJointState", default_joint_state_);
        loadData::loadCppDataType(task_file, "mpc.timeHorizon", time_to_target_);
        loadData::loadCppDataType(reference_file, "targetRotationVelocity", target_rotation_velocity_);
        loadData::loadCppDataType(reference_file, "targetDisplacementVelocity", target_displacement_velocity_);
        loadData::loadCppDataType(reference_file, "targetLateralVelocity", target_lateral_velocity_);
        twist_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                buffer_.writeFromNonRT(*msg);
                twist_count = std::max(1, ctrl_component_.frequency_ / kCmdVelHoldCycleDivisor);
                RCLCPP_DEBUG(node_->get_logger(), "cmd_vel buffered, hold cycles: %i", twist_count);
            });
    }

    void TargetManager::update(SystemObservation& observation)
    {
        // Body-frame command: linear (vx,vy,vz), yaw rate about base z (maps to state theta_base_z).
        vector_t cmdGoal = vector_t::Zero(6);

        if (buffer_.readFromRT() == nullptr || twist_count <= 0)
        {
            // Body frame: vx 用前后上限，vy 用侧向上限。
            cmdGoal[0] = ctrl_component_.control_inputs_.ly * target_displacement_velocity_;   
            cmdGoal[1] = -ctrl_component_.control_inputs_.lx * target_lateral_velocity_;
            cmdGoal[2] = ctrl_component_.control_inputs_.ry;
            cmdGoal[3] = -ctrl_component_.control_inputs_.rx * target_rotation_velocity_;
        }
        else
        {
            const geometry_msgs::msg::Twist twist = *buffer_.readFromRT();
            // 假定 linear.x/y 为机体系速度 [m/s]，按 reference 中前后/侧向最大速度限幅。
            cmdGoal[0] = std::clamp(static_cast<scalar_t>(twist.linear.x), -target_displacement_velocity_,
                                    target_displacement_velocity_);
            cmdGoal[1] = std::clamp(static_cast<scalar_t>(twist.linear.y), -target_lateral_velocity_,
                                    target_lateral_velocity_);
            cmdGoal[2] = 0;
            cmdGoal[3] = twist.angular.z;
            twist_count--;
            if (twist_count <= 0)
            {
                buffer_.reset();
            }
        }

        // Linear cmd is in body frame; rotate to world. Base euler in state: [theta_z, theta_y, theta_x] (ZYX).
        const vector_t currentPose = observation.state.segment<6>(6);
        const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
        vector_t cmd_vel_rot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdGoal.head(3);

        // 航向参考：有显式 yaw 角速度时按指令积分；否则保持已锁定的世界 yaw，使 MPC 对侧走中的偏航有恢复项。
        scalar_t yaw_target;
        if (std::abs(cmdGoal(3)) > kYawRateDeadzone)
        {
            yaw_target = currentPose(3) + cmdGoal(3) * time_to_target_;
            yaw_hold_value_ = yaw_target;
            yaw_hold_initialized_ = true;
        }
        else
        {
            if (!yaw_hold_initialized_)
            {
                yaw_hold_value_ = currentPose(3);
                yaw_hold_initialized_ = true;
            }
            yaw_target = currentPose(3) +
                angles::shortest_angular_distance(currentPose(3), yaw_hold_value_);
        }

        const vector_t targetPose = [&]
        {
            vector_t target(6); // [px, py, pz, theta_base_z, theta_base_y, theta_base_x]
            target(0) = currentPose(0) + cmd_vel_rot(0) * time_to_target_;  // px
            target(1) = currentPose(1) + cmd_vel_rot(1) * time_to_target_; // py
            target(2) = command_height_;    // pz
            target(3) = yaw_target;         // yaw
            target(4) = 0;                  // roll
            target(5) = 0;                  // pitch
            return target;
        }();

        const scalar_t targetReachingTime = observation.time + time_to_target_;
        auto trajectories =
            targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
        trajectories.stateTrajectory[0].head(3) = cmd_vel_rot;
        trajectories.stateTrajectory[1].head(3) = cmd_vel_rot;

        referenceManagerPtr_->setTargetTrajectories(std::move(trajectories));
    }
}
