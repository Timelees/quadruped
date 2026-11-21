//
// Created by tlab-uav on 24-9-30.
//

#include "ocs2_quadruped_controller/control/TargetManager.h"

#include <angles/angles.h>
#include "ocs2_quadruped_controller/control/CommandRamp.h"
#include <boost/property_tree/info_parser.hpp>
#include <stdexcept>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
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
        constexpr scalar_t kMaxPitchAngle = 15.0 * 3.14159265358979323846 / 180.0;
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
        boost::property_tree::ptree settings;
        boost::property_tree::read_info(reference_file, settings);
        linear_acceleration_ = settings.get<scalar_t>("commandLinearAcceleration", linear_acceleration_);
        yaw_acceleration_ = settings.get<scalar_t>("commandYawAcceleration", yaw_acceleration_);
        if (!std::isfinite(linear_acceleration_) || linear_acceleration_ <= 0 ||
            !std::isfinite(yaw_acceleration_) || yaw_acceleration_ <= 0) {
            throw std::invalid_argument("Command acceleration limits must be finite and positive.");
        }
        twist_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            {
                buffer_.writeFromNonRT(*msg);
                twist_count = std::max(1, ctrl_component_.frequency_ / kCmdVelHoldCycleDivisor);
                RCLCPP_DEBUG(node_->get_logger(), "cmd_vel buffered, hold cycles: %i", twist_count);
            });
    }

    TargetTrajectories TargetManager::targetPoseToTargetTrajectories(const vector_t& targetPose,
                                                                     const SystemObservation& observation,
                                                                     const scalar_t& targetReachingTime)
    {
        vector_t currentPose = observation.state.segment<6>(6);
        currentPose(2) = command_height_;
        const scalar_t trajectoryDuration = targetReachingTime - observation.time;
        const scalar_t pitchResponseDuration = std::min(kPitchResponseTime, trajectoryDuration * scalar_t(0.5));
        const scalar_t pitchReachingTime = observation.time + pitchResponseDuration;
        const scalar_t alpha = pitchResponseDuration / trajectoryDuration;

        vector_t pitchReachedPose = currentPose + alpha * (targetPose - currentPose);
        pitchReachedPose(4) = targetPose(4);

        const scalar_array_t timeTrajectory{observation.time, pitchReachingTime, targetReachingTime};
        vector_array_t stateTrajectory(3, vector_t::Zero(observation.state.size()));
        stateTrajectory[0] << vector_t::Zero(6), currentPose, default_joint_state_;
        stateTrajectory[1] << vector_t::Zero(6), pitchReachedPose, default_joint_state_;
        stateTrajectory[2] << vector_t::Zero(6), targetPose, default_joint_state_;

        // desired input trajectory (just right dimensions, they are not used)
        const vector_array_t inputTrajectory(3, vector_t::Zero(observation.input.size()));

        return {timeTrajectory, stateTrajectory, inputTrajectory};
    }

    void TargetManager::update(SystemObservation& observation)
    {
        // Body-frame command: linear (vx,vy,vz), yaw rate about base z (maps to state theta_base_z).
        vector_t cmdGoal = vector_t::Zero(6);
        scalar_t pitch_target = 0.0;

        if (buffer_.readFromRT() == nullptr || twist_count <= 0)
        {
            // Body frame: vx 用前后上限，vy 用侧向上限。
            cmdGoal[0] = ctrl_component_.control_inputs_.ly * target_displacement_velocity_;
            cmdGoal[1] = -ctrl_component_.control_inputs_.lx * target_lateral_velocity_;
            cmdGoal[3] = -ctrl_component_.control_inputs_.rx * target_rotation_velocity_;
            pitch_target = ctrl_component_.control_inputs_.ry * kMaxPitchAngle;
        }
        else
        {
            const geometry_msgs::msg::Twist twist = *buffer_.readFromRT();
            // 假定 linear.x/y 为机体系速度 [m/s]，按 reference 中前后/侧向最大速度限幅。
            cmdGoal[0] = twist.linear.x;
            cmdGoal[1] = twist.linear.y;
            cmdGoal[2] = 0;
            cmdGoal[3] = twist.angular.z;
            twist_count--;
            if (twist_count <= 0)
            {
                buffer_.reset();
            }
        }

        if (!use_ramped_reference_) {
            // ===================================================================
            // Original master-branch path: full ZYX rotation, 3-point reference.
            // Restored for blind-walking and non-perceptive modes.
            // ===================================================================
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
                had_yaw_rate_command_ = true;
            }
            else
            {
                if (had_yaw_rate_command_ || !yaw_hold_initialized_)
                {
                    yaw_hold_value_ = currentPose(3);
                    yaw_hold_initialized_ = true;
                    had_yaw_rate_command_ = false;
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
                target(4) = pitch_target;       // pitch
                target(5) = 0;                  // roll
                return target;
            }();

            const scalar_t targetReachingTime = observation.time + time_to_target_;
            auto trajectories =
                targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
            for (auto& state : trajectories.stateTrajectory)
            {
                state.head(3) = cmd_vel_rot;
            }

            referenceManagerPtr_->setTargetTrajectories(std::move(trajectories));
            return;
        }

        // ===================================================================
        // Ramped reference path: yaw-only rotation, bounded acceleration,
        // dense trajectory. Used by perceptive mode for smoother command
        // following with terrain-aware planning.
        // ===================================================================
        const vector_t currentPose = observation.state.segment<6>(6);
        const Eigen::Matrix<scalar_t, 3, 1> zyx(currentPose(3), 0.0, 0.0);
        vector_t cmd_vel_rot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdGoal.head(3);

        Eigen::Vector4d requested;
        requested << cmd_vel_rot, cmdGoal(3);
        const scalar_t dt = observation.time - previous_command_time_;
        if (previous_command_time_ < 0 || dt < 0 || dt > 0.5) {
            filtered_command_.setZero();
            yaw_hold_initialized_ = false;
            had_yaw_rate_command_ = false;
            position_hold_initialized_ = false;
            had_translation_command_ = false;
        } else {
            for (int axis = 0; axis < 4; ++axis) {
                filtered_command_[axis] = sampleCommandRamp(filtered_command_[axis], requested[axis],
                    axis == 3 ? yaw_acceleration_ : linear_acceleration_, dt).velocity;
            }
        }
        previous_command_time_ = observation.time;

        // Hold the braking endpoint in world yaw; do not snap heading back to
        // the pose measured at key release while a turn is still decelerating.
        const bool turning = std::abs(requested[3]) > kYawRateDeadzone ||
                             std::abs(filtered_command_[3]) > 1e-6;
        if (turning || !yaw_hold_initialized_ || had_yaw_rate_command_) {
            yaw_hold_value_ = currentPose(3) +
                sampleCommandRamp(filtered_command_[3], requested[3], yaw_acceleration_, time_to_target_).displacement;
            yaw_hold_initialized_ = true;
        }
        had_yaw_rate_command_ = turning;

        const bool translating = requested.head<2>().norm() > 1e-6 ||
                                 filtered_command_.head<2>().norm() > 1e-6;
        // Stand-up initializes the estimator's world translation after the
        // command manager is created. Never latch the pre-initialization pose.
        // Measured all-foot contact can occur briefly in trot. It must not
        // reset a latched world target while the robot is drifting.
        const auto& modeSchedule = referenceManagerPtr_->getModeSchedule();
        const bool plannedStand = !modeSchedule.modeSequence.empty() &&
            std::all_of(modeSchedule.modeSequence.begin(), modeSchedule.modeSequence.end(),
                [](size_t mode) { return mode == STANCE; });
        const bool initializingStance = (ctrl_component_.control_inputs_.command == 2 || plannedStand) && !translating;
        if (translating || initializingStance || !position_hold_initialized_ || had_translation_command_) {
            for (int axis = 0; axis < 2; ++axis)
                position_hold_value_[axis] = currentPose(axis) + sampleCommandRamp(
                    filtered_command_[axis], requested[axis], linear_acceleration_, time_to_target_).displacement;
            position_hold_initialized_ = true;
        }
        had_translation_command_ = translating;

        // Include every velocity breakpoint: interpolated pose and momentum
        // references describe the same bounded braking motion over the horizon.
        scalar_array_t times{0.0, std::min(kPitchResponseTime, time_to_target_ * 0.5), time_to_target_};
        for (int axis = 0; axis < 4; ++axis) {
            const double duration = std::abs(requested[axis] - filtered_command_[axis]) /
                                    (axis == 3 ? yaw_acceleration_ : linear_acceleration_);
            if (duration > 1e-6 && duration < time_to_target_) { times.push_back(duration); }
        }
        for (double t = 0.05; t < time_to_target_; t += 0.05) { times.push_back(t); }
        std::sort(times.begin(), times.end());
        times.erase(std::unique(times.begin(), times.end(), [](double a, double b) {
            return std::abs(a - b) < 1e-9;
        }), times.end());
        TargetTrajectories trajectories;
        for (const double t : times) {
            vector_t state = vector_t::Zero(observation.state.size());
            state.segment<6>(6) = currentPose;
            state(8) = command_height_;
            state(10) = currentPose(4) + std::min(1.0, t / kPitchResponseTime) * (pitch_target - currentPose(4));
            state(11) = 0;
            state.tail(12) = default_joint_state_;
            for (int axis = 0; axis < 3; ++axis) {
                const auto sample = sampleCommandRamp(filtered_command_[axis], requested[axis], linear_acceleration_, t);
                state(axis) = sample.velocity;
                if (axis < 2) {
                    state(6 + axis) = translating ? currentPose(axis) + sample.displacement : position_hold_value_[axis];
                }
            }
            state(9) = turning ? currentPose(3) +
                sampleCommandRamp(filtered_command_[3], requested[3], yaw_acceleration_, t).displacement :
                currentPose(3) + angles::shortest_angular_distance(currentPose(3), yaw_hold_value_);
            trajectories.timeTrajectory.push_back(observation.time + t);
            trajectories.stateTrajectory.push_back(std::move(state));
            trajectories.inputTrajectory.push_back(vector_t::Zero(observation.input.size()));
        }
        referenceManagerPtr_->setTargetTrajectories(std::move(trajectories));
    }
}
