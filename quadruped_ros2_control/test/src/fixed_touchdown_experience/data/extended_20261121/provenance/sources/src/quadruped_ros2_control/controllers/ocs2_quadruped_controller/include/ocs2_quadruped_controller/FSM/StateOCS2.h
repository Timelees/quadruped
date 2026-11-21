//
// Created by tlab-uav on 25-2-27.
//

#ifndef STATEOCS2_H
#define STATEOCS2_H

#include <SafetyChecker.h>
#include <ocs2_quadruped_controller/interface/constraint/SwingTrajectoryPlanner.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_quadruped_controller/control/CtrlComponent.h>
#include <ocs2_quadruped_controller/interface/constraint/SwingSafetyFilter.h>
#include <ocs2_quadruped_controller/wbc/WbcBase.h>
#include <rclcpp/duration.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "controller_common/FSM/FSMState.h"

namespace ocs2
{
    class MPC_MRT_Interface;
    class MPC_BASE;
}

namespace ocs2::legged_robot
{
    class StateOCS2 final : public FSMState
    {
    public:
        StateOCS2(CtrlInterfaces& ctrl_interfaces,
                  const std::shared_ptr<CtrlComponent>& ctrl_component
        );

        void enter() override;

        void run(const rclcpp::Time& time,
                 const rclcpp::Duration& period) override;

        void exit() override;

        FSMStateName checkChange() override;

    private:

        std::shared_ptr<CtrlComponent> ctrl_component_;
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

        // Whole Body Control
        std::shared_ptr<WbcBase> wbc_;
        std::shared_ptr<WbcBase> admission_wbc_;
        std::shared_ptr<SafetyChecker> safety_checker_;
        benchmark::RepeatedTimer wbc_timer_;
        scalar_t last_visualization_time_ = 0.0;
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diagnostics_pub_;
        scalar_t last_diagnostics_time_ = 0.0;

        double default_kp_ = 0;
        double default_kd_ = 6;

        // Sections 4.1-4.3 safety filter, off by default.
        std::unique_ptr<SwingSafetyFilter> safety_filter_;
        class SwingPolicyObserver;
        std::shared_ptr<SwingPolicyObserver> swing_policy_observer_;
        std::shared_ptr<SwingTrajectoryPlanner> swing_planner_;
        void applySafetyFilter(double period, WbcBase& wbc,
            std::shared_ptr<const SwingTrajectoryPlanner::ExecutionSwings> snapshot, bool strict = false);
        std::array<double, 4> last_logged_activation_{{-1, -1, -1, -1}};
        std::array<int,4> last_certificate_state_{{-1,-1,-1,-1}};
        Eigen::Matrix<double,3,4> previous_foot_velocity_, previous_foot_acceleration_;
        std::array<bool,4> previous_safety_active_{};
        double previous_execution_time_ = -1.0;

        vector_t optimized_state_, optimized_input_;
    };
}


#endif //STATEOCS2_H
