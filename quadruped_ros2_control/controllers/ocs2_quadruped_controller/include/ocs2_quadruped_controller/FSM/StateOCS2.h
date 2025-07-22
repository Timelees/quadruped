//
// Created by tlab-uav on 25-2-27.
//

#ifndef STATEOCS2_H
#define STATEOCS2_H

#include <SafetyChecker.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_quadruped_controller/control/CtrlComponent.h>
#include <ocs2_quadruped_controller/wbc/WbcBase.h>
#include <rclcpp/duration.hpp>

#include "controller_common/FSM/FSMState.h"
#include "control_input_msgs/msg/sim_base.hpp"

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

        // 将关节命令转换到lowCmd
        void command2lowCmd(CtrlInterfaces& ctrl_interfaces_,  const std::shared_ptr<CtrlComponent>& ctrl_component_);

        FSMStateName checkChange() override;

    private:

        std::shared_ptr<CtrlComponent> ctrl_component_;
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

        // Whole Body Control
        std::shared_ptr<WbcBase> wbc_;
        std::shared_ptr<SafetyChecker> safety_checker_;
        benchmark::RepeatedTimer wbc_timer_;

        double default_kp_ = 0;
        double default_kd_ = 6;
        

        vector_t optimized_state_, optimized_input_;    // MPC优化后的状态和输入

        // rclcpp::Publisher<control_input_msgs::msg::SimBase>::SharedPtr desired_base_pub_;
    };
}


#endif //STATEOCS2_H
