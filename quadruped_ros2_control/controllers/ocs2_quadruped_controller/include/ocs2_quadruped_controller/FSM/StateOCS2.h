//
// Created by tlab-uav on 25-2-27.
//

#ifndef STATEOCS2_H
#define STATEOCS2_H

#include <array>
#include <memory>
#include <SafetyChecker.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <ocs2_quadruped_controller/control/CtrlComponent.h>
#include <ocs2_quadruped_controller/wbc/WbcBase.h>
#include <rclcpp/duration.hpp>

#include "log_save.h"

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


        FSMStateName checkChange() override;

    private:

        std::shared_ptr<CtrlComponent> ctrl_component_;
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;

        // Whole Body Control
        std::shared_ptr<WbcBase> wbc_;
        std::shared_ptr<SafetyChecker> safety_checker_;
        benchmark::RepeatedTimer wbc_timer_;

        double default_hip_kp = 0;
        double default_hip_kd = 6;
        double default_thigh_kp = 0;
        double default_thigh_kd = 6;
        double default_calf_kp = 0;
        double default_calf_kd = 6;

        vector_t optimized_state_, optimized_input_;    // MPC优化后的状态和输入

        // WBC 输出索引 → ctrl_interfaces 索引的映射表
        // WBC 固定顺序: FL(0-2), FR(3-5), RL(6-8), RR(9-11)
        // ctrl_interfaces 顺序来自 YAML joints 参数（可能不同）
        std::array<int, 12> wbc_to_interface_map_;

        std::unique_ptr<quadruped::logs::LogSave> logger_;
        bool is_wbc_logger_enabled_ = true;
        int wbc_log_decimation_ = 10;
        size_t wbc_log_counter_ = 0;
        std::string torque_log_header_;
        std::string position_log_header_;
        std::string velocity_log_header_;
        
        // rclcpp::Publisher<control_input_msgs::msg::SimBase>::SharedPtr desired_base_pub_;
    };
}


#endif //STATEOCS2_H
