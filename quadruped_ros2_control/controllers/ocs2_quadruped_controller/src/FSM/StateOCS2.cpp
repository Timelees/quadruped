//
// Created by tlab-uav on 25-2-27.
//

#include "ocs2_quadruped_controller/FSM/StateOCS2.h"

#include <angles/angles.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_quadruped_controller/wbc/WeightedWbc.h>
#include <ocs2_sqp/SqpMpc.h>

namespace ocs2::legged_robot
{
    StateOCS2::StateOCS2(CtrlInterfaces& ctrl_interfaces,
                         const std::shared_ptr<CtrlComponent>& ctrl_component)
        : FSMState(FSMStateName::OCS2, "OCS2 State", ctrl_interfaces),
          ctrl_component_(ctrl_component),
          node_(ctrl_component->node_)
    {
        // Parameters may be declared elsewhere (e.g. by another controller/state).
        // Avoid throwing ParameterAlreadyDeclaredException during controller loading.
        if (!node_->has_parameter("default_kp")) {
            node_->declare_parameter<double>("default_kp", default_kp_);
        }
        if (!node_->has_parameter("default_kd")) {
            node_->declare_parameter<double>("default_kd", default_kd_);
        }
        default_kp_ = node_->get_parameter("default_kp").as_double();
        default_kd_ = node_->get_parameter("default_kd").as_double();

        // Whole body control
        wbc_ = std::make_shared<WeightedWbc>(ctrl_component_->legged_interface_->getPinocchioInterface(),
                                             ctrl_component_->legged_interface_->getCentroidalModelInfo(),
                                             *ctrl_component_->ee_kinematics_);
        wbc_->loadTasksSetting(ctrl_component_->task_file_, ctrl_component_->verbose_);
        diagnostics_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/wbc_diagnostics", 5);

        // Safety Checker
        safety_checker_ = std::make_shared<SafetyChecker>(ctrl_component_->legged_interface_->getCentroidalModelInfo());
    }

    void StateOCS2::enter()
    {
        ctrl_component_->init();
    }

    void StateOCS2::run(const rclcpp::Time& time,
                        const rclcpp::Duration& period)
    {
        if (ctrl_component_->mpc_running_ == false)
        {
            return;
        }

        // Load the latest MPC policy
        ctrl_component_->mpc_mrt_interface_->updatePolicy();

        // Evaluate the current policy
        size_t planned_mode = 0; // The mode that is active at the time the policy is evaluated at.
        ctrl_component_->mpc_mrt_interface_->evaluatePolicy(ctrl_component_->observation_.time,
                                                            ctrl_component_->observation_.state,
                                                            optimized_state_,
                                                            optimized_input_, planned_mode);


        // Whole body control
        ctrl_component_->observation_.input = optimized_input_;
        ctrl_interfaces_.mpc_input_.assign(optimized_input_.data(), optimized_input_.data() + optimized_input_.size());

        wbc_timer_.startTimer();
        vector_t x = wbc_->update(optimized_state_, optimized_input_, ctrl_component_->measured_rbd_state_,
                                  planned_mode,
                                  period.seconds());
        wbc_timer_.endTimer();
        // wbc计算的12个关节的扭矩，关节期望位置和期望速度
        vector_t torque = x.tail(12);
        vector_t pos_des = centroidal_model::getJointAngles(optimized_state_,
                                                            ctrl_component_->legged_interface_->
                                                                             getCentroidalModelInfo());
        vector_t vel_des = centroidal_model::getJointVelocities(optimized_input_,
                                                                ctrl_component_->legged_interface_->
                                                                getCentroidalModelInfo());

        if (diagnostics_pub_->get_subscription_count() > 0 &&
            ctrl_component_->observation_.time - last_diagnostics_time_ >= 0.01) {
            last_diagnostics_time_ = ctrl_component_->observation_.time;
            std_msgs::msg::Float64MultiArray msg;
            // [time, mode, QP status, nWSR, violation, valid, tau(12), F(12), feet(4*12)].
            msg.data = {last_diagnostics_time_, static_cast<double>(planned_mode),
                        static_cast<double>(wbc_->solverStatus()), static_cast<double>(wbc_->solverIterations()),
                        wbc_->constraintViolation(), wbc_->solutionValid() ? 1.0 : 0.0};
            msg.data.insert(msg.data.end(), torque.data(), torque.data() + 12);
            msg.data.insert(msg.data.end(), x.data() + 18, x.data() + 30);
            const auto& feet = wbc_->footTracking();
            msg.data.insert(msg.data.end(), feet.data(), feet.data() + feet.size());
            msg.data.push_back(time.seconds());
            for (const auto& sensor : ctrl_interfaces_.foot_force_state_interface_) {
                msg.data.push_back(sensor.get().get_value());
            }
            diagnostics_pub_->publish(msg);
        }

        if (!wbc_->solutionValid()) {
            RCLCPP_ERROR(node_->get_logger(), "WBC QP invalid: status=%d nWSR=%d residual=%g; entering passive",
                         wbc_->solverStatus(), wbc_->solverIterations(), wbc_->constraintViolation());
            torque.setZero();
            vel_des.setZero();
        }

        for (int i = 0; i < 12; i++)
        {
            ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value(torque(i));
            ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(pos_des(i));
            ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(vel_des(i));
            ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(default_kp_);
            ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(default_kd_);
        }

        // Visualization
        if (ctrl_component_->observation_.time - last_visualization_time_ >= 0.02) {
            last_visualization_time_ = ctrl_component_->observation_.time;
            ctrl_component_->visualizer_->update(ctrl_component_->mpc_mrt_interface_->getPolicy(),
                                                 ctrl_component_->mpc_mrt_interface_->getCommand());
        }
        
    }


    void StateOCS2::exit()
    {
    }

    FSMStateName StateOCS2::checkChange()
    {
        if (ctrl_component_->mpc_failed_ || !wbc_->solutionValid()) {
            return FSMStateName::PASSIVE;
        }
        // Safety check, if failed, stop the controller
        if (!safety_checker_->check(ctrl_component_->observation_, optimized_state_, optimized_input_))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[Legged Controller] Safety check failed, stopping the controller.");
            return FSMStateName::PASSIVE;
        }
        switch (ctrl_interfaces_.control_inputs_.command)
        {
        case 1:
            return FSMStateName::PASSIVE;
        default:
            return FSMStateName::OCS2;
        }
    }
}
