//
// Created by tlab-uav on 25-2-27.
//

#include "ocs2_quadruped_controller/FSM/StateOCS2.h"

#include <angles/angles.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h>
#include <ocs2_quadruped_controller/wbc/WeightedWbc.h>
#include <ocs2_sqp/SqpMpc.h>

namespace ocs2::legged_robot
{
    // Use the same MRT buffer swap for a policy and its swing references.
    // The active snapshot is subsequently read only by the control thread.
    class StateOCS2::SwingPolicyObserver final : public MrtObserver
    {
    public:
        explicit SwingPolicyObserver(const std::shared_ptr<SwingTrajectoryPlanner>& planner) : planner_(planner) {}
        void modifyBufferedSolution(const CommandData&, PrimalSolution&) override {
            buffered_ = planner_->executionSwings();
        }
        void modifyActiveSolution(const CommandData&, PrimalSolution&) override { active_ = buffered_; }
        std::shared_ptr<const SwingTrajectoryPlanner::ExecutionSwings> active() const { return active_; }
    private:
        std::shared_ptr<SwingTrajectoryPlanner> planner_;
        std::shared_ptr<const SwingTrajectoryPlanner::ExecutionSwings> buffered_, active_;
    };

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

        auto manager = std::dynamic_pointer_cast<PerceptiveLeggedReferenceManager>(
            ctrl_component_->legged_interface_->getReferenceManagerPtr());
        if (manager) {
            swing_planner_ = manager->getSwingTrajectoryPlanner();
            const auto& config = swing_planner_->getConfig();
            if (config.safetyFilterConfig.enabled) {
                if (!config.useTerrainPlanning || !config.terrainQpConfig.safetyCorridorEnabled) {
                    throw std::invalid_argument("Swing safety filtering requires terrain corridors.");
                }
                safety_filter_ = std::make_unique<SwingSafetyFilter>(config.safetyFilterConfig);
            }
            if (config.safetyFilterConfig.enabled || config.replanConfig.enabled) {
                swing_policy_observer_ = std::make_shared<SwingPolicyObserver>(swing_planner_);
                ctrl_component_->mpc_mrt_interface_->addMrtObserver(swing_policy_observer_);
            }
        }

        // Safety Checker
        safety_checker_ = std::make_shared<SafetyChecker>(ctrl_component_->legged_interface_->getCentroidalModelInfo());
    }

    void StateOCS2::applySafetyFilter(double period)
    {
        wbc_->setSwingSafetyCallback({});
        if (!safety_filter_ || !safety_filter_->config().enabled) return;
        const auto snapshot = swing_policy_observer_->active();
        const double now = ctrl_component_->observation_.time;
        wbc_->setSwingSafetyCallback([this, snapshot, now, period](size_t leg,
                const vector3_t& position, const vector3_t& velocity, const vector3_t& nominal) {
            SafetyFilterResult result;
            result.status = SafetyFilterStatus::NoCorridor;
            if (!snapshot) return result;
            for (const auto& swing : (*snapshot)[leg]) {
                if (now < swing.start || now >= swing.end) continue;
                if (now < swing.reference.getKnots()[0] || now > swing.corridor.mapTiming.validUntil) return result;
                SwingSafetyFilter::FootState state;
                state.position = position;
                state.velocity = velocity;
                vector3_t desired = swing.reference.getPosition(now);
                desired.z() += swing.corridor.footRadius;
                state.positionError = position - desired;
                state.velocityError = velocity - swing.reference.getVelocity(now);
                state.referenceAcceleration = swing.reference.getAcceleration(now);
                result = safety_filter_->filter(swing.corridor, state, nominal,
                    (now - swing.start) / (swing.end - swing.start), period, swing.end - now);
                if (result.failed()) {
                    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                        "Swing safety filter unavailable: leg=%zu status=%d dt=%.4f h=%g psi1=%g tube=%g",
                        leg, static_cast<int>(result.status), period, result.clearance, result.psi1, result.tubeSlack);
                }
                return result;
            }
            return result;
        });
    }

    void StateOCS2::enter()
    {
        last_logged_activation_.fill(-1.0);
        last_diagnostics_time_ = 0.0;
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

        applySafetyFilter(period.seconds());

        wbc_timer_.startTimer();
        vector_t x = wbc_->update(optimized_state_, optimized_input_, ctrl_component_->measured_rbd_state_,
                                  planned_mode,
                                  period.seconds());
        wbc_timer_.endTimer();
        if (swing_planner_ && swing_planner_->getConfig().replanConfig.enabled && wbc_->solutionValid()) {
            auto feedback = std::make_shared<SwingTrajectoryPlanner::FootFeedback>();
            feedback->time = ctrl_component_->observation_.time;
            for (size_t leg = 0; leg < 4; ++leg) {
                feedback->velocity[leg] = wbc_->footTracking().col(leg).segment<3>(3);
            }
            swing_planner_->setFootFeedback(std::move(feedback));
        }
        const auto activeSwings = swing_policy_observer_ ? swing_policy_observer_->active() : nullptr;
        if (activeSwings && wbc_->solutionValid()) {
            const auto stance = modeNumber2StanceLeg(planned_mode);
            const double now = ctrl_component_->observation_.time;
            for (size_t leg = 0; leg < activeSwings->size(); ++leg) {
                if (stance[leg]) continue;
                for (const auto& swing : (*activeSwings)[leg]) {
                    const double activation = swing.reference.getKnots()[0];
                    if (now < activation || now >= swing.end || activation <= swing.start + 1e-9 ||
                        activation <= last_logged_activation_[leg]) continue;
                    const vector3_t centre = swing.reference.getPosition(now) +
                        swing.corridor.footRadius * vector3_t::UnitZ();
                    const auto foot = wbc_->footTracking().col(leg);
                    last_logged_activation_[leg] = activation;
                    RCLCPP_INFO(node_->get_logger(),
                        "Swing replan applied leg=%zu activation=%.9f now=%.9f remaining=%.6f p_error=%.6g v_error=%.6g",
                        leg, activation, now, swing.end - now, (centre - foot.head<3>()).norm(),
                        (swing.reference.getVelocity(now) - foot.segment<3>(3)).norm());
                }
            }
        }
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
            msg.data.push_back(period.seconds());
            for (size_t leg = 0; leg < wbc_->safetyResults().size(); ++leg) {
                const auto& result = wbc_->safetyResults()[leg];
                msg.data.insert(msg.data.end(), {static_cast<double>(result.status), result.clearance,
                    result.psi1, result.tubeSlack, result.correction.norm(), wbc_->safetyResiduals()[leg],
                    static_cast<double>(result.tubeStatus)});
            }
            msg.data.push_back(wbc_timer_.getLastIntervalInMilliseconds());
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
