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
    // The admission hook runs under MRT's buffer mutex, before the policy swap.
    class StateOCS2::SwingPolicyObserver final : public MrtObserver
    {
    public:
        SwingPolicyObserver(StateOCS2& owner, std::shared_ptr<SwingTrajectoryPlanner> planner)
            : owner_(owner), planner_(std::move(planner)) {}
        void modifyBufferedSolution(const CommandData&, PrimalSolution&) override {
            buffered_ = planner_->executionSwings();
        }
        bool acceptBufferedSolution(const CommandData&, const PrimalSolution& policy) override {
            checked_ = buffered_;
            if (!active_ || !buffered_) return true; // initial stand policy has no executing update
            const double now=owner_.ctrl_component_->observation_.time;
            if (policy.timeTrajectory_.empty() || now > policy.timeTrajectory_.back()) return false;
            const auto map=planner_->certificationTerrain();
            std::shared_ptr<SwingTrajectoryPlanner::ExecutionSwings> checked;
            Eigen::Matrix<scalar_t,6,4> measured;
            bool measuredReady=false;
            for (size_t leg=0; leg<4; ++leg) {
                for (size_t i=0; i<(*buffered_)[leg].size(); ++i) {
                    const auto& swing=(*buffered_)[leg][i];
                    const double activation=swing.reference.getKnots()[0];
                    if (now<swing.start || now>=swing.end) continue;
                    const SwingTrajectoryPlanner::ExecutionSwing* previous=nullptr;
                    for (const auto& old:(*active_)[leg])
                        if (std::abs(old.start-swing.start)<1e-8 && std::abs(old.end-swing.end)<1e-8) previous=&old;
                    if (previous && previous->reference.getControlPoints().rows()==swing.reference.getControlPoints().rows() &&
                        (previous->reference.getKnots()-swing.reference.getKnots()).norm()<1e-10 &&
                        (previous->reference.getControlPoints()-swing.reference.getControlPoints()).norm()<1e-10) continue;
                    // A package generated before the last commit can still carry
                    // the original spline. Reverting an executing update is a
                    // reference change too, and must pass the same actual gate.
                    if (!previous && activation<=swing.start+1e-9) continue;
                    if (!measuredReady) { measured=owner_.wbc_->measureFeet(owner_.ctrl_component_->measured_rbd_state_); measuredReady=true; }
                    SwingTrajectoryPlanner::ActivationCheck verdict;
                    if (map) verdict=planner_->checkActivation(swing,previous,now,period_,
                        measured.col(leg).head<3>(),measured.col(leg).tail<3>(),*map);
                    else verdict.reason="activation_map_missing";
                    RCLCPP_INFO(owner_.node_->get_logger(),
                        "Swing activation accepted=%d leg=%zu activation=%.9f now=%.9f delay=%.6f p_error=%.6g v_error=%.6g reason=%s",
                        verdict.accepted,leg,activation,now,verdict.delay,verdict.positionError,verdict.velocityError,verdict.reason.c_str());
                    if (!verdict.accepted) {
                        planner_->rejectExecutionUpdate(leg,swing.end);
                        ++rejected_;
                        return false;
                    }
                    if (!checked) checked=std::make_shared<SwingTrajectoryPlanner::ExecutionSwings>(*buffered_);
                    (*checked)[leg][i].corridor=std::move(verdict.corridor);
                }
            }
            if (checked) checked_=std::move(checked);
            ++accepted_;
            return true;
        }
        void modifyActiveSolution(const CommandData&, PrimalSolution&) override {
            active_=checked_;
            planner_->setActiveExecutionSwings(active_);
            lastMap_.reset();
        }
        void refresh(double period) {
            period_=period;
            const auto map=planner_->certificationTerrain();
            if (!active_ || !map || map==lastMap_) return;
            const double now=owner_.ctrl_component_->observation_.time;
            auto refreshed=std::make_shared<SwingTrajectoryPlanner::ExecutionSwings>(*active_);
            for (auto& leg:*refreshed) for (auto& swing:leg) {
                if (now<swing.start || now>=swing.end) continue;
                swing.corridor=planner_->certifyReference(swing.reference,swing.start,swing.end,now,*map);
            }
            active_=std::move(refreshed);
            planner_->setActiveExecutionSwings(active_);
            lastMap_=map;
        }
        auto active() const { return active_; }
        size_t accepted() const { return accepted_; }
        size_t rejected() const { return rejected_; }
    private:
        StateOCS2& owner_;
        std::shared_ptr<SwingTrajectoryPlanner> planner_;
        std::shared_ptr<const SwingTrajectoryPlanner::ExecutionSwings> active_, buffered_, checked_;
        std::shared_ptr<const SwingTrajectoryPlanner::TerrainSnapshot> lastMap_;
        double period_=0.005;
        size_t accepted_=0,rejected_=0;
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
                swing_policy_observer_ = std::make_shared<SwingPolicyObserver>(*this, swing_planner_);
                ctrl_component_->mpc_mrt_interface_->addMrtObserver(swing_policy_observer_);
            }
        }

        // Safety Checker
        safety_checker_ = std::make_shared<SafetyChecker>(ctrl_component_->legged_interface_->getCentroidalModelInfo());
    }

    void StateOCS2::applySafetyFilter(double period)
    {
        wbc_->setSwingSafetyCallback({});
        wbc_->setSwingReferenceCallback({});
        if (!safety_filter_ || !safety_filter_->config().enabled) return;
        const auto snapshot = swing_policy_observer_->active();
        const double now = ctrl_component_->observation_.time;
        wbc_->setSwingReferenceCallback([snapshot,now](size_t leg,WbcBase::SwingReference& result) {
            if (!snapshot) return false;
            for (const auto& swing:(*snapshot)[leg]) {
                if (!swing.corridor.valid || now<swing.start || now>=swing.end ||
                    now<swing.reference.getKnots()[0] || now>swing.corridor.mapTiming.validUntil) continue;
                result.position=swing.reference.getPosition(now)+swing.corridor.footRadius*vector3_t::UnitZ();
                result.velocity=swing.reference.getVelocity(now);
                result.acceleration=swing.reference.getAcceleration(now);
                return true;
            }
            return false;
        });
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
                state.reference = &swing.reference; state.referenceTime=now;
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

        // Revalidate the old reference on the newest immutable synchronized map.
        // Policy, references and observer state change only after a successful gate.
        if (swing_policy_observer_) swing_policy_observer_->refresh(period.seconds());
        ctrl_component_->mpc_mrt_interface_->updatePolicy();
        if (swing_policy_observer_) swing_policy_observer_->refresh(period.seconds());

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
            ctrl_component_->observation_.time - last_diagnostics_time_ >= 0.004) {
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
            // Append-only schema: the original 113 entries retain their indices.
            for (const auto* values : {&wbc_->commandedFootAcceleration(), &wbc_->referenceFootAcceleration(),
                                       &wbc_->nominalFootAcceleration()})
                msg.data.insert(msg.data.end(), values->data(), values->data()+values->size());
            msg.data.push_back(swing_policy_observer_ ? swing_policy_observer_->accepted() : 0);
            msg.data.push_back(swing_policy_observer_ ? swing_policy_observer_->rejected() : 0);
            const auto& baselineAcceleration = wbc_->baselineFootAcceleration();
            msg.data.insert(msg.data.end(), baselineAcceleration.data(), baselineAcceleration.data() + baselineAcceleration.size());
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
