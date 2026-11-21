//
// Created by tlab-uav on 25-2-27.
//

#include "ocs2_quadruped_controller/FSM/StateOCS2.h"

#include <angles/angles.h>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <set>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_core/misc/LoadData.h>
#include <boost/property_tree/info_parser.hpp>
#include <ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h>
#include <ocs2_quadruped_controller/wbc/WeightedWbc.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_quadruped_controller/interface/constraint/PolynomialHull.h>

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
        bool acceptBufferedSolution(const CommandData& command, const PrimalSolution& policy) override {
            const auto gateBegin=std::chrono::steady_clock::now();
            checked_ = buffered_;
            if (!active_ || !buffered_) return true; // initial stand policy has no executing update
            const double now=owner_.ctrl_component_->observation_.time;
            RCLCPP_INFO(owner_.node_->get_logger(), "MRT transport release now=%.9f source=%.9f residence=%.9f",
                now, command.mpcInitObservation_.time, owner_.ctrl_component_->mpc_mrt_interface_->policyTransportResidence());
            if (policy.timeTrajectory_.empty() || now > policy.timeTrajectory_.back()) {
                ++rejected_;
                RCLCPP_INFO(owner_.node_->get_logger(), "Swing package rejected reason=policy_horizon");
                return false;
            }
            const auto map=planner_->certificationTerrain();
            std::shared_ptr<SwingTrajectoryPlanner::ExecutionSwings> checked;
            Eigen::Matrix<scalar_t,6,4> measured;
            bool measuredReady=false;
            std::array<bool,4> changed{};
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
                    if (planner_->getConfig().directReplacementTestOnly) {
                        const auto p = swing.reference.getPosition(now) + planner_->getConfig().footRadius*vector3_t::UnitZ();
                        RCLCPP_INFO(owner_.node_->get_logger(),
                            "Swing direct activation leg=%zu now=%.9f p_error=%.9g v_error=%.9g reference_jump=%.9g",
                            leg, now, (p-measured.col(leg).head<3>()).norm(),
                            (swing.reference.getVelocity(now)-measured.col(leg).tail<3>()).norm(),
                            previous ? (swing.reference.getPosition(now)-previous->reference.getPosition(now)).norm() : 0.0);
                        continue;
                    }
                    // Re-instantiate at the timestamp of this control tick, using
                    // the last valid allocated input, never old-reference acceleration.
                    const auto feedback=planner_->footFeedback();
                    SwingReplanDecision instance;
                    if (feedback && now>=feedback->time && now-feedback->time<=0.03 &&
                        now-activation<=planner_->getConfig().replanConfig.maxActivationDelay) {
                        instance=SwingOnlineReplanner(planner_->getConfig().replanConfig).instantiate(
                            swing.reference,swing.corridor,swing.start,now,swing.end,
                            measured.col(leg).head<3>()-planner_->getConfig().footRadius*vector3_t::UnitZ(),
                            measured.col(leg).tail<3>(),feedback->acceleration[leg]);
                    }
                    if (!instance.admitted()) {
                        planner_->rejectExecutionUpdate(leg,swing.end);++rejected_;
                        RCLCPP_INFO(owner_.node_->get_logger(),"Swing package rejected reason=%s",instance.reason.c_str());
                        return false;
                    }
                    auto aligned=swing;aligned.reference=std::move(instance.candidate);
                    SwingTrajectoryPlanner::ActivationCheck verdict;
                    if (map) verdict=planner_->checkActivation(aligned,previous,now,period_,
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
                    aligned.corridor=std::move(verdict.corridor);
                    (*checked)[leg][i]=std::move(aligned);
                    changed[leg] = true;
                }
            }
            if (checked) checked_=std::move(checked);
            if (owner_.safety_filter_ && std::any_of(changed.begin(), changed.end(), [](bool value) { return value; })) {
                auto& preview = *owner_.admission_wbc_;
                preview.copyControlHistoryFrom(*owner_.wbc_);
                owner_.applySafetyFilter(period_, preview, checked_, true);
                const auto& observation = owner_.ctrl_component_->observation_;
                const auto state = LinearInterpolation::interpolate(now, policy.timeTrajectory_, policy.stateTrajectory_);
                const auto input = policy.controllerPtr_->computeInput(now, observation.state);
                const size_t mode = policy.modeSchedule_.modeAtTime(now);
                preview.update(state, input, owner_.ctrl_component_->measured_rbd_state_, mode, period_);
                bool feasible = preview.solutionValid() && preview.actuatorModelValid();
                for (size_t leg=0; leg<4; ++leg) if (changed[leg]) {
                    bool liftOffTransition=false;
                    for (const auto& sw:(*checked_)[leg]) if(now>=sw.start && now<sw.end)
                        liftOffTransition=(now-sw.start)/(sw.end-sw.start)<sw.corridor.start;
                    // Lift-off is outside the configured noncontact certificate.
                    // It still needs a feasible allocated WBC solution; do not
                    // misclassify the explicit transition status as a QP failure.
                    feasible &= liftOffTransition ||
                        (preview.safetyResults()[leg].active() && preview.safetyResiduals()[leg] >= -1e-6);
                }
                if (!feasible) {
                    for (size_t leg=0; leg<4; ++leg) if (changed[leg])
                        for (const auto& swing : (*checked_)[leg]) if (now >= swing.start && now < swing.end)
                            planner_->rejectExecutionUpdate(leg, swing.end);
                    ++rejected_;
                    RCLCPP_INFO(owner_.node_->get_logger(), "Swing package rejected reason=activation_joint_wbc");
                    return false;
                }
            }
            const double gateSeconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-gateBegin).count();
            if (std::any_of(changed.begin(),changed.end(),[](bool v){return v;}) && gateSeconds>period_) {
                ++rejected_;
                RCLCPP_INFO(owner_.node_->get_logger(),"Swing package rejected reason=activation_gate_budget elapsed=%.9f budget=%.9f",gateSeconds,period_);
                return false;
            }
            // Include snapshot serialization in the control-cycle gate budget.
            std::vector<std::string> evidence;
            if (checked_) for (size_t leg=0;leg<4;++leg) if (changed[leg])
                for (const auto& sw:(*checked_)[leg]) if (now>=sw.start && now<sw.end) {
                    const SwingTrajectoryPlanner::ExecutionSwing* old=nullptr;
                    for (const auto& prev:(*active_)[leg]) if (std::abs(prev.end-sw.end)<1e-8) old=&prev;
                    std::ostringstream out;out<<std::setprecision(17);
                    out<<"{\"event\":\"commit\",\"leg\":"<<leg<<",\"now\":"<<now
                       <<",\"start\":"<<sw.start<<",\"end\":"<<sw.end<<",\"gate_seconds\":"<<gateSeconds;
                    const auto feedback=planner_->footFeedback();
                    out<<",\"acceleration_source\":\"allocated_wbc_previous_tick\",\"feedback_time\":"<<feedback->time;
                    out<<",\"boundary_position_centre\":["<<measured(0,leg)<<","<<measured(1,leg)<<","<<measured(2,leg)<<"]";
                    out<<",\"boundary_velocity\":["<<measured(3,leg)<<","<<measured(4,leg)<<","<<measured(5,leg)<<"]";
                    const auto a=feedback->acceleration[leg];
                    out<<",\"boundary_acceleration\":["<<a.x()<<","<<a.y()<<","<<a.z()<<"]";
                    const auto emit=[&](const char* name,const BSpline& ref) {
                        out<<",\""<<name<<"\":{\"knots\":[";
                        for(int k=0;k<ref.getKnots().size();++k) out<<(k?",":"")<<ref.getKnots()[k];
                        out<<"],\"controls\":[";const auto c=ref.getControlPoints();
                        for(int k=0;k<c.rows();++k) out<<(k?",":"")<<"["<<c(k,0)<<","<<c(k,1)<<","<<c(k,2)<<"]";
                        out<<"]}";
                    };
                    emit("new",sw.reference);if(old)emit("old",old->reference);
                    for(const auto& proposed:(*buffered_)[leg])if(std::abs(proposed.end-sw.end)<1e-8)emit("planned",proposed.reference);
                    out<<"}";
                    evidence.push_back(out.str());
                }
            const double finalGateSeconds=std::chrono::duration<double>(std::chrono::steady_clock::now()-gateBegin).count();
            if(!evidence.empty() && finalGateSeconds>period_) {
                ++rejected_;
                RCLCPP_INFO(owner_.node_->get_logger(),"Swing package rejected reason=activation_serialization_budget elapsed=%.9f",finalGateSeconds);
                return false;
            }
            for(const auto& item:evidence)RCLCPP_INFO(owner_.node_->get_logger(),"SWING_EVIDENCE %s",item.c_str());
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
        admission_wbc_ = std::make_shared<WeightedWbc>(ctrl_component_->legged_interface_->getPinocchioInterface(),
            ctrl_component_->legged_interface_->getCentroidalModelInfo(), *ctrl_component_->ee_kinematics_);
        admission_wbc_->loadTasksSetting(ctrl_component_->task_file_, false);
        boost::property_tree::ptree taskSettings;
        boost::property_tree::read_info(ctrl_component_->task_file_, taskSettings);
        ctrl_component_->mpc_mrt_interface_->setPolicyTransportDelay(
            taskSettings.get<double>("swing_trajectory_config.mrtTransportDelay", 0.0),
            taskSettings.get<double>("swing_trajectory_config.mrtTransportStartTime", 0.0));
        diagnostics_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("/wbc_diagnostics", 5);

        auto manager = std::dynamic_pointer_cast<PerceptiveLeggedReferenceManager>(
            ctrl_component_->legged_interface_->getReferenceManagerPtr());
        if (manager) {
            swing_planner_ = manager->getSwingTrajectoryPlanner();
            const auto& config = swing_planner_->getConfig();
            if (config.directReplacementTestOnly && config.safetyFilterConfig.enabled)
                throw std::invalid_argument("Direct replacement ablation requires the execution safety filter disabled.");
            if (config.safetyFilterConfig.enabled) {
                if (!config.useTerrainPlanning || !config.terrainQpConfig.safetyCorridorEnabled) {
                    throw std::invalid_argument("Swing safety filtering requires terrain corridors.");
                }
                safety_filter_ = std::make_unique<SwingSafetyFilter>(config.safetyFilterConfig);
            }
            if (config.terrainQpConfig.safetyCorridorEnabled) {
                swing_policy_observer_ = std::make_shared<SwingPolicyObserver>(*this, swing_planner_);
                ctrl_component_->mpc_mrt_interface_->addMrtObserver(swing_policy_observer_);
            }
        }

        // Safety Checker
        safety_checker_ = std::make_shared<SafetyChecker>(ctrl_component_->legged_interface_->getCentroidalModelInfo());
    }

    void StateOCS2::applySafetyFilter(double period, WbcBase& wbc,
            std::shared_ptr<const SwingTrajectoryPlanner::ExecutionSwings> snapshot, bool strict)
    {
        wbc.setSwingSafetyCallback({});
        wbc.setSwingReferenceCallback({});
        const double now = ctrl_component_->observation_.time;
        wbc.setSwingReferenceCallback([snapshot,now](size_t leg,WbcBase::SwingReference& result) {
            if (!snapshot) return false;
            for (const auto& swing:(*snapshot)[leg]) {
                if (!swing.corridor.valid || now<swing.start || now>=swing.end ||
                    now<swing.reference.getKnots()[0] || now>swing.corridor.mapTiming.validUntil) continue;
                result.position=swing.reference.getPosition(now)+swing.corridor.footRadius*vector3_t::UnitZ();
                result.velocity=swing.reference.getVelocity(now);
                result.acceleration=swing.reference.getAcceleration(now);
                result.onlineUpdate=swing.reference.getKnots()[0]>swing.start+1e-9;
                return true;
            }
            return false;
        });
        // Reference execution is independent of the optional acceleration filter.
        if (!safety_filter_ || !safety_filter_->config().enabled) return;
        wbc.setSwingSafetyCallback([this, snapshot, now, period, strict, &wbc](size_t leg,
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
                state.correctionCentre = wbc.nominalFootAcceleration().col(leg);
                state.reference = &swing.reference; state.referenceTime=now;
                const double phase = (now - swing.start) / (swing.end - swing.start);
                result = strict ? safety_filter_->filter(swing.corridor, state, nominal, phase, period, swing.end-now) :
                    safety_filter_->filterExecution(swing.corridor, state, nominal, phase, period, swing.end-now);
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
        last_certificate_state_.fill(-1);
        previous_execution_time_ = -1.0;
        previous_safety_active_.fill(false);
        wbc_->resetControlHistory();
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
        vector_t motorCurrent(ctrl_interfaces_.motor_current_state_interface_.size());
        vector_t appliedTorque(ctrl_interfaces_.applied_torque_state_interface_.size());
        for (Eigen::Index i=0; i<motorCurrent.size(); ++i) motorCurrent[i]=ctrl_interfaces_.motor_current_state_interface_[i].get().get_value();
        for (Eigen::Index i=0; i<appliedTorque.size(); ++i) appliedTorque[i]=ctrl_interfaces_.applied_torque_state_interface_[i].get().get_value();
        wbc_->setActuatorState(motorCurrent, appliedTorque, default_kp_, default_kd_);

        // Revalidate the old reference on the newest immutable synchronized map.
        // Policy, references and observer state change only after a successful gate.
        if (swing_policy_observer_) swing_policy_observer_->refresh(period.seconds());
        ctrl_component_->mpc_mrt_interface_->setPolicyTransportTime(ctrl_component_->observation_.time);
        if (ctrl_component_->mpc_mrt_interface_->updatePolicy()) {
            RCLCPP_DEBUG(node_->get_logger(), "MRT package delivery residence=%.6f drops=%zu",
                ctrl_component_->mpc_mrt_interface_->policyTransportResidence(),
                ctrl_component_->mpc_mrt_interface_->policyTransportDrops());
        }
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

        applySafetyFilter(period.seconds(), *wbc_, swing_policy_observer_ ? swing_policy_observer_->active() : nullptr);

        wbc_->setExperimentTime(ctrl_component_->observation_.time);
        wbc_timer_.startTimer();
        vector_t x = wbc_->update(optimized_state_, optimized_input_, ctrl_component_->measured_rbd_state_,
                                  planned_mode,
                                  period.seconds());
        wbc_timer_.endTimer();
        std::array<double,4> certificate{}, residualLower;
        residualLower.fill(std::numeric_limits<double>::quiet_NaN());
        const auto stanceNow = modeNumber2StanceLeg(planned_mode);
        const double executionTime = ctrl_component_->observation_.time;
        const double executionDt = executionTime - previous_execution_time_;
        for (size_t leg=0; leg<4; ++leg) {
            // 0 = contact/transition, 1 = remaining non-contact reference
            // certified, 2 = uncertified nominal execution (no safety claim).
            int state = stanceNow[leg] ? 0 : 2;
            std::string reason = stanceNow[leg] ? "contact" : "no_reference";
            const auto snapshot = swing_policy_observer_ ? swing_policy_observer_->active() : nullptr;
            if (!stanceNow[leg] && snapshot) for (const auto& swing : (*snapshot)[leg]) {
                if (executionTime < swing.start || executionTime >= swing.end) continue;
                const double phase = (executionTime-swing.start)/(swing.end-swing.start);
                const double transition = swing_planner_->getConfig().terrainQpConfig.corridorConfig.contactPhase;
                if (phase < transition || phase > 1.0-transition) {
                    state = 0;
                    reason = "contact_transition";
                    break;
                }
                const bool valid = swing.corridor.valid && executionTime <= swing.corridor.mapTiming.validUntil;
                state = valid ? 1 : 2;
                reason = executionTime > swing.corridor.mapTiming.validUntil ? "expired_map" : swing.corridor.failureReason;
                break;
            }
            certificate[leg] = state;
            if (state != last_certificate_state_[leg]) {
                if (state==2 || last_certificate_state_[leg]==2)
                    RCLCPP_INFO(node_->get_logger(), "Swing certificate leg=%zu now=%.9f state=%d reason=%s fallback=nominal_uncertified",
                        leg, executionTime, state, reason.c_str());
                last_certificate_state_[leg] = state;
            }
            if (safety_filter_ && previous_safety_active_[leg] && !stanceNow[leg] && executionDt > 0 &&
                executionDt <= safety_filter_->config().maxPeriod+1e-9) {
                const vector3_t dv = wbc_->footTracking().col(leg).segment<3>(3)-previous_foot_velocity_.col(leg);
                // A lower bound on the *average* model mismatch after allowing
                // both endpoint velocity measurement errors. This can falsify
                // a budget; passing samples cannot prove a deterministic bound.
                residualLower[leg] = std::max(0.0, (dv/executionDt-previous_foot_acceleration_.col(leg)).norm() -
                    2*safety_filter_->config().velocityMeasurementError/executionDt);
            }
            previous_foot_velocity_.col(leg) = wbc_->footTracking().col(leg).segment<3>(3);
            previous_safety_active_[leg] = wbc_->solutionValid() && wbc_->safetyResults()[leg].active();
        }
        previous_foot_acceleration_ = wbc_->commandedFootAcceleration();
        previous_execution_time_ = executionTime;
        if (swing_planner_ && swing_planner_->getConfig().replanConfig.enabled && wbc_->solutionValid()) {
            auto feedback = std::make_shared<SwingTrajectoryPlanner::FootFeedback>();
            if (const auto previous=swing_planner_->footFeedback()) {
                feedback->history=previous->history;
                feedback->history.push_back(static_cast<const SwingTrajectoryPlanner::FootSample&>(*previous));
                if(feedback->history.size()>8)feedback->history.erase(feedback->history.begin());
            }
            feedback->time = ctrl_component_->observation_.time;
            for (size_t leg = 0; leg < 4; ++leg) {
                feedback->velocity[leg] = wbc_->footTracking().col(leg).segment<3>(3);
                feedback->acceleration[leg] = wbc_->commandedFootAcceleration().col(leg);
            }
            swing_planner_->setFootFeedback(std::move(feedback));
        }
        const auto activeSwings = swing_policy_observer_ ? swing_policy_observer_->active() : nullptr;
        if (activeSwings && std::getenv("FIXED_TOUCHDOWN_EXPORT")) {
            static std::set<std::string> exported;
            for(size_t leg=0;leg<4;++leg) for(const auto& swing:(*activeSwings)[leg]) {
                const double now=ctrl_component_->observation_.time;
                if(now<swing.start || now>=swing.end) continue;
                const std::string key=std::to_string(leg)+":"+std::to_string(swing.start)+":"+
                    std::to_string(swing.end)+":"+std::to_string(swing.reference.getKnots()[0]);
                if(!exported.insert(key).second)continue;
                std::ofstream out(std::string(std::getenv("FIXED_TOUCHDOWN_EXPORT"))+".swings.jsonl",std::ios::app);
                out<<std::setprecision(17)<<"{\"leg\":"<<leg<<",\"now\":"<<now<<",\"start\":"<<swing.start
                   <<",\"end\":"<<swing.end<<",\"knots\":[";
                const auto& knots=swing.reference.getKnots();
                for(int k=0;k<knots.size();++k)out<<(k?",":"")<<knots[k];
                out<<"],\"controls\":[";const auto c=swing.reference.getControlPoints();
                for(int k=0;k<c.rows();++k)out<<(k?",":"")<<"["<<c(k,0)<<","<<c(k,1)<<","<<c(k,2)<<"]";
                out<<"]}\n";
            }
        }
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
                    const auto bounds = swingDerivativeBounds(swing.reference);
                    RCLCPP_INFO(node_->get_logger(),
                        "Swing applied envelope leg=%zu activation=%.9f v_max=%.9g a_max=%.9g j_max=%.9g end_error=%.9g touchdown_vz=%.9g",
                        leg, activation, bounds.row(0).maxCoeff(), bounds.row(1).maxCoeff(), bounds.row(2).maxCoeff(),
                        std::abs(swing.reference.getKnots().tail<1>()[0]-swing.end),
                        swing.reference.getVelocity(swing.end).z());
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
            msg.data.push_back(ctrl_component_->mpc_mrt_interface_->policyTransportResidence());
            msg.data.push_back(ctrl_component_->mpc_mrt_interface_->policyTransportDrops());
            msg.data.push_back(wbc_->actuatorModelValid() ? 1.0 : 0.0);
            const auto& predicted = wbc_->actuatorPrediction();
            msg.data.insert(msg.data.end(), predicted.data(), predicted.data()+predicted.size());
            msg.data.insert(msg.data.end(), appliedTorque.data(), appliedTorque.data()+appliedTorque.size());
            msg.data.insert(msg.data.end(), motorCurrent.data(), motorCurrent.data()+motorCurrent.size());
            msg.data.insert(msg.data.end(), certificate.begin(), certificate.end());
            msg.data.insert(msg.data.end(), residualLower.begin(), residualLower.end());
            msg.data.insert(msg.data.end(), optimized_state_.data(), optimized_state_.data()+12);
            const auto com = wbc_->comTracking();
            msg.data.insert(msg.data.end(), com.data(), com.data()+com.size());
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
