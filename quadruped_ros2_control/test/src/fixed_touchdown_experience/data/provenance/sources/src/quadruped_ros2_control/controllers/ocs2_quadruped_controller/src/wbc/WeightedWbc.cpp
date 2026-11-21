//
// Created by qiayuan on 22-12-23.
//

#include "ocs2_quadruped_controller/wbc/WeightedWbc.h"

#include <qpOASES.hpp>
#include <Eigen/QR>
#include <algorithm>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <gz_quadruped_hardware/actuator.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2::legged_robot
{
    vector_t WeightedWbc::update(const vector_t& stateDesired, const vector_t& inputDesired,
                                 const vector_t& rbdStateMeasured, size_t mode,
                                 scalar_t period)
    {
        WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period);

        // First track the accepted swing (MPC PD when unavailable). Safety projects its
        // feasible output, keeping baseline and correction in the same space.
        Task swingTask = formulateSwingLegTask();
        const Task otherTasks = formulateBaseAccelTask(stateDesired, inputDesired, period) * weightBaseAccel_ +
            formulateContactForceTask(inputDesired) * weightContactForce_;
        const Task dynamics = formulateConstraints();
        vector_t qpSol = vector_t::Zero(getNumDecisionVars());
        const auto solve = [&](const Task& constraints) {
            const Task weighedTask = swingTask * weightSwingLeg_ + otherTasks;
            size_t numConstraints = constraints.b_.size() + constraints.f_.size();

            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
            vector_t lbA(numConstraints), ubA(numConstraints); // clang-format off
            A << constraints.a_,
                    constraints.d_;

            lbA << constraints.b_,
                    -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
            ubA << constraints.b_,
                    constraints.f_; // clang-format on

            // Cost
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H =
                weighedTask.a_.transpose() * weighedTask.a_;
            vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;

            // Solve
            auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
            qpOASES::Options options;
            options.setToMPC();
            options.printLevel = qpOASES::PL_NONE;
            options.enableEqualities = qpOASES::BT_TRUE;
            qpProblem.setOptions(options);
            // A cold start can need >20 iterations even during standing up.
            int nWsr = 100;

            solver_status_ = qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
            solver_iterations_ = nWsr;
            qpSol.setZero();
            const auto primalStatus = qpProblem.getPrimalSolution(qpSol.data());
            const vector_t value = A * qpSol;
            constraint_violation_ = std::max(0.0, std::max((lbA - value).maxCoeff(), (value - ubA).maxCoeff()));
            return primalStatus == qpOASES::SUCCESSFUL_RETURN && qpSol.allFinite() &&
                constraint_violation_ <= 1e-5 &&
                (solver_status_ == qpOASES::SUCCESSFUL_RETURN || solver_status_ == qpOASES::RET_MAX_NWSR_REACHED);
        };
        solution_valid_ = solve(dynamics);
        const vector_t originalBaseline = qpSol;
        vector_t jointPd = vector_t::Zero(info_.actuatedDofNum);
        vector_t torqueLower(info_.actuatedDofNum), torqueUpper(info_.actuatedDofNum);
        actuator_prediction_ = vector_t::Constant(info_.actuatedDofNum, std::numeric_limits<double>::quiet_NaN());
        actuator_model_valid_ = false;
        bool safetyProjection = false;
        bool baselineReachable = true;
        if (solution_valid_ && swing_safety_callback_ && actuatorSafetyModel_) {
            // Predict the nominal tracking QP's *net*
            // torque, including the hardware PD, before projecting for safety.
            const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
            jointPd = joint_kp_ * (qDesired.tail(info_.actuatedDofNum) - q_measured_.tail(info_.actuatedDofNum)) +
                joint_kd_ * (centroidal_model::getJointVelocities(inputDesired, info_) - v_measured_.tail(info_.actuatedDofNum));
            if (motor_current_.size() == info_.actuatedDofNum && motor_applied_.size() == info_.actuatedDofNum &&
                motor_current_.allFinite() && motor_applied_.allFinite()) {
                gazebo::Actuator_ motor;
                for (size_t i = 0; i < info_.actuatedDofNum; ++i) {
                    const double speed = v_measured_(6 + i);
                    const double tnLow = motor.GetTorque(-1e6, speed), tnHigh = motor.GetTorque(1e6, speed);
                    const double currentLow = motor.getActuatorT(motor_current_[i] - 3.0, motor_applied_[i]);
                    const double currentHigh = motor.getActuatorT(motor_current_[i] + 3.0, motor_applied_[i]);
                    torqueLower[i] = std::max(tnLow, currentLow);
                    torqueUpper[i] = std::min(tnHigh, currentHigh);
                    const double target = motor.GetTorque(originalBaseline.tail(info_.actuatedDofNum)[i] + jointPd[i], speed);
                    const double current = motor.getActuatorI(target, speed);
                    actuator_prediction_[i] = std::abs(current - motor_current_[i]) <= 3.0 ? target :
                        motor.getActuatorT(std::clamp(current, motor_current_[i] - 3.0, motor_current_[i] + 3.0), motor_applied_[i]);
                    baselineReachable &= std::abs(current - motor_current_[i]) <= 3.0;
                }
                // Contact-constrained forward dynamics of the baseline actuator
                // output. Preserve this body/support acceleration, not the
                // ideal WBC acceleration that omitted PD and motor saturation.
                const int nMotion = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts;
                const matrix_t forwardA = dynamics.a_.leftCols(nMotion);
                const vector_t forwardB = dynamics.b_ - dynamics.a_.rightCols(info_.actuatedDofNum) * actuator_prediction_;
                const vector_t motion = forwardA.completeOrthogonalDecomposition().solve(forwardB);
                vector_t physical = originalBaseline;
                physical.head(nMotion) = motion;
                physical.tail(info_.actuatedDofNum) = actuator_prediction_;
                actuator_model_valid_ = physical.allFinite() &&
                    (dynamics.a_ * physical - dynamics.b_).lpNorm<Eigen::Infinity>() < 1e-5 &&
                    (dynamics.d_ * physical - dynamics.f_).maxCoeff() < 1e-5 &&
                    (torqueUpper - torqueLower).minCoeff() >= 0.0;
                if (actuator_model_valid_) qpSol = std::move(physical);
            }
        }
        baseline_foot_acceleration_.setConstant(std::numeric_limits<double>::quiet_NaN());
        if (solution_valid_) {
            for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
                baseline_foot_acceleration_.col(i) = j_.middleRows<3>(3 * i) *
                    qpSol.head(info_.generalizedCoordinatesNum) + dj_.middleRows<3>(3 * i) * v_measured_;
            }
        }
        if (solution_valid_ && swing_safety_callback_) {
            swingTask = formulateSwingLegTask(&baseline_foot_acceleration_);
        }
        const bool constrained = swing_safety_constraints_.f_.size() > 0;
        const bool baselineSafe = constrained && baselineReachable &&
            (swing_safety_constraints_.f_ - swing_safety_constraints_.d_ * qpSol).minCoeff() >= -1e-8;
        if (solution_valid_ && constrained && actuatorSafetyModel_ && !actuator_model_valid_) {
            for (auto& result : safety_results_)
                if (result.active()) result.status = SafetyFilterStatus::JointInfeasible;
        } else if (solution_valid_ && constrained && !baselineSafe) {
            // First preserve the feasible baseline's body acceleration. Merely
            // adding foot rows to a weighted objective allows them to sacrifice
            // body support even when the combined QP is mathematically feasible.
            const vector_t baseline = qpSol;
            const auto baselineStatus = solver_status_;
            const auto baselineIterations = solver_iterations_;
            const auto baselineViolation = constraint_violation_;
            matrix_t body = matrix_t::Zero(6, getNumDecisionVars());
            body.leftCols(6).setIdentity();
            const Task preserveBody(body, baseline.head(6), matrix_t(), vector_t());
            const int nForces = 3 * info_.numThreeDofContacts;
            matrix_t forceBox = matrix_t::Zero(2 * nForces, getNumDecisionVars());
            forceBox.block(0, info_.generalizedCoordinatesNum, nForces, nForces).setIdentity();
            forceBox.bottomRows(nForces) = -forceBox.topRows(nForces);
            vector_t forceBounds(2 * nForces);
            forceBounds << baseline.segment(info_.generalizedCoordinatesNum, nForces),
                          -baseline.segment(info_.generalizedCoordinatesNum, nForces);
            forceBounds.array() += safetyContactForceDeviation_;
            const Task preserveSupport(matrix_t(), vector_t(), forceBox, forceBounds);
            // The reference PD+feedforward target can already differ from the
            // original MPC/WBC task. Its correction box alone does not bound
            // how far the safety solve moves the feasible baseline output.
            matrix_t footBox(2 * swingTask.a_.rows(), getNumDecisionVars());
            footBox << swingTask.a_, -swingTask.a_;
            const vector_t baselineFoot = swingTask.a_ * baseline;
            vector_t footBounds(footBox.rows());
            footBounds << baselineFoot, -baselineFoot;
            footBounds.array() += safetyAccelerationDeviation_;
            const Task preserveSwing(matrix_t(), vector_t(), footBox, footBounds);
            Task actuatorBounds(getNumDecisionVars());
            if (actuator_model_valid_) {
                matrix_t d = matrix_t::Zero(2 * info_.actuatedDofNum, getNumDecisionVars());
                d.topRightCorner(info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();
                d.bottomRows(info_.actuatedDofNum) = -d.topRows(info_.actuatedDofNum);
                vector_t bound(2 * info_.actuatedDofNum); bound << torqueUpper, -torqueLower;
                actuatorBounds = Task(matrix_t(), vector_t(), d, bound);
            }
            if (!solve(dynamics + actuatorBounds + preserveBody + preserveSupport + preserveSwing + swing_safety_constraints_)) {
                for (auto& result : safety_results_) {
                    if (result.active()) result.status = SafetyFilterStatus::JointInfeasible;
                }
                qpSol = baseline;
                solver_status_ = baselineStatus;
                solver_iterations_ = baselineIterations;
                constraint_violation_ = baselineViolation;
            } else {
                safetyProjection = true;
            }
        }
        if (solution_valid_ && constrained) {
            for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
                const auto& result = safety_results_[i];
                if (!result.active()) continue;
                const vector3_t acceleration =
                    j_.middleRows<3>(3 * i) * qpSol.head(info_.generalizedCoordinatesNum) +
                    dj_.middleRows<3>(3 * i) * v_measured_;
                safety_residuals_[i] = (result.accelerationB - result.accelerationA * acceleration).minCoeff();
            }
        }
        if (!solution_valid_) qpSol.setZero();
        commanded_foot_acceleration_.setConstant(std::numeric_limits<double>::quiet_NaN());
        if (solution_valid_) {
            for (size_t i=0;i<info_.numThreeDofContacts;++i) {
                commanded_foot_acceleration_.col(i)=j_.middleRows<3>(3*i)*qpSol.head(info_.generalizedCoordinatesNum)+
                    dj_.middleRows<3>(3*i)*v_measured_;
            }
        }
        if (solution_valid_ && actuatorSafetyModel_ && swing_safety_callback_) {
            if (safetyProjection && actuator_model_valid_) {
                actuator_prediction_ = qpSol.tail(info_.actuatedDofNum);
                // Compensate PD only for the accepted net-torque correction.
                // The unchanged baseline is never globally PD-cancelled.
                qpSol.tail(info_.actuatedDofNum) -= jointPd;
            } else {
                qpSol.tail(info_.actuatedDofNum) = originalBaseline.tail(info_.actuatedDofNum);
            }
        }
        return qpSol;
    }

    Task WeightedWbc::formulateConstraints()
    {
        return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() +
            formulateNoContactMotionTask();
    }

    void WeightedWbc::loadTasksSetting(const std::string& taskFile, bool verbose)
    {
        WbcBase::loadTasksSetting(taskFile, verbose);

        boost::property_tree::ptree pt;
        read_info(taskFile, pt);
        safetyContactForceDeviation_ = pt.get<scalar_t>("swingLegTask.safetyContactForceDeviation", 5.0);
        safetyAccelerationDeviation_ = pt.get<scalar_t>("swing_trajectory_config.safetyFilterMaxCorrection", 40.0);
        if (!std::isfinite(safetyContactForceDeviation_) || safetyContactForceDeviation_ <= 0.0) {
            throw std::invalid_argument("swingLegTask.safetyContactForceDeviation must be positive.");
        }
        if (!std::isfinite(safetyAccelerationDeviation_) || safetyAccelerationDeviation_ <= 0.0) {
            throw std::invalid_argument("swing_trajectory_config.safetyFilterMaxCorrection must be positive.");
        }
        const std::string prefix = "weight.";
        if (verbose)
        {
            std::cerr << "\n #### WBC weight:";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, weightSwingLeg_, prefix + "swingLeg", verbose);
        loadData::loadPtreeValue(pt, weightBaseAccel_, prefix + "baseAccel", verbose);
        loadData::loadPtreeValue(pt, weightContactForce_, prefix + "contactForce", verbose);
    }
} // namespace legged
