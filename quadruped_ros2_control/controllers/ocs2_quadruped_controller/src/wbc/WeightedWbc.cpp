//
// Created by qiayuan on 22-12-23.
//

#include "ocs2_quadruped_controller/wbc/WeightedWbc.h"

#include <qpOASES.hpp>

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

        // Constraints
        Task constraints = formulateConstraints();
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
        Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
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
        vector_t qpSol = vector_t::Zero(getNumDecisionVars());
        const auto primalStatus = qpProblem.getPrimalSolution(qpSol.data());
        const vector_t value = A * qpSol;
        constraint_violation_ = std::max(0.0, std::max((lbA - value).maxCoeff(), (value - ubA).maxCoeff()));
        solution_valid_ = primalStatus == qpOASES::SUCCESSFUL_RETURN && qpSol.allFinite() &&
            constraint_violation_ <= 1e-5 &&
            (solver_status_ == qpOASES::SUCCESSFUL_RETURN || solver_status_ == qpOASES::RET_MAX_NWSR_REACHED);
        if (!solution_valid_) qpSol.setZero();
        return qpSol;
    }

    Task WeightedWbc::formulateConstraints()
    {
        return formulateFloatingBaseEomTask() + formulateTorqueLimitsTask() + formulateFrictionConeTask() +
            formulateNoContactMotionTask();
    }

    Task WeightedWbc::formulateWeightedTasks(const vector_t& stateDesired, const vector_t& inputDesired,
                                             scalar_t period)
    {
        return formulateSwingLegTask() * weightSwingLeg_ + formulateBaseAccelTask(stateDesired, inputDesired, period) *
            weightBaseAccel_ +
            formulateContactForceTask(inputDesired) * weightContactForce_;
    }

    void WeightedWbc::loadTasksSetting(const std::string& taskFile, bool verbose)
    {
        WbcBase::loadTasksSetting(taskFile, verbose);

        boost::property_tree::ptree pt;
        read_info(taskFile, pt);
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
