//
// Created by qiayuan on 22-12-23.
//

#include "ocs2_quadruped_controller/wbc/WeightedWbc.h"
#include "qpOASES/MessageHandling.hpp"

#include <cmath>

#include <qpOASES.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <ocs2_core/misc/LoadData.h>

namespace ocs2::legged_robot
{
    vector_t WeightedWbc::update(const vector_t& stateDesired, const vector_t& inputDesired,
                                 const vector_t& rbdStateMeasured, size_t mode,
                                 scalar_t period, scalar_t currentTime)
    {
        WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period, currentTime);

        // Constraints 构建约束。CBF 使用 slack 软化，避免安全层瞬态冲突导致整个 WBC QP 不可行。
        Task constraints = formulateConstraints();
        Task footPlacementCbfTask = formulateFootPlacementCbfTask();
        const Eigen::Index numBaseDecisionVars = static_cast<Eigen::Index>(getNumDecisionVars());
        const Eigen::Index numCbfSlackVars = footPlacementCbfTask.f_.size();
        const Eigen::Index numDecisionVars = numBaseDecisionVars + numCbfSlackVars;
        const Eigen::Index numConstraints = constraints.b_.size() + constraints.f_.size() +
                                            footPlacementCbfTask.f_.size() + numCbfSlackVars;

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A =
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>::Zero(numConstraints, numDecisionVars);
        vector_t lbA(numConstraints), ubA(numConstraints); // clang-format off
        Eigen::Index row = 0;
        if (constraints.b_.size() > 0) {
            A.block(row, 0, constraints.a_.rows(), numBaseDecisionVars) = constraints.a_;
            lbA.segment(row, constraints.b_.size()) = constraints.b_;
            ubA.segment(row, constraints.b_.size()) = constraints.b_;
            row += constraints.b_.size();
        }
        if (constraints.f_.size() > 0) {
            A.block(row, 0, constraints.d_.rows(), numBaseDecisionVars) = constraints.d_;
            lbA.segment(row, constraints.f_.size()) = -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
            ubA.segment(row, constraints.f_.size()) = constraints.f_;
            row += constraints.f_.size();
        }
        if (footPlacementCbfTask.f_.size() > 0) {
            A.block(row, 0, footPlacementCbfTask.d_.rows(), numBaseDecisionVars) = footPlacementCbfTask.d_;
            A.block(row, numBaseDecisionVars, numCbfSlackVars, numCbfSlackVars) =
                -matrix_t::Identity(numCbfSlackVars, numCbfSlackVars);
            lbA.segment(row, numCbfSlackVars) = -qpOASES::INFTY * vector_t::Ones(numCbfSlackVars);
            ubA.segment(row, numCbfSlackVars) = footPlacementCbfTask.f_;
            row += numCbfSlackVars;

            A.block(row, numBaseDecisionVars, numCbfSlackVars, numCbfSlackVars) =
                -matrix_t::Identity(numCbfSlackVars, numCbfSlackVars);
            lbA.segment(row, numCbfSlackVars) = -qpOASES::INFTY * vector_t::Ones(numCbfSlackVars);
            ubA.segment(row, numCbfSlackVars) = vector_t::Zero(numCbfSlackVars);
        } // clang-format on

        // Cost 
        Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
        const Eigen::Index numTaskRows = weighedTask.b_.size() + numCbfSlackVars;
        matrix_t taskA = matrix_t::Zero(numTaskRows, numDecisionVars);
        vector_t taskB = vector_t::Zero(numTaskRows);
        if (weighedTask.b_.size() > 0)
        {
            taskA.block(0, 0, weighedTask.a_.rows(), numBaseDecisionVars) = weighedTask.a_;
            taskB.head(weighedTask.b_.size()) = weighedTask.b_;
        }
        if (numCbfSlackVars > 0)
        {
            const scalar_t slackCost = std::sqrt(std::max<scalar_t>(weightFootPlacementCbfSlack_, 0.0));
            taskA.block(weighedTask.b_.size(), numBaseDecisionVars, numCbfSlackVars, numCbfSlackVars) =
                slackCost * matrix_t::Identity(numCbfSlackVars, numCbfSlackVars);
        }
        // QP成本函数 
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H =
            taskA.transpose() * taskA;        // 二次项：H = a_^T * a_
        vector_t g = -taskA.transpose() * taskB;      // 线性项：g = -a_^T * b_

        // Solve
        auto qpProblem = qpOASES::QProblem(numDecisionVars, numConstraints);
        qpOASES::Options options;
        options.setToMPC();
        options.printLevel = qpOASES::PL_LOW;
        options.enableEqualities = qpOASES::BT_TRUE;
        qpProblem.setOptions(options);
        int nWsr = 100;

        qpOASES::returnValue status = qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
        if (status != qpOASES::SUCCESSFUL_RETURN)
        {
            std::cerr << "[WeightedWbc] QP initialization failed with status: " << status << std::endl;
            return vector_t::Zero(getNumDecisionVars());
        }
        vector_t qpSol(numDecisionVars);

        qpProblem.getPrimalSolution(qpSol.data());
        const vector_t baseQpSol = qpSol.head(numBaseDecisionVars);
        return baseQpSol;
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
        if (const auto footPlacementCbfSlack = pt.get_optional<scalar_t>(prefix + "footPlacementCbfSlack"))
        {
            weightFootPlacementCbfSlack_ = *footPlacementCbfSlack;
        }
        if (verbose)
        {
            std::cerr << prefix << "footPlacementCbfSlack: " << weightFootPlacementCbfSlack_ << std::endl;
        }
    }
} // namespace legged
