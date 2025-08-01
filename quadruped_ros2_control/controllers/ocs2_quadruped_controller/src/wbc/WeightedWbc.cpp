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

        // Constraints 构建约束
        Task constraints = formulateConstraints();
        size_t numConstraints = constraints.b_.size() + constraints.f_.size();

        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A(numConstraints, getNumDecisionVars());
        vector_t lbA(numConstraints), ubA(numConstraints); // clang-format off
        // QP 约束矩阵A
        A << constraints.a_,
                constraints.d_;
        // 构造约束边界
        lbA << constraints.b_,          
                -qpOASES::INFTY * vector_t::Ones(constraints.f_.size());
        ubA << constraints.b_,
                constraints.f_; // clang-format on

        // Cost 
        Task weighedTask = formulateWeightedTasks(stateDesired, inputDesired, period);
        // QP成本函数 
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> H =
            weighedTask.a_.transpose() * weighedTask.a_;        // 二次项：H = a_^T * a_
        vector_t g = -weighedTask.a_.transpose() * weighedTask.b_;      // 线性项：g = -a_^T * b_

        // Solve
        auto qpProblem = qpOASES::QProblem(getNumDecisionVars(), numConstraints);
        qpOASES::Options options;
        options.setToMPC();
        options.printLevel = qpOASES::PL_LOW;
        options.enableEqualities = qpOASES::BT_TRUE;
        qpProblem.setOptions(options);
        int nWsr = 100;

        qpProblem.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWsr);
        vector_t qpSol(getNumDecisionVars());

        qpProblem.getPrimalSolution(qpSol.data());
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
