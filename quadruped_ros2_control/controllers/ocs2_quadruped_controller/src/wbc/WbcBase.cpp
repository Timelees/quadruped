//
// Created by qiayuan on 2022/7/1.
//

#include "ocs2_quadruped_controller/wbc/WbcBase.h"

#include <utility>
#include <algorithm>
#include <vector>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>

#include <boost/property_tree/info_parser.hpp>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>

#include "ocs2_quadruped_controller/perceptive/common/FootPlacementSafetySet.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

namespace ocs2::legged_robot
{
    namespace
    {
        scalar_t clampSymmetric(scalar_t value, scalar_t limit)
        {
            if (limit <= 0.0)
            {
                return 0.0;
            }
            return std::clamp(value, -limit, limit);
        }
    }

    WbcBase::WbcBase(const PinocchioInterface& pinocchioInterface, CentroidalModelInfo info,
                     const PinocchioEndEffectorKinematics& eeKinematics,
                     std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr)
        : pinocchio_interface_measured_(pinocchioInterface),
          pinocchio_interface_desired_(pinocchioInterface),
          info_(std::move(info)),
          ee_kinematics_(eeKinematics.clone()),
          mapping_(info_),
          referenceManagerPtr_(std::move(referenceManagerPtr)),
          input_last_(vector_t::Zero(info_.inputDim))
    {
        num_decision_vars_ = info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts + info_.actuatedDofNum;
        q_measured_ = vector_t(info_.generalizedCoordinatesNum);
        v_measured_ = vector_t(info_.generalizedCoordinatesNum);
    }

    vector_t WbcBase::update(const vector_t& stateDesired, const vector_t& inputDesired,
                             const vector_t& rbdStateMeasured, size_t mode,
                             scalar_t /*period*/, scalar_t currentTime)
    {
        contact_flag_ = modeNumber2StanceLeg(mode);
        currentTime_ = currentTime;
        num_contacts_ = 0;
        for (const bool flag : contact_flag_)
        {
            if (flag)
            {
                num_contacts_++;
            }
        }

        updateMeasured(rbdStateMeasured);
        updateDesired(stateDesired, inputDesired);

        return {};
    }

    void WbcBase::updateMeasured(const vector_t& rbdStateMeasured)
    {
        q_measured_.head<3>() = rbdStateMeasured.segment<3>(3);     // 基座位置
        q_measured_.segment<3>(3) = rbdStateMeasured.head<3>();     // 基座姿态
        q_measured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);     // 关节角度
        v_measured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);       // 基座线速度
        v_measured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            q_measured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));   // 基座角速度
        v_measured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(
            info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);     // 关节速度

        const auto& model = pinocchio_interface_measured_.getModel();
        auto& data = pinocchio_interface_measured_.getData();

        // For floating base EoM task
        forwardKinematics(model, data, q_measured_, v_measured_);
        computeJointJacobians(model, data);
        updateFramePlacements(model, data);
        crba(model, data, q_measured_);
        data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
        nonLinearEffects(model, data, q_measured_, v_measured_);
        
        // 足端雅可比
        j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
            jac.setZero(6, info_.generalizedCoordinatesNum);
            getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED,
                             jac);
            // 取前 3 行（线速度部分）
            j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
        }

        // For not contact motion task
        computeJointJacobiansTimeVariation(model, data, q_measured_, v_measured_);
        dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
            jac.setZero(6, info_.generalizedCoordinatesNum);
            getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i],
                                          pinocchio::LOCAL_WORLD_ALIGNED, jac);
            dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
        }
    }

    void WbcBase::updateDesired(const vector_t& stateDesired, const vector_t& inputDesired)
    {
        const auto& model = pinocchio_interface_desired_.getModel();
        auto& data = pinocchio_interface_desired_.getData();

        mapping_.setPinocchioInterface(pinocchio_interface_desired_);
        // stateDesired 映射为 Pinocchio 广义坐标（qDesired）
        const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
        forwardKinematics(model, data, qDesired);
        computeJointJacobians(model, data, qDesired);
        updateFramePlacements(model, data);
        updateCentroidalDynamics(pinocchio_interface_desired_, info_, qDesired);
        // 映射期望速度（vDesired），更新运动学
        const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);
        forwardKinematics(model, data, qDesired, vDesired);
    }

    Task WbcBase::formulateFloatingBaseEomTask()
    {
        const auto& data = pinocchio_interface_measured_.getData();
        // 选择矩阵  s（12 × 18）选择关节力矩，忽略基座自由度
        matrix_t s(info_.actuatedDofNum, info_.generalizedCoordinatesNum);
        s.block(0, 0, info_.actuatedDofNum, 6).setZero();
        s.block(0, 6, info_.actuatedDofNum, info_.actuatedDofNum).setIdentity();
        // 等式约束
        matrix_t a = (matrix_t(info_.generalizedCoordinatesNum, num_decision_vars_) << data.M, -j_.transpose(), -s.
            transpose()).finished();
        vector_t b = -data.nle;

        return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateTorqueLimitsTask()
    {
        // 约束力矩
        matrix_t d(2 * info_.actuatedDofNum, num_decision_vars_);
        d.setZero();
        matrix_t i = matrix_t::Identity(info_.actuatedDofNum, info_.actuatedDofNum);
        d.block(0, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts, info_.actuatedDofNum,
                info_.actuatedDofNum) = i;
        d.block(info_.actuatedDofNum, info_.generalizedCoordinatesNum + 3 * info_.numThreeDofContacts,
                info_.actuatedDofNum,
                info_.actuatedDofNum) = -i;
        vector_t f(2 * info_.actuatedDofNum);   // 约束边界
        for (size_t l = 0; l < 2 * info_.actuatedDofNum / 3; ++l)
        {
            f.segment<3>(3 * l) = torque_limits_;
        }

        return {matrix_t(), vector_t(), d, f};
    }

    Task WbcBase::formulateNoContactMotionTask()
    {
        matrix_t a(3 * num_contacts_, num_decision_vars_);
        vector_t b(a.rows());
        a.setZero();
        b.setZero();
        size_t j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; i++)
        {
            if (contact_flag_[i])
            {
                a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(
                    3 * i, 0, 3, info_.generalizedCoordinatesNum);
                b.segment(3 * j, 3) = -dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * v_measured_;
                j++;
            }
        }

        return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateFrictionConeTask()
    {
        // 非接触足零力
        matrix_t a(3 * (info_.numThreeDofContacts - num_contacts_), num_decision_vars_);
        a.setZero();
        size_t j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            if (!contact_flag_[i])
            {
                a.block(3 * j++, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
            }
        }
        vector_t b(a.rows());
        b.setZero();
        // 摩擦椎约束
        matrix_t frictionPyramic(5, 3); // clang-format off
        frictionPyramic << 0, 0, -1,
                1, 0, -friction_coeff_,
                -1, 0, -friction_coeff_,
                0, 1, -friction_coeff_,
                0, -1, -friction_coeff_; // clang-format on

        matrix_t d(5 * num_contacts_ + 3 * (info_.numThreeDofContacts - num_contacts_), num_decision_vars_);
        d.setZero();
        j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            if (contact_flag_[i])
            {
                d.block(5 * j++, info_.generalizedCoordinatesNum + 3 * i, 5, 3) = frictionPyramic;
            }
        }
        vector_t f = Eigen::VectorXd::Zero(d.rows());

        return {a, b, d, f};
    }

    Task WbcBase::formulateBaseAccelTask(const vector_t& stateDesired, const vector_t& inputDesired, scalar_t period)
    {
        matrix_t a(6, num_decision_vars_);
        a.setZero();
        a.block(0, 0, 6, 6) = matrix_t::Identity(6, 6);

        vector_t jointAccel = centroidal_model::getJointVelocities(inputDesired - input_last_, info_) / period;
        input_last_ = inputDesired;
        mapping_.setPinocchioInterface(pinocchio_interface_desired_);

        const auto& model = pinocchio_interface_desired_.getModel();
        auto& data = pinocchio_interface_desired_.getData();
        const auto qDesired = mapping_.getPinocchioJointPosition(stateDesired);
        const vector_t vDesired = mapping_.getPinocchioJointVelocity(stateDesired, inputDesired);

        const auto& A = getCentroidalMomentumMatrix(pinocchio_interface_desired_);
        const Matrix6 Ab = A.template leftCols<6>();
        const auto AbInv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);
        const auto Aj = A.rightCols(info_.actuatedDofNum);
        const auto ADot = dccrba(model, data, qDesired, vDesired);
        Vector6 centroidalMomentumRate = info_.robotMass * getNormalizedCentroidalMomentumRate(
            pinocchio_interface_desired_, info_, inputDesired);
        centroidalMomentumRate.noalias() -= ADot * vDesired;
        centroidalMomentumRate.noalias() -= Aj * jointAccel;

        Vector6 b = AbInv * centroidalMomentumRate;

        return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateSwingLegTask()
    {
        ee_kinematics_->setPinocchioInterface(pinocchio_interface_measured_);
        std::vector<vector3_t> posMeasured = ee_kinematics_->getPosition(vector_t());
        std::vector<vector3_t> velMeasured = ee_kinematics_->getVelocity(vector_t(), vector_t());
        ee_kinematics_->setPinocchioInterface(pinocchio_interface_desired_);
        std::vector<vector3_t> posDesired = ee_kinematics_->getPosition(vector_t());
        std::vector<vector3_t> velDesired = ee_kinematics_->getVelocity(vector_t(), vector_t());

        matrix_t a(3 * (info_.numThreeDofContacts - num_contacts_), num_decision_vars_);
        vector_t b(a.rows());
        a.setZero();
        b.setZero();
        size_t j = 0;
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            if (!contact_flag_[i])
            {
                vector3_t accel = swing_kp_ * (posDesired[i] - posMeasured[i]) + swing_kd_ * (
                    velDesired[i] - velMeasured[i]);
                a.block(3 * j, 0, 3, info_.generalizedCoordinatesNum) = j_.block(
                    3 * i, 0, 3, info_.generalizedCoordinatesNum);
                b.segment(3 * j, 3) = accel - dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) * v_measured_;
                j++;
            }
        }

        return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateContactForceTask(const vector_t& inputDesired) const
    {
        matrix_t a(3 * info_.numThreeDofContacts, num_decision_vars_);
        vector_t b(a.rows());
        a.setZero();

        for (size_t i = 0; i < info_.numThreeDofContacts; ++i)
        {
            a.block(3 * i, info_.generalizedCoordinatesNum + 3 * i, 3, 3) = matrix_t::Identity(3, 3);
        }
        b = inputDesired.head(a.rows());

        return {a, b, matrix_t(), vector_t()};
    }

    Task WbcBase::formulateFootPlacementCbfTask()
    {
        const auto* perceptiveRefMgr = dynamic_cast<const PerceptiveLeggedReferenceManager*>(referenceManagerPtr_.get());
        if (perceptiveRefMgr == nullptr)
        {
            return {matrix_t(), vector_t(), matrix_t(), vector_t()};
        }

        const auto& selector = *perceptiveRefMgr->getConvexRegionSelectorPtr();
        const scalar_t activationProgress = foot_cbf_activation_progress_ >= 0.0
                                                ? foot_cbf_activation_progress_
                                                : perceptiveRefMgr->getFootPlacementRelaxationConfig().swingActivationProgress;
        const scalar_t dt = std::max<scalar_t>(foot_cbf_relaxation_derivative_dt_, 1e-6);

        ee_kinematics_->setPinocchioInterface(pinocchio_interface_measured_);
        const std::vector<vector3_t> posMeasured = ee_kinematics_->getPosition(vector_t());
        const std::vector<vector3_t> velMeasured = ee_kinematics_->getVelocity(vector_t(), vector_t());

        std::vector<matrix_t> dBlocks;
        std::vector<vector_t> fBlocks;

        for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg)
        {
            const auto safetyPolygon = selector.getConvexPolygon(leg, currentTime_);
            const bool hasPolygon = safetyPolygon.size() >= 3;
            const bool isContact = contact_flag_[leg];
            const scalar_t swingProgress = perceptiveRefMgr->getSwingTrajectoryPlanner()->getSwingPhaseProgress(leg, currentTime_);
            const bool isLateSwing = swingProgress >= 0.0 && swingProgress >= activationProgress;
            if (!(hasPolygon && !isContact && isLateSwing))
            {
                continue;
            }

            const auto projection = selector.getProjection(leg, currentTime_);
            if (projection.regionPtr == nullptr)
            {
                continue;
            }

            const auto halfSpace = foot_placement::buildWorldHalfSpace(
                safetyPolygon, projection.regionPtr->transformPlaneToWorld);
            if (halfSpace.a.rows() == 0)
            {
                continue;
            }

            const matrix_t& A = halfSpace.a;
            const vector_t& b = halfSpace.b;
            const size_t numRows = static_cast<size_t>(A.rows());

            const vector3_t& position = posMeasured[leg];
            const vector3_t& velocity = velMeasured[leg];
            const matrix_t J = j_.block(3 * leg, 0, 3, info_.generalizedCoordinatesNum);
            const matrix_t dJ = dj_.block(3 * leg, 0, 3, info_.generalizedCoordinatesNum);
            const vector3_t dJv = dJ * v_measured_;

            const scalar_t relaxation = perceptiveRefMgr->getFootPlacementRelaxation(leg, currentTime_);
            const scalar_t relaxationNext = perceptiveRefMgr->getFootPlacementRelaxation(leg, currentTime_ + dt);
            const scalar_t relaxationNext2 = perceptiveRefMgr->getFootPlacementRelaxation(leg, currentTime_ + 2.0 * dt);
            const scalar_t relaxationRate = clampSymmetric((relaxationNext - relaxation) / dt,
                                                           foot_cbf_max_relaxation_rate_);
            const scalar_t relaxationAccel = clampSymmetric(
                (relaxationNext2 - 2.0 * relaxationNext + relaxation) / (dt * dt),
                foot_cbf_max_relaxation_accel_);

            vector_t relaxationOffset = vector_t::Zero(numRows);
            vector_t relaxationRateOffset = vector_t::Zero(numRows);
            vector_t relaxationAccelOffset = vector_t::Zero(numRows);
            for (int row = 0; row < A.rows(); ++row)
            {
                const scalar_t rowNorm = A.row(row).norm();
                relaxationOffset(row) = relaxation * rowNorm;
                relaxationRateOffset(row) = relaxationRate * rowNorm;
                relaxationAccelOffset(row) = relaxationAccel * rowNorm;
            }

            const vector_t h = A * position + b + relaxationOffset;
            const vector_t hDot = A * velocity + relaxationRateOffset;

            matrix_t dLeg = matrix_t::Zero(numRows, num_decision_vars_);
            dLeg.block(0, 0, numRows, info_.generalizedCoordinatesNum) = -A * J;
            vector_t fLeg = A * dJv + relaxationAccelOffset +
                            (foot_cbf_gamma_ + foot_cbf_xi_) * hDot +
                            foot_cbf_gamma_ * foot_cbf_xi_ * h;

            dBlocks.push_back(std::move(dLeg));
            fBlocks.push_back(std::move(fLeg));
        }

        if (dBlocks.empty())
        {
            return {matrix_t(), vector_t(), matrix_t(), vector_t()};
        }

        matrix_t d = dBlocks.front();
        vector_t f = fBlocks.front();
        for (size_t index = 1; index < dBlocks.size(); ++index)
        {
            d = Task::concatenateMatrices(d, dBlocks[index]);
            f = Task::concatenateVectors(f, fBlocks[index]);
        }

        return {matrix_t(), vector_t(), d, f};
    }

    void WbcBase::loadTasksSetting(const std::string& taskFile, const bool verbose)
    {
        // Load task file
        torque_limits_ = vector_t(info_.actuatedDofNum / 4);
        loadData::loadEigenMatrix(taskFile, "torqueLimitsTask", torque_limits_);
        if (verbose)
        {
            std::cerr << "\n #### Torque Limits Task:";
            std::cerr << "\n #### =============================================================================\n";
            std::cerr << "\n #### Hip_joint Thigh_joint Calf_joint: " << torque_limits_.transpose() << "\n";
            std::cerr << " #### =============================================================================\n";
        }
        boost::property_tree::ptree pt;
        read_info(taskFile, pt);
        std::string prefix = "frictionConeTask.";
        if (verbose)
        {
            std::cerr << "\n #### Friction Cone Task:";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, friction_coeff_, prefix + "frictionCoefficient", verbose);
        if (verbose)
        {
            std::cerr << " #### =============================================================================\n";
        }
        prefix = "swingLegTask.";
        if (verbose)
        {
            std::cerr << "\n #### Swing Leg Task:";
            std::cerr << "\n #### =============================================================================\n";
        }
        loadData::loadPtreeValue(pt, swing_kp_, prefix + "kp", verbose);
        loadData::loadPtreeValue(pt, swing_kd_, prefix + "kd", verbose);

        prefix = "footPlacementCbfTask.";
        if (verbose)
        {
            std::cerr << "\n #### Foot Placement CBF Task:";
            std::cerr << "\n #### =============================================================================\n";
        }
        if (const auto gamma = pt.get_optional<scalar_t>(prefix + "gamma"))
        {
            foot_cbf_gamma_ = *gamma;
        }
        if (const auto xi = pt.get_optional<scalar_t>(prefix + "xi"))
        {
            foot_cbf_xi_ = *xi;
        }
        if (const auto activationProgress = pt.get_optional<scalar_t>(prefix + "swingActivationProgress"))
        {
            foot_cbf_activation_progress_ = *activationProgress;
        }
        if (const auto relaxationDerivativeDt = pt.get_optional<scalar_t>(prefix + "relaxationDerivativeDt"))
        {
            foot_cbf_relaxation_derivative_dt_ = *relaxationDerivativeDt;
        }
        if (const auto maxRelaxationRate = pt.get_optional<scalar_t>(prefix + "maxRelaxationRate"))
        {
            foot_cbf_max_relaxation_rate_ = *maxRelaxationRate;
        }
        if (const auto maxRelaxationAccel = pt.get_optional<scalar_t>(prefix + "maxRelaxationAccel"))
        {
            foot_cbf_max_relaxation_accel_ = *maxRelaxationAccel;
        }
        if (verbose)
        {
            std::cerr << prefix << "gamma: " << foot_cbf_gamma_ << std::endl;
            std::cerr << prefix << "xi: " << foot_cbf_xi_ << std::endl;
            std::cerr << prefix << "swingActivationProgress: " << foot_cbf_activation_progress_ << std::endl;
            std::cerr << prefix << "relaxationDerivativeDt: " << foot_cbf_relaxation_derivative_dt_ << std::endl;
            std::cerr << prefix << "maxRelaxationRate: " << foot_cbf_max_relaxation_rate_ << std::endl;
            std::cerr << prefix << "maxRelaxationAccel: " << foot_cbf_max_relaxation_accel_ << std::endl;
        }
    }
} // namespace legged
