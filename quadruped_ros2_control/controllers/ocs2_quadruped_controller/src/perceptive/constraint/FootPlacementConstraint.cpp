//
// Created by biao on 3/21/25.
//

#include "ocs2_quadruped_controller/perceptive/constraint/FootPlacementConstraint.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedPrecomputation.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

namespace ocs2::legged_robot
{
    FootPlacementConstraint::FootPlacementConstraint(const SwitchedModelReferenceManager& referenceManager,
                                                     const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                     size_t contactPointIndex)
        : StateConstraint(ConstraintOrder::Linear),
          referenceManagerPtr_(&referenceManager),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          contactPointIndex_(contactPointIndex)
    {
    }

    FootPlacementConstraint::FootPlacementConstraint(const FootPlacementConstraint& rhs)
        : StateConstraint(ConstraintOrder::Linear),
          referenceManagerPtr_(rhs.referenceManagerPtr_),
          endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
          contactPointIndex_(rhs.contactPointIndex_)
    {
    }

    bool FootPlacementConstraint::isActive(scalar_t time) const
    {
        return dynamic_cast<const PerceptiveLeggedReferenceManager&>(*referenceManagerPtr_).getFootPlacementFlags(time)[
            contactPointIndex_];
    }

    size_t FootPlacementConstraint::getNumConstraints(scalar_t time) const
    {
        const auto& refMgr = dynamic_cast<const PerceptiveLeggedReferenceManager&>(*referenceManagerPtr_);
        const auto& selector = *refMgr.getConvexRegionSelectorPtr();
        auto polygon = selector.getConvexPolygon(contactPointIndex_, time);
        return polygon.size();
    }

    vector_t FootPlacementConstraint::getValue(scalar_t time, const vector_t& state,
                                               const PreComputation& preComp) const
    {
        const auto param = cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[
            contactPointIndex_];
        const auto rawValue = computeRawConstraintValue(state, param);    // h​(x)=Ai​⋅pWi​​(x)+bi​≥0
        const auto relaxationDistance = dynamic_cast<const PerceptiveLeggedReferenceManager&>(*referenceManagerPtr_)
                                            .getFootPlacementRelaxation(contactPointIndex_, time);
        const auto relaxationOffset = computeRelaxationOffset(param, relaxationDistance);

        return rawValue + relaxationOffset;
    }

    VectorFunctionLinearApproximation FootPlacementConstraint::getLinearApproximation(
        scalar_t time, const vector_t& state,
        const PreComputation& preComp) const
    {
        const auto param = cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[
            contactPointIndex_];

        const size_t numConstraints = param.a.rows();

        VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(
            numConstraints, state.size(), 0);

        const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
        const auto relaxationDistance = dynamic_cast<const PerceptiveLeggedReferenceManager&>(*referenceManagerPtr_)
                                            .getFootPlacementRelaxation(contactPointIndex_, time);
        const auto relaxationOffset = computeRelaxationOffset(param, relaxationDistance);
        approx.f = param.a * positionApprox.f + param.b + relaxationOffset;
        approx.dfdx = param.a * positionApprox.dfdx;
        return approx;
    }

  
    vector_t FootPlacementConstraint::computeRawConstraintValue(const vector_t& state, const Parameter& param) const
    {
        return param.a * endEffectorKinematicsPtr_->getPosition(state).front() + param.b;    // h​(x)=Ai​⋅pWi​​(x)+bi​≥0
    }

    vector_t FootPlacementConstraint::computeRelaxationOffset(const Parameter& param, scalar_t relaxationDistance) const
    {
        vector_t relaxationOffset = vector_t::Zero(param.a.rows());
        for (int row = 0; row < param.a.rows(); ++row)
        {
            relaxationOffset(row) = relaxationDistance * param.a.row(row).norm();
        }
        return relaxationOffset;
    }
} // namespace legged
