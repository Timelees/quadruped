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

    vector_t FootPlacementConstraint::getValue(scalar_t /*time*/, const vector_t& state,
                                               const PreComputation& preComp) const
    {
        const auto param = cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[
            contactPointIndex_];
        return param.a * endEffectorKinematicsPtr_->getPosition(state).front() + param.b;
    }

    VectorFunctionLinearApproximation FootPlacementConstraint::getLinearApproximation(
        scalar_t /*time*/, const vector_t& state,
        const PreComputation& preComp) const
    {
        const auto param = cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[
            contactPointIndex_];

        const size_t numConstraints = param.a.rows();

        VectorFunctionLinearApproximation approx = VectorFunctionLinearApproximation::Zero(
            numConstraints, state.size(), 0);

        const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
        approx.f = param.a * positionApprox.f + param.b;
        approx.dfdx = param.a * positionApprox.dfdx;
        return approx;
    }
} // namespace legged
