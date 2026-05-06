#include "ocs2_quadruped_controller/perceptive/constraint/FootPlacementCBFConstraint.h"

#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedPrecomputation.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

#include <algorithm>

namespace ocs2::legged_robot
{
	FootPlacementCBFConstraint::FootPlacementCBFConstraint(const SwitchedModelReferenceManager& referenceManager,
														   const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
										   size_t contactPointIndex,
										   Config config)
		: StateInputConstraint(ConstraintOrder::Linear),
		  referenceManagerPtr_(&referenceManager),
		  endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
		  contactPointIndex_(contactPointIndex),
		  config_(config)
	{
	}

	FootPlacementCBFConstraint::FootPlacementCBFConstraint(const FootPlacementCBFConstraint& rhs)
		: StateInputConstraint(rhs),
		  referenceManagerPtr_(rhs.referenceManagerPtr_),
		  endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
		  contactPointIndex_(rhs.contactPointIndex_),
		  config_(rhs.config_)
	{
	}

	bool FootPlacementCBFConstraint::isActive(scalar_t time) const
	{
		const auto& refMgr = dynamic_cast<const PerceptiveLeggedReferenceManager&>(*referenceManagerPtr_);
		const auto& selector = *refMgr.getConvexRegionSelectorPtr();
		const bool hasPolygon = selector.getConvexPolygon(contactPointIndex_, time).size() >= 3;
		const bool isContact = refMgr.getContactFlags(time)[contactPointIndex_];
		const scalar_t swingProgress = refMgr.getSwingTrajectoryPlanner()->getSwingPhaseProgress(contactPointIndex_, time);
		const scalar_t activationProgress = config_.swingActivationProgress >= 0.0
		                                        ? config_.swingActivationProgress
		                                        : refMgr.getFootPlacementRelaxationConfig().swingActivationProgress;
		const bool isLateSwing = swingProgress >= 0.0 && swingProgress >= activationProgress;
		return hasPolygon && !isContact && isLateSwing;
	}

	size_t FootPlacementCBFConstraint::getNumConstraints(scalar_t time) const
	{
		const auto& refMgr = dynamic_cast<const PerceptiveLeggedReferenceManager&>(*referenceManagerPtr_);
		const auto& selector = *refMgr.getConvexRegionSelectorPtr();
		return selector.getConvexPolygon(contactPointIndex_, time).size();
	}

	vector_t FootPlacementCBFConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
												  const PreComputation& preComp) const
	{
		const auto param = cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[contactPointIndex_];
		const auto& refMgr = dynamic_cast<const PerceptiveLeggedReferenceManager&>(*referenceManagerPtr_);
		const auto relaxationDistance = refMgr.getFootPlacementRelaxation(contactPointIndex_, time);
		const auto relaxationRate = computeRelaxationRate(time);
		return computeRawConstraintValue(state, input, param, relaxationDistance, relaxationRate);
	}

	VectorFunctionLinearApproximation FootPlacementCBFConstraint::getLinearApproximation(
		scalar_t time, const vector_t& state, const vector_t& input,
		const PreComputation& preComp) const
	{
		const auto param = cast<PerceptiveLeggedPrecomputation>(preComp).getFootPlacementConParameters()[contactPointIndex_];
		const size_t numConstraints = static_cast<size_t>(param.a.rows());
		VectorFunctionLinearApproximation approx =
			VectorFunctionLinearApproximation::Zero(numConstraints, state.size(), input.size());

		const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
		const auto velocityApprox = endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input).front();
		const auto& refMgr = dynamic_cast<const PerceptiveLeggedReferenceManager&>(*referenceManagerPtr_);
		const auto relaxationDistance = refMgr.getFootPlacementRelaxation(contactPointIndex_, time);
		const auto relaxationRate = computeRelaxationRate(time);

		const auto relaxationOffset = computeRelaxationOffset(param, relaxationDistance);
		const auto relaxationRateOffset = computeRelaxationRateOffset(param, relaxationRate);

		approx.f = param.a * velocityApprox.f + relaxationRateOffset +
				   config_.gamma * (param.a * positionApprox.f + param.b + relaxationOffset);
		approx.dfdx = param.a * velocityApprox.dfdx + config_.gamma * (param.a * positionApprox.dfdx);
		approx.dfdu = param.a * velocityApprox.dfdu;
		return approx;
	}

	vector_t FootPlacementCBFConstraint::computeRawConstraintValue(
		const vector_t& state, const vector_t& input, const FootPlacementConstraint::Parameter& param,
		scalar_t relaxationDistance, scalar_t relaxationRate) const
	{
		const auto position = endEffectorKinematicsPtr_->getPosition(state).front();
		const auto velocity = endEffectorKinematicsPtr_->getVelocity(state, input).front();
		const auto relaxationOffset = computeRelaxationOffset(param, relaxationDistance);
		const auto relaxationRateOffset = computeRelaxationRateOffset(param, relaxationRate);
		return param.a * velocity + relaxationRateOffset +
			   config_.gamma * (param.a * position + param.b + relaxationOffset);
	}

	vector_t FootPlacementCBFConstraint::computeRelaxationOffset(const FootPlacementConstraint::Parameter& param,
																 scalar_t relaxationDistance) const
	{
		vector_t relaxationOffset = vector_t::Zero(param.a.rows());
		for (int row = 0; row < param.a.rows(); ++row)
		{
			relaxationOffset(row) = relaxationDistance * param.a.row(row).norm();
		}
		return relaxationOffset;
	}

	vector_t FootPlacementCBFConstraint::computeRelaxationRateOffset(const FootPlacementConstraint::Parameter& param,
																	 scalar_t relaxationRate) const
	{
		vector_t relaxationRateOffset = vector_t::Zero(param.a.rows());
		for (int row = 0; row < param.a.rows(); ++row)
		{
			relaxationRateOffset(row) = relaxationRate * param.a.row(row).norm();
		}
		return relaxationRateOffset;
	}

	scalar_t FootPlacementCBFConstraint::computeRelaxationRate(scalar_t time) const
	{
		const auto& refMgr = dynamic_cast<const PerceptiveLeggedReferenceManager&>(*referenceManagerPtr_);
		const scalar_t dt = std::max<scalar_t>(config_.relaxationDerivativeDt, 1e-6);
		const scalar_t futureRelaxation = refMgr.getFootPlacementRelaxation(contactPointIndex_, time + dt);
		const scalar_t currentRelaxation = refMgr.getFootPlacementRelaxation(contactPointIndex_, time);
		return (futureRelaxation - currentRelaxation) / dt;
	}
} // namespace ocs2::legged_robot
