#include "ocs2_quadruped_controller/perceptive/interface/FootPlacementSafetyProvider.h"

#include "ocs2_quadruped_controller/perceptive/common/FootPlacementCBFCommon.h"

#include <algorithm>

namespace ocs2::legged_robot
{
    FootPlacementSafetyProvider::FootPlacementSafetyProvider(PerceptiveLeggedReferenceManager& referenceManager)
        : referenceManager_(referenceManager)
    {
    }

    FootPlacementConstraint::Parameter FootPlacementSafetyProvider::getHalfSpaceParams(size_t leg, scalar_t time) const
    {
        FootPlacementConstraint::Parameter params;
        params.a = matrix_t::Zero(0, 3);
        params.b = vector_t::Zero(0);

        const auto& selector = *referenceManager_.getConvexRegionSelectorPtr();
        const auto projection = selector.getProjection(leg, time);
        if (projection.regionPtr == nullptr)
        {
            return params;
        }

        const auto safetyPolygon = selector.getConvexPolygon(leg, time);
        if (safetyPolygon.size() < 3)
        {
            return params;
        }

        const auto halfSpace = foot_placement::buildWorldHalfSpace(
            safetyPolygon, projection.regionPtr->transformPlaneToWorld);
        params.a = halfSpace.a;
        params.b = halfSpace.b;
        return params;
    }

    scalar_t FootPlacementSafetyProvider::getRelaxation(size_t leg, scalar_t time) const
    {
        return referenceManager_.getFootPlacementRelaxation(leg, time);
    }

    scalar_t FootPlacementSafetyProvider::getRelaxationRate(size_t leg, scalar_t time, scalar_t derivativeDt) const
    {
        const scalar_t dt = std::max<scalar_t>(derivativeDt, 1e-6);
        const scalar_t futureRelaxation = referenceManager_.getFootPlacementRelaxation(leg, time + dt);
        const scalar_t currentRelaxation = referenceManager_.getFootPlacementRelaxation(leg, time);
        return (futureRelaxation - currentRelaxation) / dt;
    }

    bool FootPlacementSafetyProvider::isCbfActive(size_t leg, scalar_t time, scalar_t swingActivationProgress) const
    {
        const auto& selector = *referenceManager_.getConvexRegionSelectorPtr();
        const bool hasPolygon = selector.getConvexPolygon(leg, time).size() >= 3;
        const bool isContact = referenceManager_.getContactFlags(time)[leg];
        const scalar_t swingProgress = foot_placement::computeSwingPhaseProgressFromModeSchedule(
            referenceManager_.getModeSchedule(), selector, leg, time);
        const scalar_t activationProgress = swingActivationProgress >= 0.0
                                                ? swingActivationProgress
                                                : referenceManager_.getFootPlacementRelaxationConfig().swingActivationProgress;
        const bool isLateSwing = swingProgress >= 0.0 && swingProgress >= activationProgress;
        return hasPolygon && !isContact && isLateSwing;
    }
} // namespace ocs2::legged_robot
