//
// Runtime foot placement safety set provider for WBC CBF tasks.
//

#pragma once

#include <memory>

#include "ocs2_quadruped_controller/perceptive/common/FootPlacementSafetySet.h"
#include "ocs2_quadruped_controller/perceptive/constraint/FootPlacementConstraint.h"
#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

namespace ocs2::legged_robot
{
    class FootPlacementSafetyProvider
    {
    public:
        explicit FootPlacementSafetyProvider(PerceptiveLeggedReferenceManager& referenceManager);

        FootPlacementConstraint::Parameter getHalfSpaceParams(size_t leg, scalar_t time) const;

        scalar_t getRelaxation(size_t leg, scalar_t time) const;

        scalar_t getRelaxationRate(size_t leg, scalar_t time, scalar_t derivativeDt) const;

        bool isCbfActive(size_t leg, scalar_t time, scalar_t swingActivationProgress) const;

        const PerceptiveLeggedReferenceManager& referenceManager() const { return referenceManager_; }

    private:
        PerceptiveLeggedReferenceManager& referenceManager_;
    };
} // namespace ocs2::legged_robot
