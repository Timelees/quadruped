#pragma once

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "ocs2_quadruped_controller/perceptive/constraint/FootPlacementConstraint.h"


namespace ocs2::legged_robot
{
    class FootPlacementCBFConstraint final : public StateInputConstraint
    {
    public:
        struct Config
        {
            scalar_t gamma = 5.0;
            scalar_t swingActivationProgress = -1.0;
            scalar_t relaxationDerivativeDt = 1e-3;
        };

        FootPlacementCBFConstraint(const SwitchedModelReferenceManager& referenceManager,
                                  const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                  size_t contactPointIndex,
                                  Config config);

        ~FootPlacementCBFConstraint() override = default;
        FootPlacementCBFConstraint* clone() const override { return new FootPlacementCBFConstraint(*this); }

        bool isActive(scalar_t time) const override;
        size_t getNumConstraints(scalar_t time) const override;
        vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input,
                          const PreComputation& preComp) const override;
        VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                                 const vector_t& input,
                                                                 const PreComputation& preComp) const override;

    private:
        vector_t computeRawConstraintValue(const vector_t& state, const vector_t& input,
                                           const FootPlacementConstraint::Parameter& param,
                                           scalar_t relaxationDistance, scalar_t relaxationRate) const;
        vector_t computeRelaxationOffset(const FootPlacementConstraint::Parameter& param,
                                         scalar_t relaxationDistance) const;
        vector_t computeRelaxationRateOffset(const FootPlacementConstraint::Parameter& param,
                                             scalar_t relaxationRate) const;
        scalar_t computeRelaxationRate(scalar_t time) const;

        FootPlacementCBFConstraint(const FootPlacementCBFConstraint& rhs);

        const SwitchedModelReferenceManager* referenceManagerPtr_;
        std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;

        const size_t contactPointIndex_;
        Config config_;
    };

}
