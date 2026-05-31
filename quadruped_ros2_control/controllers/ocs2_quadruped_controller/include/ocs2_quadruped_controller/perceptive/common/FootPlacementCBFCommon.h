//
// Shared first- and second-order foot placement CBF math for MPC and WBC layers.
//

#pragma once

#include <ocs2_core/reference/ModeSchedule.h>
#include <ocs2_legged_robot/common/Types.h>

namespace ocs2::legged_robot
{
    class ConvexRegionSelector;
}

namespace ocs2::legged_robot::foot_placement
{
    struct CbfState
    {
        vector_t h;
        vector_t hDot;
        vector_t hE;
    };

    struct SecondOrderCbfRows
    {
        matrix_t d;
        vector_t fUpper;
        scalar_t hMin{0.0};
        scalar_t hDotMin{0.0};
        scalar_t hEMin{0.0};
        scalar_t ecbfResidualMin{0.0};
    };

    vector_t computeRelaxationOffset(const matrix_t& a, scalar_t relaxationDistance);

    vector_t computeRelaxationRateOffset(const matrix_t& a, scalar_t relaxationRate);

    CbfState evaluateFirstOrder(const vector3_t& position,
                                const vector3_t& velocity,
                                const matrix_t& a,
                                const vector_t& b,
                                scalar_t relaxationDistance,
                                scalar_t relaxationRate,
                                scalar_t gamma);

    vector_t evaluateFirstOrderConstraint(const CbfState& state);

    /** Build WBC inequality rows: d * x <= fUpper with x = [qdd, F, tau]. */
    SecondOrderCbfRows buildSecondOrderInequality(const matrix_t& a,
                                                  const vector_t& b,
                                                  const vector3_t& position,
                                                  const vector3_t& velocity,
                                                  const matrix_t& jFoot,
                                                  const matrix_t& djFoot,
                                                  const vector_t& vMeasured,
                                                  scalar_t relaxationDistance,
                                                  scalar_t relaxationRate,
                                                  scalar_t gamma,
                                                  scalar_t xi,
                                                  size_t numDecisionVars);

    scalar_t computeSwingPhaseProgressFromModeSchedule(const ModeSchedule& modeSchedule,
                                                       const ConvexRegionSelector& selector,
                                                       size_t leg,
                                                       scalar_t time);

} // namespace ocs2::legged_robot::foot_placement
