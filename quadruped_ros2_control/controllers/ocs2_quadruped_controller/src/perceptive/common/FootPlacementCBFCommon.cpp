#include "ocs2_quadruped_controller/perceptive/common/FootPlacementCBFCommon.h"

#include "ocs2_quadruped_controller/perceptive/interface/ConvexRegionSelector.h"

#include <algorithm>
#include <limits>

#include <ocs2_core/misc/Lookup.h>

namespace ocs2::legged_robot::foot_placement
{
    vector_t computeRelaxationOffset(const matrix_t& a, scalar_t relaxationDistance)
    {
        vector_t relaxationOffset = vector_t::Zero(a.rows());
        for (int row = 0; row < a.rows(); ++row)
        {
            relaxationOffset(row) = relaxationDistance * a.row(row).norm();
        }
        return relaxationOffset;
    }

    vector_t computeRelaxationRateOffset(const matrix_t& a, scalar_t relaxationRate)
    {
        vector_t relaxationRateOffset = vector_t::Zero(a.rows());
        for (int row = 0; row < a.rows(); ++row)
        {
            relaxationRateOffset(row) = relaxationRate * a.row(row).norm();
        }
        return relaxationRateOffset;
    }

    CbfState evaluateFirstOrder(const vector3_t& position,
                                const vector3_t& velocity,
                                const matrix_t& a,
                                const vector_t& b,
                                scalar_t relaxationDistance,
                                scalar_t relaxationRate,
                                scalar_t gamma)
    {
        const auto relaxationOffset = computeRelaxationOffset(a, relaxationDistance);
        const auto relaxationRateOffset = computeRelaxationRateOffset(a, relaxationRate);

        CbfState state;
        state.h = a * position + b + relaxationOffset;
        state.hDot = a * velocity + relaxationRateOffset;
        state.hE = state.hDot + gamma * state.h;
        return state;
    }

    vector_t evaluateFirstOrderConstraint(const CbfState& state)
    {
        return state.hE;
    }

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
                                                  size_t numDecisionVars)
    {
        SecondOrderCbfRows result;
        const int numRows = static_cast<int>(a.rows());
        if (numRows == 0)
        {
            result.d = matrix_t::Zero(0, static_cast<Eigen::Index>(numDecisionVars));
            result.fUpper = vector_t::Zero(0);
            return result;
        }

        const CbfState cbf = evaluateFirstOrder(position, velocity, a, b, relaxationDistance, relaxationRate, gamma);
        result.hMin = cbf.h.size() > 0 ? cbf.h.minCoeff() : 0.0;
        result.hDotMin = cbf.hDot.size() > 0 ? cbf.hDot.minCoeff() : 0.0;
        result.hEMin = cbf.hE.size() > 0 ? cbf.hE.minCoeff() : 0.0;

        result.d = matrix_t::Zero(numRows, static_cast<Eigen::Index>(numDecisionVars));
        result.fUpper = vector_t::Zero(numRows);

        const vector3_t footAccelNoQdd = djFoot * vMeasured;
        result.ecbfResidualMin = std::numeric_limits<scalar_t>::max();
        for (int row = 0; row < numRows; ++row)
        {
            const vector3_t aRow = a.row(row).transpose();
            const scalar_t hDotRow = cbf.hDot(row);
            const scalar_t hERow = cbf.hE(row);

            // h_e_dot + xi * h_e >= 0
            // => -a * J * qdd <= a * dJv + gamma * h_dot + xi * h_e
            const scalar_t upperBound = aRow.dot(footAccelNoQdd) + gamma * hDotRow + xi * hERow;
            result.d.row(row).head(jFoot.cols()).noalias() = -aRow.transpose() * jFoot;
            result.fUpper(row) = upperBound;

            // Residual of h_e_dot + xi*h_e with qdd = 0 (constant part only).
            result.ecbfResidualMin = std::min(result.ecbfResidualMin, upperBound);
        }
        if (numRows == 0)
        {
            result.ecbfResidualMin = 0.0;
        }
        return result;
    }

    scalar_t computeSwingPhaseProgressFromModeSchedule(const ModeSchedule& modeSchedule,
                                                       const ConvexRegionSelector& selector,
                                                       size_t leg,
                                                       scalar_t time)
    {
        if (modeSchedule.eventTimes.empty() || modeSchedule.modeSequence.empty())
        {
            return scalar_t(-1.0);
        }

        const auto contactFlagStocks = selector.extractContactFlags(modeSchedule.modeSequence);
        if (leg >= contactFlagStocks.size())
        {
            return scalar_t(-1.0);
        }

        const auto& contactFlags = contactFlagStocks[leg];
        if (contactFlags.empty())
        {
            return scalar_t(-1.0);
        }

        const size_t phaseIndex = static_cast<size_t>(lookup::findIndexInTimeArray(modeSchedule.eventTimes, time));

        const size_t clampedIndex = std::min(phaseIndex, contactFlags.size() - 1);
        if (contactFlags[clampedIndex])
        {
            return scalar_t(-1.0);
        }

        int startIdx = -1;
        for (int i = static_cast<int>(clampedIndex) - 1; i >= 0; --i)
        {
            if (contactFlags[static_cast<size_t>(i)])
            {
                startIdx = i;
                break;
            }
        }

        int finalIdx = static_cast<int>(contactFlags.size()) - 1;
        for (size_t i = clampedIndex + 1; i < contactFlags.size(); ++i)
        {
            if (contactFlags[i])
            {
                finalIdx = static_cast<int>(i) - 1;
                break;
            }
        }
        if (startIdx < 0)
        {
            return scalar_t(-1.0);
        }

        const int lastEvent = static_cast<int>(modeSchedule.eventTimes.size()) - 1;
        const int safeStart = std::min(std::max(startIdx, 0), lastEvent);
        const int safeFinal = std::min(std::max(finalIdx, 0), lastEvent);

        const scalar_t t0 = modeSchedule.eventTimes[static_cast<size_t>(safeStart)];
        const scalar_t t1Raw = modeSchedule.eventTimes[static_cast<size_t>(safeFinal)];
        const scalar_t t1 = (t1Raw > t0) ? t1Raw : (t0 + scalar_t(1e-3));

        const scalar_t alpha = (time - t0) / (t1 - t0);
        return std::min<scalar_t>(1.0, std::max<scalar_t>(0.0, alpha));
    }
} // namespace ocs2::legged_robot::foot_placement
