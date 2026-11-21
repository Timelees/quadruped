//
// Created by biao on 3/21/25.
//

#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedPrecomputation.h"

#include "ocs2_quadruped_controller/perceptive/common/FootPlacementSafetySet.h"

namespace ocs2::legged_robot
{
    PerceptiveLeggedPrecomputation::PerceptiveLeggedPrecomputation(PinocchioInterface pinocchioInterface,
                                                                   const CentroidalModelInfo& info,
                                                                   const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                                                                   ModelSettings settings,
                                                                   const ConvexRegionSelector& convexRegionSelector)
        : LeggedRobotPreComputation(std::move(pinocchioInterface), info, swingTrajectoryPlanner, std::move(settings)),
          convexRegionSelectorPtr_(&convexRegionSelector)
    {
        footPlacementConParameters_.resize(info.numThreeDofContacts);
    }

    PerceptiveLeggedPrecomputation::PerceptiveLeggedPrecomputation(const PerceptiveLeggedPrecomputation& rhs)
        : LeggedRobotPreComputation(rhs),
          convexRegionSelectorPtr_(rhs.convexRegionSelectorPtr_)
    {
        footPlacementConParameters_.resize(rhs.footPlacementConParameters_.size());
    }

    void PerceptiveLeggedPrecomputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u)
    {
        if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint))
        {
            return;
        }
        LeggedRobotPreComputation::request(request, t, x, u);

        if (request.contains(Request::Constraint) || request.contains(Request::SoftConstraint))
        {
            for (size_t i = 0; i < footPlacementConParameters_.size(); i++)
            {
                FootPlacementConstraint::Parameter params;
                params.a = matrix_t::Zero(0, 3);
                params.b = vector_t::Zero(0);
                footPlacementConParameters_[i] = params;
            
                auto projection = convexRegionSelectorPtr_->getProjection(i, t);    // 获取足端投影
                if (projection.regionPtr == nullptr)
                {
                    // Swing leg
                    continue;
                }
            
                const auto safetyPolygon = convexRegionSelectorPtr_->getConvexPolygon(i, t);
                if (safetyPolygon.size() < 3)
                {
                    continue;
                }

                const auto halfSpace = foot_placement::buildWorldHalfSpace(
                    safetyPolygon, projection.regionPtr->transformPlaneToWorld);
                params.a = halfSpace.a;
                params.b = halfSpace.b;
            
                footPlacementConParameters_[i] = params;
            }
        }
    }
} // namespace legged
