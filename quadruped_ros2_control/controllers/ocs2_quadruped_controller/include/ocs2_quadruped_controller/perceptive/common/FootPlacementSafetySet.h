//
// Common safe-set construction for perceptive foot placement.
//

#pragma once

#include <utility>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>
#include <ocs2_legged_robot/common/Types.h>

namespace ocs2::legged_robot::foot_placement
{
    constexpr scalar_t kSafetyPolygonShrinkDistance = 0.01;

    struct HalfSpace2d
    {
        matrix_t a;
        vector_t b;
    };

    struct HalfSpace3d
    {
        matrix_t a;
        vector_t b;
    };

    HalfSpace2d buildPlanarHalfSpace(const convex_plane_decomposition::CgalPolygon2d& polygon,
                                     scalar_t shrinkDistance = 0.0);

    convex_plane_decomposition::CgalPolygon2d shrinkConvexPolygon(
        const convex_plane_decomposition::CgalPolygon2d& polygon,
        scalar_t shrinkDistance = kSafetyPolygonShrinkDistance);

    HalfSpace3d buildWorldHalfSpace(const convex_plane_decomposition::CgalPolygon2d& safetyPolygon,
                                    const Eigen::Isometry3d& transformPlaneToWorld);
} // namespace ocs2::legged_robot::foot_placement
