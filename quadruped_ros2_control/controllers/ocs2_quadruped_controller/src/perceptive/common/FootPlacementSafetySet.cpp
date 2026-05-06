#include "ocs2_quadruped_controller/perceptive/common/FootPlacementSafetySet.h"

#include <algorithm>
#include <cmath>

namespace ocs2::legged_robot::foot_placement
{
    namespace
    {
        bool isInsideHalfSpace(const vector_t& values)
        {
            constexpr scalar_t tolerance = 1e-8;
            return values.minCoeff() >= -tolerance;
        }
    } // namespace

    HalfSpace2d buildPlanarHalfSpace(const convex_plane_decomposition::CgalPolygon2d& polygon,
                                     scalar_t shrinkDistance)
    {
        const size_t numVertices = polygon.size();
        HalfSpace2d halfSpace;
        halfSpace.a = matrix_t::Zero(numVertices, 2);
        halfSpace.b = vector_t::Zero(numVertices);

        for (size_t i = 0; i < numVertices; ++i)
        {
            const size_t j = (i + 1) % numVertices;
            const size_t k = (j + 1) % numVertices;

            const auto pointA = polygon.vertex(i);
            const auto pointB = polygon.vertex(j);
            const auto pointC = polygon.vertex(k);

            halfSpace.a.row(i) << pointB.y() - pointA.y(), pointA.x() - pointB.x();
            halfSpace.b(i) = pointA.y() * pointB.x() - pointA.x() * pointB.y();
            if (halfSpace.a.row(i) * (vector_t(2) << pointC.x(), pointC.y()).finished() + halfSpace.b(i) < 0.0)
            {
                halfSpace.a.row(i) *= -1.0;
                halfSpace.b(i) *= -1.0;
            }

            halfSpace.b(i) -= shrinkDistance * halfSpace.a.row(i).norm();
        }

        return halfSpace;
    }

    convex_plane_decomposition::CgalPolygon2d shrinkConvexPolygon(
        const convex_plane_decomposition::CgalPolygon2d& polygon,
        scalar_t shrinkDistance)
    {
        convex_plane_decomposition::CgalPolygon2d shrunkPolygon;
        if (polygon.size() < 3)
        {
            return shrunkPolygon;
        }

        const auto halfSpace = buildPlanarHalfSpace(polygon, shrinkDistance);
        shrunkPolygon.container().reserve(polygon.size());

        for (size_t i = 0; i < polygon.size(); ++i)
        {
            const size_t j = (i + 1) % polygon.size();
            const Eigen::Vector2d ai = halfSpace.a.row(i).transpose();
            const Eigen::Vector2d aj = halfSpace.a.row(j).transpose();
            const scalar_t determinant = ai.x() * aj.y() - ai.y() * aj.x();
            if (std::abs(determinant) < 1e-10)
            {
                continue;
            }

            const scalar_t x = (-halfSpace.b(i) * aj.y() + ai.y() * halfSpace.b(j)) / determinant;
            const scalar_t y = (-ai.x() * halfSpace.b(j) + halfSpace.b(i) * aj.x()) / determinant;
            const vector_t halfSpaceValue = halfSpace.a * (vector_t(2) << x, y).finished() + halfSpace.b;
            if (isInsideHalfSpace(halfSpaceValue))
            {
                shrunkPolygon.push_back({x, y});
            }
        }

        return shrunkPolygon.size() >= 3 ? shrunkPolygon : convex_plane_decomposition::CgalPolygon2d();
    }

    HalfSpace3d buildWorldHalfSpace(const convex_plane_decomposition::CgalPolygon2d& safetyPolygon,
                                    const Eigen::Isometry3d& transformPlaneToWorld)
    {
        const auto planarHalfSpace = buildPlanarHalfSpace(safetyPolygon);
        matrix_t planeSelector = matrix_t::Zero(2, 3);
        planeSelector(0, 0) = 1.0;
        planeSelector(1, 1) = 1.0;

        const auto worldToPlane = transformPlaneToWorld.inverse();
        HalfSpace3d worldHalfSpace;
        worldHalfSpace.a = planarHalfSpace.a * planeSelector * worldToPlane.linear();
        worldHalfSpace.b = planarHalfSpace.b + planarHalfSpace.a * worldToPlane.translation().head(2);
        return worldHalfSpace;
    }
} // namespace ocs2::legged_robot::foot_placement
