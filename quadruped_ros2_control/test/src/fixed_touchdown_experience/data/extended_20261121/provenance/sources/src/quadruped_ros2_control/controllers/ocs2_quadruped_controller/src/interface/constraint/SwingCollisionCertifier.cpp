#include "ocs2_quadruped_controller/interface/constraint/SwingCollisionCertifier.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace ocs2::legged_robot {
namespace {
// A separating projection of (hull + sphere) and the entire downward prism.
// Candidate axes include hull faces, prism faces, and edge cross products.
// Failure to separate is UNKNOWN, never a collision assertion.
bool separated(const Eigen::MatrixXd& hull, const TerrainPrism& prism, double radius, double tolerance) {
    auto axisSeparates = [&](Eigen::Vector3d axis) {
        const double norm = axis.norm();
        if (norm < 1e-14) {
            return false;
        }
        axis /= norm;
        const Eigen::VectorXd projections = hull * axis;
        const double upper = axis.x() * (axis.x() >= 0 ? prism.maxXY.x() : prism.minXY.x()) +
                             axis.y() * (axis.y() >= 0 ? prism.maxXY.y() : prism.minXY.y()) +
                             axis.z() * prism.height;
        const double lower = axis.x() * (axis.x() >= 0 ? prism.minXY.x() : prism.maxXY.x()) +
                             axis.y() * (axis.y() >= 0 ? prism.minXY.y() : prism.maxXY.y()) +
                             axis.z() * prism.height;
        return (axis.z() >= 0 && projections.minCoeff() - radius >= upper - tolerance) ||
               (axis.z() <= 0 && projections.maxCoeff() + radius <= lower + tolerance);
    };
    for (int k = 0; k < 3; ++k) {
        if (axisSeparates(Eigen::Vector3d::Unit(k))) {
            return true;
        }
    }
    for (int i = 0; i < hull.rows(); ++i) {
        for (int j = i + 1; j < hull.rows(); ++j) {
            const Eigen::Vector3d edge = (hull.row(j) - hull.row(i)).transpose();
            for (int k = 0; k < 3; ++k) {
                if (axisSeparates(edge.cross(Eigen::Vector3d::Unit(k)))) {
                    return true;
                }
            }
            for (int k = j + 1; k < hull.rows(); ++k) {
                if (axisSeparates(edge.cross(Eigen::Vector3d((hull.row(k) - hull.row(i)).transpose())))) {
                    return true;
                }
            }
        }
    }
    return false;
}
}  // namespace
SwingCollisionCertifier::SwingCollisionCertifier(std::shared_ptr<TerrainInterface> terrain,
                                                 CollisionCheckConfig config)
    : terrain_(std::move(terrain)), config_(config) {
    if (!terrain_ || config_.maxDepth < 0 || config_.maxSegments < 1 ||
        !std::isfinite(config_.numericalTolerance) || config_.numericalTolerance < 0) {
        throw std::invalid_argument("Invalid collision certificate configuration.");
    }
}
CollisionCertificate SwingCollisionCertifier::check(const BSpline& spline, double radius) const {
    if (!std::isfinite(radius) || radius < 0) {
        throw std::invalid_argument("Invalid foot radius.");
    }
    const Eigen::MatrixXd controls = spline.getControlPoints();
    auto spans = extractPolynomialSegments(spline);
    const double trajectoryStart = spans.front().start, trajectoryEnd = spans.back().end;
    struct Pending {
        PolynomialSegment segment;
        int depth;
    };
    std::vector<Pending> pending;
    for (auto& span : spans) {
        pending.push_back({std::move(span), 0});
    }
    CollisionCertificate result;
    bool unknown = false;
    try {
        while (!pending.empty() && result.checkedSegments < config_.maxSegments) {
            const auto current = std::move(pending.back());
            pending.pop_back();
            ++result.checkedSegments;
            const bool contactSegment =
                current.segment.start == trajectoryStart || current.segment.end == trajectoryEnd;
            Eigen::MatrixXd hull =
                hullMap(current.segment, contactSegment ? HullBasis::Bezier : config_.basis) * controls;
            hull.col(2).array() += radius;  // stored spline is centre z minus foot radius
            const Eigen::Vector2d lo =
                hull.leftCols(2).colwise().minCoeff().transpose() - Eigen::Vector2d::Constant(radius);
            const Eigen::Vector2d hi =
                hull.leftCols(2).colwise().maxCoeff().transpose() + Eigen::Vector2d::Constant(radius);
            bool safe = true;
            for (const auto& prism : terrain_->getPrisms(lo, hi)) {
                if (!separated(hull, prism, radius, config_.numericalTolerance)) {
                    safe = false;
                    break;
                }
            }
            // Contact segments touch the terrain at their endpoint by design
            // (lift-off / touch-down). The hull-vs-prism separator cannot
            // separate a hull that rests on the surface from the surface prism
            // below it, so skip the surface-penetration witness for contact
            // endpoints — they are legitimate contact, not collision.
            if (safe) {
                continue;
            }
            // A point penetrating the actual interpolated surface is a genuine witness.
            // This only rejects curves; successful samples never certify them.
            // Skip the trajectory endpoints: they are the lift-off and touch-down
            // contacts, where the foot is on the ground by construction.
            for (double t : {current.segment.start, 0.5 * (current.segment.start + current.segment.end),
                             current.segment.end}) {
                const bool isContactEndpoint = (t == trajectoryStart || t == trajectoryEnd);
                if (isContactEndpoint) {
                    continue;
                }
                const auto point = spline.getPosition(t);
                if (point.z() < terrain_->getHeight(point.x(), point.y()) - config_.numericalTolerance) {
                    result.status = CollisionStatus::CollisionDetected;
                    result.witnessTime = t;
                    return result;
                }
            }
            if (current.depth == config_.maxDepth) {
                unknown = true;
                continue;
            }
            auto halves = current.segment.bisect();
            pending.push_back({std::move(halves.first), current.depth + 1});
            pending.push_back({std::move(halves.second), current.depth + 1});
            ++result.subdivisions;
        }
    } catch (const TerrainUnavailable&) {
        return result;
    }
    result.status = unknown || !pending.empty() ? CollisionStatus::Unknown : CollisionStatus::ProvenSafe;
    return result;
}
}  // namespace ocs2::legged_robot
