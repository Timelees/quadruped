#include "ocs2_quadruped_controller/interface/constraint/TerrainSafetyCorridor.h"

#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <limits>

namespace ocs2::legged_robot {
namespace {
void addPlane(CorridorPolytope& cell, const Eigen::Vector3d& normal, double bound,
              double trackingMargin) {
    for (auto& plane : cell.halfSpaces) {
        if ((plane.normal - normal).norm() < 1e-10) {
            plane.geometricOffset = std::min(plane.geometricOffset, bound);
            plane.planningOffset = plane.geometricOffset - trackingMargin;
            return;
        }
    }
    cell.halfSpaces.push_back({normal, bound, bound - trackingMargin});
}

std::vector<PolynomialSegment> pieces(const BSpline& spline, const CorridorConfig& config, int divisions) {
    std::vector<PolynomialSegment> result;
    for (const auto& span : extractPolynomialSegments(spline)) {
        std::vector<PolynomialSegment> work{span};
        for (int count = 1; count < divisions; count *= 2) {
            std::vector<PolynomialSegment> next;
            for (const auto& piece : work) {
                auto halves = piece.bisect();
                next.push_back(std::move(halves.first));
                next.push_back(std::move(halves.second));
            }
            work = std::move(next);
        }
        for (auto piece : work) {
            if (piece.end <= config.contactPhase || piece.start >= 1.0 - config.contactPhase) {
                continue;
            }
            if (piece.start < config.contactPhase) {
                piece = piece.split(config.contactPhase).second;
            }
            if (piece.end > 1.0 - config.contactPhase) {
                piece = piece.split(1.0 - config.contactPhase).first;
            }
            result.push_back(std::move(piece));
        }
    }
    return result;
}
}  // namespace

TerrainSafetyCorridor buildTerrainSafetyCorridor(const TerrainInterface& terrain, const BSpline& reference,
                                                 const CorridorConfig& config, int subdivisionsPerSpan,
                                                 HullBasis basis) {
    const double values[] = {config.lateralExtent, config.longitudinalExtent, config.verticalExtent,
                             config.footRadius, config.mapMargin, config.geometricClearance,
                             config.trackingMargin, config.numericalMargin, config.contactPhase};
    for (double value : values) {
        if (!std::isfinite(value) || value < 0.0) {
            throw std::invalid_argument("Corridor distances must be finite and nonnegative.");
        }
    }
    if (subdivisionsPerSpan < 1 || config.contactPhase >= 0.5 || config.geometricClearance <= 0.0 ||
        config.lateralExtent <= config.trackingMargin + config.numericalMargin ||
        config.longitudinalExtent <= config.trackingMargin + config.numericalMargin ||
        config.verticalExtent <= config.trackingMargin + config.numericalMargin ||
        reference.getDegree() != 5 || reference.getControlPoints().rows() < 6 ||
        reference.getKnots().size() != reference.getControlPoints().rows() + 6 ||
        reference.getKnots()[5] != 0.0 || reference.getKnots()[reference.getControlPoints().rows()] != 1.0 ||
        !reference.getControlPoints().allFinite()) {
        throw std::invalid_argument("Invalid corridor extent, normalized spline or subdivision count.");
    }
    TerrainSafetyCorridor corridor;
    corridor.footRadius = config.footRadius;
    corridor.numericalMargin = config.numericalMargin;
    corridor.start = config.contactPhase;
    corridor.end = 1.0 - config.contactPhase;
    Eigen::MatrixXd controls = reference.getControlPoints();
    controls.col(2).array() += config.footRadius;
    Eigen::Vector2d direction = (controls.bottomRows(1) - controls.topRows(1)).leftCols(2).transpose();
    direction = direction.norm() > 1e-9 ? direction.normalized().eval() : Eigen::Vector2d::UnitX().eval();
    const Eigen::Vector3d forward(direction.x(), direction.y(), 0.0);
    const Eigen::Vector3d lateral(-direction.y(), direction.x(), 0.0);
    try {
        const auto segments = pieces(reference, config, subdivisionsPerSpan);
        std::vector<Eigen::MatrixXd> seeds;
        std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> localBoxes;
        Eigen::Vector2d queryLo = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
        Eigen::Vector2d queryHi = -queryLo;
        for (const auto& piece : segments) {
            seeds.push_back(hullMap(piece, basis) * controls);
            const auto& seed = seeds.back();
            // The oriented local box's XY AABB. Query outside it by the complete
            // dilation, otherwise an obstacle just outside the map query could
            // intersect the corridor after foot/map inflation.
            const double fLo = (seed * forward).minCoeff() - config.longitudinalExtent;
            const double fHi = (seed * forward).maxCoeff() + config.longitudinalExtent;
            const double lLo = (seed * lateral).minCoeff() - config.lateralExtent;
            const double lHi = (seed * lateral).maxCoeff() + config.lateralExtent;
            Eigen::Vector2d lo = Eigen::Vector2d::Constant(std::numeric_limits<double>::infinity());
            Eigen::Vector2d hi = -lo;
            for (double f : {fLo, fHi}) {
                for (double l : {lLo, lHi}) {
                    const Eigen::Vector2d corner = (f * forward + l * lateral).head<2>();
                    lo = lo.cwiseMin(corner);
                    hi = hi.cwiseMax(corner);
                }
            }
            localBoxes.emplace_back(lo, hi);
            queryLo = queryLo.cwiseMin(lo);
            queryHi = queryHi.cwiseMax(hi);
        }
        const double dilation = config.footRadius + config.mapMargin;
        const Eigen::Vector2d padding = Eigen::Vector2d::Constant(dilation);
        // One immutable occupied-prism snapshot per swing. Neighboring cells
        // overlap heavily; repeated interpolation queries dominated online cost.
        const auto prisms = terrain.getPrisms(queryLo - padding, queryHi + padding);
        if (prisms.empty()) throw TerrainUnavailable("Empty terrain coverage.");
        for (const auto& prism : prisms) {
            if (!prism.minXY.allFinite() || !prism.maxXY.allFinite() || !std::isfinite(prism.height) ||
                (prism.minXY.array() > prism.maxXY.array()).any()) {
                throw TerrainUnavailable("Invalid occupied terrain prism.");
            }
        }
        for (size_t index = 0; index < segments.size(); ++index) {
            const auto& piece = segments[index];
            CorridorPolytope cell;
            cell.start = piece.start;
            cell.end = piece.end;
            cell.hullMapping = hullMap(piece, basis);
            const Eigen::MatrixXd& seed = seeds[index];
            for (const auto& axis : {forward, lateral, Eigen::Vector3d(Eigen::Vector3d::UnitZ())}) {
                const double extent = axis.z() != 0.0 ? config.verticalExtent :
                    (std::abs(axis.dot(forward)) > 0.5 ? config.longitudinalExtent : config.lateralExtent);
                const Eigen::VectorXd projection = seed * axis;
                addPlane(cell, axis, projection.maxCoeff() + extent, config.trackingMargin);
                addPlane(cell, -axis, -projection.minCoeff() + extent, config.trackingMargin);
            }
            const Eigen::Vector2d& lo = localBoxes[index].first;
            const Eigen::Vector2d& hi = localBoxes[index].second;
            const Eigen::Vector3d seedLo = seed.colwise().minCoeff().transpose();
            const Eigen::Vector3d seedHi = seed.colwise().maxCoeff().transpose();
            for (const auto& prism : prisms) {
                if ((prism.maxXY.array() + dilation < lo.array()).any() ||
                    (prism.minXY.array() - dilation > hi.array()).any()) continue;
                const Eigen::Vector2d inflatedLo = prism.minXY - Eigen::Vector2d::Constant(dilation);
                const Eigen::Vector2d inflatedHi = prism.maxXY + Eigen::Vector2d::Constant(dilation);
                // Try lateral/longitudinal separation of the ENTIRE fixed XY
                // hull. Choose the largest gap, never an individually safe vertex.
                Eigen::Vector3d normal = -Eigen::Vector3d::UnitZ();
                double bound = -(prism.height + dilation) - config.geometricClearance;
                double bestGap = config.trackingMargin + config.numericalMargin;
                for (int axis = 0; axis < 2; ++axis) {
                    for (double sign : {-1.0, 1.0}) {
                        const Eigen::Vector3d candidate = sign * Eigen::Vector3d::Unit(axis);
                        const double face = sign > 0.0 ? inflatedLo[axis] : -inflatedHi[axis];
                        const double candidateBound = face - config.geometricClearance;
                        const double gap = candidateBound - (sign > 0.0 ? seedHi[axis] : -seedLo[axis]);
                        if (gap > bestGap) {
                            bestGap = gap;
                            normal = candidate;
                            bound = candidateBound;
                        }
                    }
                }
                addPlane(cell, normal, bound, config.trackingMargin);
            }
            // XY seed hull is inside every lateral planning face by construction.
            // A feasible vertical interval is therefore sufficient for nonemptiness.
            double bottom = -std::numeric_limits<double>::infinity();
            double top = std::numeric_limits<double>::infinity();
            for (const auto& plane : cell.halfSpaces) {
                const double bound = plane.planningOffset - config.numericalMargin;
                if (plane.normal.z() < -0.5) bottom = std::max(bottom, -bound);
                if (plane.normal.z() > 0.5) top = std::min(top, bound);
            }
            if (bottom >= top) {
                corridor.failureReason = "Empty planning cell at phase " + std::to_string(cell.start);
                return corridor;
            }
            corridor.cells.push_back(std::move(cell));
        }
    } catch (const TerrainUnavailable& e) {
        corridor.failureReason = e.what();
        return corridor;
    }
    corridor.valid = !corridor.cells.empty();
    return corridor;
}

CorridorLinearConstraints corridorControlPointConstraints(const TerrainSafetyCorridor& corridor) {
    if (!corridor.valid || corridor.cells.empty()) {
        throw std::invalid_argument("Cannot export an invalid corridor.");
    }
    const int n = corridor.cells.front().hullMapping.cols();
    size_t count = 0;
    for (const auto& cell : corridor.cells) count += 6 * cell.halfSpaces.size();
    CorridorLinearConstraints result{Eigen::MatrixXd::Zero(count, 3 * n), Eigen::VectorXd(count)};
    int row = 0;
    for (const auto& cell : corridor.cells) {
        for (const auto& plane : cell.halfSpaces) {
            for (int vertex = 0; vertex < 6; ++vertex, ++row) {
                for (int axis = 0; axis < 3; ++axis) {
                    result.A.block(row, axis * n, 1, n) = plane.normal[axis] * cell.hullMapping.row(vertex);
                }
                result.b[row] = plane.planningOffset - corridor.numericalMargin -
                    plane.normal.z() * corridor.footRadius;
            }
        }
    }
    return result;
}

CorridorAccelerationConstraint corridorHocbfConstraint(const CorridorHalfSpace& plane,
                                                       const Eigen::Vector3d& centre,
                                                       const Eigen::Vector3d& velocity,
                                                       double gain1, double gain2,
                                                       double disturbanceSupport) {
    if (!centre.allFinite() || !velocity.allFinite() || !std::isfinite(gain1) || !std::isfinite(gain2) ||
        gain1 <= 0.0 || gain2 <= 0.0 || !std::isfinite(disturbanceSupport) || disturbanceSupport < 0.0) {
        throw std::invalid_argument("Invalid HOCBF state, gains or disturbance support.");
    }
    const double h = plane.geometricOffset - plane.normal.dot(centre);
    return {-plane.normal, (gain1 + gain2) * plane.normal.dot(velocity) - gain1 * gain2 * h +
                             disturbanceSupport};
}

std::vector<Eigen::Vector3d> corridorVertices(const CorridorPolytope& cell, bool planning) {
    std::vector<Eigen::Vector3d> result;
    const auto& planes = cell.halfSpaces;
    auto bound = [planning](const CorridorHalfSpace& p) {
        return planning ? p.planningOffset : p.geometricOffset;
    };
    for (size_t i = 0; i < planes.size(); ++i) {
        for (size_t j = i + 1; j < planes.size(); ++j) {
            for (size_t k = j + 1; k < planes.size(); ++k) {
                Eigen::Matrix3d A;
                A << planes[i].normal.transpose(), planes[j].normal.transpose(), planes[k].normal.transpose();
                const Eigen::FullPivLU<Eigen::Matrix3d> lu(A);
                if (!lu.isInvertible()) continue;
                const Eigen::Vector3d p = lu.solve(Eigen::Vector3d(bound(planes[i]), bound(planes[j]), bound(planes[k])));
                bool inside = true;
                for (const auto& plane : planes) inside &= plane.normal.dot(p) <= bound(plane) + 1e-9;
                for (const auto& previous : result) inside &= (p - previous).norm() > 1e-8;
                if (inside) result.push_back(p);
            }
        }
    }
    return result;
}
}  // namespace ocs2::legged_robot
