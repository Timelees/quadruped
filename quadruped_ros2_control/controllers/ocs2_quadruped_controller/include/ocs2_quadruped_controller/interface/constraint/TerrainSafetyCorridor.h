#pragma once

#include <Eigen/Core>
#include <limits>
#include <string>
#include <vector>

#include "ocs2_quadruped_controller/interface/constraint/PolynomialHull.h"
#include "ocs2_quadruped_controller/interface/constraint/TerrainGeometry.h"

namespace ocs2::legged_robot {

// All corridor planes use the SPHERE CENTRE in the map frame, a^T p <= b.
// The controller's spline uses the foot contact reference: p_c = p_ref + r*e_z.
struct CorridorHalfSpace {
    Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();  // unit length
    double geometricOffset = 0.0;
    double planningOffset = 0.0;  // geometricOffset - support_E(normal)
    double basisMargin = 0.0;
};

struct CorridorPolytope {
    double start = 0.0, end = 0.0;  // normalized swing phase, closed interval
    std::vector<CorridorHalfSpace> halfSpaces;
    Eigen::MatrixXd hullMapping;  // six hull vertices from original spline controls
};

struct TerrainMapTiming {
    double stamp = std::numeric_limits<double>::quiet_NaN(); // controller time
    double validUntil = std::numeric_limits<double>::infinity();
};

struct CorridorConfig {
    double lateralExtent = 0.15;
    double longitudinalExtent = 0.04;
    double verticalExtent = 0.08;
    double footRadius = 0.019;
    double terrainRiseRate = 0.0;   // upper height variation [m/s]
    double heightError = 0.0;       // one-sided physical elevation error [m]
    double mapMargin = 0.0;          // isotropic map position error, obstacle dilation
    double geometricClearance = 0.001;  // strict gap to the inflated obstacle
    double trackingMargin = 0.02;  // Euclidean position error ball
    double numericalMargin = 1e-7; // additional distance reserved in QP rows
    // Contact transitions retain the controller's existing contact-envelope rows.
    // Only [contactPhase, 1-contactPhase] has the non-contact corridor certificate.
    double contactPhase = 0.10;
    // Target support footprint window for approach mode only. Noncontact
    // builders never exclude the target. Approach retains a no-penetration row.
    double landingPositionWindow = 0.025;
};

struct TerrainSafetyCorridor {
    std::vector<CorridorPolytope> cells;
    std::vector<CorridorPolytope> approachCells;  // execution only, target nonpenetration replaces clearance
    double footRadius = 0.0;
    double numericalMargin = 0.0;
    Eigen::Vector3d touchdownCentre = Eigen::Vector3d::Zero();
    Eigen::Vector3d touchdownVelocity = Eigen::Vector3d::Zero();
    TerrainMapTiming mapTiming;
    double mapToTouchdown = 0.0;
    double mapHeightMargin = 0.0;
    double start = 0.0, end = 0.0;
    bool valid = false;
    std::string failureReason;
};

// normalizedReference must be a quintic spline on [0,1]. Each nonzero knot span
// is split exactly (de Casteljau); every cell excludes ALL occupied prisms in its
// bounded map query, including the floor and obstacles beside the nominal path.
// guardrail: conservative box dilation of a sphere; no lateral route search.
// A Z-only consumer can reject infeasible cells, but must never delete a plane.
TerrainSafetyCorridor buildTerrainSafetyCorridor(const TerrainInterface& terrain,
                                                 const BSpline& normalizedReference,
                                                 const CorridorConfig& config,
                                                 int subdivisionsPerSpan = 4,
                                                 HullBasis basis = HullBasis::Minvo,
                                                 TerrainMapTiming map = {}, double touchdownTime = 0.0);

// Build only a remaining normalized interval. Past/contact cells must not veto
// a certificate for the future noncontact section after a terrain update.
TerrainSafetyCorridor buildTerrainSafetyCorridor(const TerrainInterface& terrain,
    const BSpline& normalizedReference, const CorridorConfig& config, int subdivisionsPerSpan,
    HullBasis basis, TerrainMapTiming map, double touchdownTime, double start, double end);

// Build the execution-only terminal cells from the accepted reference. The tip
// position is in contact coordinates. No changes to the planning certificate.
void buildLandingCorridor(TerrainSafetyCorridor& corridor, const TerrainInterface& terrain,
                          const BSpline& reference, const CorridorConfig& config,
                          double swingStart, double swingEnd, double positionWindow);

// Block control order [qx(0..N-1), qy(0..N-1), qz(0..N-1)], FOOT REFERENCE
// coordinates. Includes the sphere-centre translation and numerical reserve.
// A*q <= b is exactly a_r^T (G_i*q + r*e_z) <= b_plan-rho, for all six vertices.
struct CorridorLinearConstraints {
    Eigen::MatrixXd A;
    Eigen::VectorXd b;
};
CorridorLinearConstraints corridorControlPointConstraints(const TerrainSafetyCorridor& corridor);

// Geometric-set barrier, for double-integrator sphere-centre dynamics p''=u+w:
// accelerationNormal.dot(u) >= lowerBound. Disturbance support bounds a^T w.
// This is an export, not a claim of WBC feasibility or a running safety filter.
struct CorridorAccelerationConstraint {
    Eigen::Vector3d accelerationNormal;
    double lowerBound;
};
CorridorAccelerationConstraint corridorHocbfConstraint(const CorridorHalfSpace& plane,
                                                       const Eigen::Vector3d& centre,
                                                       const Eigen::Vector3d& velocity,
                                                       double gain1, double gain2,
                                                       double disturbanceSupport = 0.0);

// Geometric or planning vertices for RViz/export, computed from plane triples.
std::vector<Eigen::Vector3d> corridorVertices(const CorridorPolytope& cell, bool planning);
}  // namespace ocs2::legged_robot
