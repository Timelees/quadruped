#pragma once

#include <utility>

#include "ocs2_quadruped_controller/interface/constraint/BSpline.h"

namespace ocs2::legged_robot {

enum class HullBasis { Bezier = 0, Minvo = 1 };

// Rows map the original spline control points to a Bezier segment, u in [0,1].
struct PolynomialSegment {
    double start, end;
    Eigen::MatrixXd bezierMap;
    std::pair<PolynomialSegment, PolynomialSegment> split(double time) const;
    std::pair<PolynomialSegment, PolynomialSegment> bisect() const { return split(0.5 * (start + end)); }
};

std::vector<PolynomialSegment> extractPolynomialSegments(const BSpline& spline);
Eigen::MatrixXd hullMap(const PolynomialSegment& segment, HullBasis basis);
const Eigen::Matrix<double, 6, 6>& minvoBasisMatrix();  // descending powers on [-1,1]

// Per-axis Bernstein bounds in the spline's physical time, rows v/a/jerk.
Eigen::Matrix3d swingDerivativeBounds(const BSpline& spline);

}  // namespace ocs2::legged_robot
