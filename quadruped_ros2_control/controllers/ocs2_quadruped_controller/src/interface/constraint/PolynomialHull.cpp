#include "ocs2_quadruped_controller/interface/constraint/PolynomialHull.h"

#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ocs2::legged_robot {
// Degree-5 MINVO coefficient matrix from MIT ACL's MINVO repository (BSD-3-Clause).
// Rows follow getA_MV.m's degree-5 order [3, 4, 2, 5, 1, 6], for s in [-1, 1].
// Copyright 2020 Jesus Tordesillas Torres, Aerospace Controls Laboratory, MIT.
// Redistribution and use in source and binary forms, with or without modification, are permitted
// provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright notice, this list of conditions
//    and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of
//    conditions and the following disclaimer in the documentation and/or other materials provided
//    with the distribution.
// 3. Neither the name of the copyright holder nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific prior written permission.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
// FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
// IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
// OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// Source: https://github.com/mit-acl/minvo.
const Eigen::Matrix<double, 6, 6> &minvoBasisMatrix() {
    static const Eigen::Matrix<double, 6, 6> matrix = [] {
        Eigen::Matrix<double, 6, 6> value;
        value << -0.739186417375602, 0.776885081198033, 0.330192274509370, -0.377284453092952,
            -0.036500738446832, 0.045894253207982, 1.503106410717892, -1.319318454373542, -1.365549927198963,
            1.333002683287254, -0.120976958190391, 0.002895296414827, -1.750065767219131, 0.542433373175509,
            2.776543561738747, -0.955718230194302, -1.064403387878015, 0.451210450377191, 1.750065767219131,
            0.542433373175509, -2.776543561738747, -0.955718230194302, 1.064403387878015, 0.451210450377191,
            -1.503106410717892, -1.319318454373542, 1.365549927198963, 1.333002683287254, 0.120976958190391,
            0.002895296414827, 0.739186417375602, 0.776885081198033, -0.330192274509370, -0.377284453092952,
            0.036500738446832, 0.045894253207982;
        return value;
    }();
    return matrix;
}

std::pair<PolynomialSegment, PolynomialSegment> PolynomialSegment::split(double time) const {
    const double ratio = (time - start) / (end - start);
    const int degree = bezierMap.rows() - 1;
    Eigen::MatrixXd work = bezierMap, left = work, right = work;
    for (int level = 1; level <= degree; ++level) {
        work.topRows(degree + 1 - level) =
            ((1 - ratio) * work.topRows(degree + 1 - level) + ratio * work.middleRows(1, degree + 1 - level))
                .eval();
        left.row(level) = work.row(0);
        right.row(degree - level) = work.row(degree - level);
    }
    const double mid = time;
    return {{start, mid, left}, {mid, end, right}};
}

std::vector<PolynomialSegment> extractPolynomialSegments(const BSpline &spline) {
    const int degree = spline.getDegree();
    Eigen::VectorXd knots = spline.getKnots();
    const int count = spline.getControlPoints().rows();
    if (degree != 5 || count < degree + 1 || !(knots[count] > knots[degree]) ||
        knots.head(degree + 1).minCoeff() != knots[degree] ||
        knots.tail(degree + 1).maxCoeff() != knots[count] ||
        !(knots.tail(knots.size() - 1) - knots.head(knots.size() - 1)).array().isFinite().all() ||
        (knots.tail(knots.size() - 1) - knots.head(knots.size() - 1)).minCoeff() < 0.0) {
        throw std::invalid_argument("Polynomial hull extraction requires a clamped quintic spline.");
    }
    Eigen::MatrixXd mapping = Eigen::MatrixXd::Identity(count, count);
    std::vector<double> internal;
    for (int i = degree + 1; i < count; ++i) {
        if (internal.empty() || knots[i] != internal.back()) {
            internal.push_back(knots[i]);
        }
    }
    // Exact knot insertion (no collocation or Vandermonde fit), including
    // nonuniform spans.
    for (double u : internal) {
        int multiplicity = static_cast<int>((knots.array() == u).count());
        for (; multiplicity < degree; ++multiplicity) {
            const int k = std::upper_bound(knots.data(), knots.data() + knots.size(), u) - knots.data() - 1;
            Eigen::MatrixXd next(mapping.rows() + 1, count);
            for (int i = 0; i <= k - degree; ++i) {
                next.row(i) = mapping.row(i);
            }
            for (int i = k - multiplicity + 1; i < next.rows(); ++i) {
                next.row(i) = mapping.row(i - 1);
            }
            for (int i = k - degree + 1; i <= k - multiplicity; ++i) {
                const double alpha = (u - knots[i]) / (knots[i + degree] - knots[i]);
                next.row(i) = alpha * mapping.row(i) + (1.0 - alpha) * mapping.row(i - 1);
            }
            Eigen::VectorXd nextKnots(knots.size() + 1);
            nextKnots.head(k + 1) = knots.head(k + 1);
            nextKnots[k + 1] = u;
            nextKnots.tail(knots.size() - k - 1) = knots.tail(knots.size() - k - 1);
            knots = std::move(nextKnots);
            mapping = std::move(next);
        }
    }
    std::vector<PolynomialSegment> result;
    for (int k = degree; k < mapping.rows(); ++k) {
        if (knots[k + 1] > knots[k]) {
            result.push_back({knots[k], knots[k + 1], mapping.middleRows(k - degree, degree + 1)});
        }
    }
    return result;
}

Eigen::MatrixXd hullMap(const PolynomialSegment &segment, HullBasis basis) {
    if (basis == HullBasis::Bezier) {
        return segment.bezierMap;
    }
    static const Eigen::Matrix<double, 6, 6> transform = [] {
        // Bernstein basis coefficients on [-1,1]:
        // binom(5,i)*(1+s)^i*(1-s)^(5-i)/32.
        const double choose[6][6] = {{1},          {1, 1},          {1, 2, 1},
                                     {1, 3, 3, 1}, {1, 4, 6, 4, 1}, {1, 5, 10, 10, 5, 1}};
        Eigen::Matrix<double, 6, 6> bernstein = Eigen::Matrix<double, 6, 6>::Zero();
        for (int i = 0; i <= 5; ++i) {
            for (int a = 0; a <= i; ++a) {
                for (int b = 0; b <= 5 - i; ++b) {
                    bernstein(i, 5 - a - b) +=
                        choose[5][i] * choose[i][a] * choose[5 - i][b] * (b % 2 ? -1.0 : 1.0) / 32.0;
                }
            }
        }
        return Eigen::Matrix<double, 6, 6>(
            minvoBasisMatrix().transpose().fullPivLu().solve(bernstein.transpose()));
    }();
    return transform * segment.bezierMap;
}
}  // namespace ocs2::legged_robot
