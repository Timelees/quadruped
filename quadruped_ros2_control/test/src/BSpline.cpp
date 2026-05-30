#include "BSpline.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

void BSpline::validateSplineData() const {
    if (x_.size() == 0 || y_.size() == 0 || z_.size() == 0) {
        throw std::runtime_error("Control points are not set.");
    }
    if (x_.size() != y_.size() || x_.size() != z_.size()) {
        throw std::runtime_error("Control point arrays must have the same length.");
    }
    if (knots_.size() != x_.size() + degree_ + 1) {
        throw std::runtime_error("Knot vector size is inconsistent with control points and degree.");
    }
}

double BSpline::sanitizeParameter(double t, const Eigen::VectorXd& knots) const {
    const double leftEndpoint = knots[0];
    const double rightEndpoint = knots[knots.size() - 1];
    const double clamped = std::min(std::max(t, leftEndpoint), rightEndpoint);
    return (clamped >= rightEndpoint) ? std::nextafter(rightEndpoint, leftEndpoint) : clamped;
}

BSpline::BSpline(int degree) : degree_(degree) {}

void BSpline::setKnots(const Eigen::VectorXd& knots) {
    knots_ = knots;
}

void BSpline::setControlPoints(const Eigen::VectorXd& x, const Eigen::VectorXd& y, const Eigen::VectorXd& z) {
    x_ = x;
    y_ = y;
    z_ = z;
    if (x_.size() != y_.size() || x_.size() != z_.size()) {
        throw std::invalid_argument("x, y, and z must have the same length.");
    }
}

Eigen::VectorXd BSpline::generateKnots(int numControlPoints, double start, double end, int method) const {
    const int order = degree_ + 1;
    if (numControlPoints < order) {
        throw std::invalid_argument("Number of control points must be at least degree + 1.");
    }

    if (method == 1) {
        return Eigen::VectorXd::LinSpaced(numControlPoints + order, start, end);
    }

    Eigen::VectorXd knots(numControlPoints + order);
    knots.head(order - 1).setConstant(start);
    knots.tail(order - 1).setConstant(end);
    knots.segment(order - 1, numControlPoints - order + 2) =
        Eigen::VectorXd::LinSpaced(numControlPoints - order + 2, start, end);
    return knots;
}

Eigen::MatrixXd BSpline::generateSplineBasisInternal(const Eigen::VectorXd& pts,
                                                     int degree,
                                                     const Eigen::VectorXd& knots) const {
    const int numParams = pts.size();
    const int numKnots = knots.size();
    const int order = degree + 1;

    if (numKnots <= 1) {
        std::cerr << "Error: Number of knots must be greater than 1." << std::endl;
        return Eigen::MatrixXd();
    }
    if (numKnots < order + 1) {
        throw std::invalid_argument("Degree is inconsistent with knot vector.");
    }

    const double rightEndpoint = knots[numKnots - 1];
    const double leftEndpoint = knots[0];

    Eigen::MatrixXd basis = Eigen::MatrixXd::Zero(numParams, numKnots - 1);

    for (int i = 0; i < numKnots - 1; ++i) {
        for (int j = 0; j < numParams; ++j) {
            const double clamped = std::min(std::max(pts[j], leftEndpoint), rightEndpoint);
            const double u = (clamped >= rightEndpoint)
                                 ? std::nextafter(rightEndpoint, leftEndpoint)
                                 : clamped;
            if (u >= knots[i] && u < knots[i + 1]) {
                basis(j, i) = 1.0;
            }
        }
    }

    for (int currentOrder = 2; currentOrder <= order; ++currentOrder) {
        for (int i = 0; i < numKnots - currentOrder; ++i) {
            for (int j = 0; j < numParams; ++j) {
                const double clamped = std::min(std::max(pts[j], leftEndpoint), rightEndpoint);
                const double u = (clamped >= rightEndpoint)
                                     ? std::nextafter(rightEndpoint, leftEndpoint)
                                     : clamped;
                const double leftCoeff = (knots[i + currentOrder - 1] != knots[i])
                                             ? (u - knots[i]) / (knots[i + currentOrder - 1] - knots[i])
                                             : 0.0;
                const double rightCoeff = (knots[i + currentOrder] != knots[i + 1])
                                              ? (knots[i + currentOrder] - u) / (knots[i + currentOrder] - knots[i + 1])
                                              : 0.0;
                basis(j, i) = leftCoeff * basis(j, i) + rightCoeff * basis(j, i + 1);
            }
        }
    }

    return basis.leftCols(numKnots - order);
}

Eigen::MatrixXd BSpline::generateSplineBasis(const Eigen::VectorXd& pts) const {
    validateSplineData();
    return generateSplineBasisInternal(pts, degree_, knots_);
}

Eigen::Vector3d BSpline::evaluateCurve(const Eigen::VectorXd& xControl,
                                       const Eigen::VectorXd& yControl,
                                       const Eigen::VectorXd& zControl,
                                       int degree,
                                       const Eigen::VectorXd& knots,
                                       double t) const {
    Eigen::VectorXd singleParam(1);
    singleParam[0] = sanitizeParameter(t, knots);
    const Eigen::MatrixXd basis = generateSplineBasisInternal(singleParam, degree, knots);
    return Eigen::Vector3d((basis * xControl)[0], (basis * yControl)[0], (basis * zControl)[0]);
}

Eigen::Vector3d BSpline::getPosition(double t) const {
    validateSplineData();
    return evaluateCurve(x_, y_, z_, degree_, knots_, t);
}

Eigen::Vector3d BSpline::evaluateDerivative(int derivativeOrder, double t) const {
    validateSplineData();
    if (derivativeOrder <= 0) {
        return getPosition(t);
    }

    Eigen::VectorXd derivativeX = x_;
    Eigen::VectorXd derivativeY = y_;
    Eigen::VectorXd derivativeZ = z_;
    Eigen::VectorXd derivativeKnots = knots_;
    int derivativeDegree = degree_;

    for (int order = 0; order < derivativeOrder; ++order) {
        if (derivativeDegree <= 0 || derivativeX.size() < 2) {
            return Eigen::Vector3d::Zero();
        }

        Eigen::VectorXd nextX(derivativeX.size() - 1);
        Eigen::VectorXd nextY(derivativeY.size() - 1);
        Eigen::VectorXd nextZ(derivativeZ.size() - 1);
        for (int i = 0; i < nextX.size(); ++i) {
            const double denominator = derivativeKnots[i + derivativeDegree + 1] - derivativeKnots[i + 1];
            const double scale = (std::abs(denominator) > 1e-12)
                                     ? static_cast<double>(derivativeDegree) / denominator
                                     : 0.0;
            nextX[i] = scale * (derivativeX[i + 1] - derivativeX[i]);
            nextY[i] = scale * (derivativeY[i + 1] - derivativeY[i]);
            nextZ[i] = scale * (derivativeZ[i + 1] - derivativeZ[i]);
        }

        derivativeX = nextX;
        derivativeY = nextY;
        derivativeZ = nextZ;
        derivativeKnots = derivativeKnots.segment(1, derivativeKnots.size() - 2);
        --derivativeDegree;
    }

    return evaluateCurve(derivativeX, derivativeY, derivativeZ, derivativeDegree, derivativeKnots, t);
}

Eigen::Vector3d BSpline::getVelocity(double t) const {
    return evaluateDerivative(1, t);
}

Eigen::Vector3d BSpline::getAcceleration(double t) const {
    return evaluateDerivative(2, t);
}

Eigen::VectorXd BSpline::getSplineX(const Eigen::VectorXd& pts) const {
    validateSplineData();
    const Eigen::MatrixXd basis = generateSplineBasisInternal(pts, degree_, knots_);
    return basis * x_;
}

Eigen::VectorXd BSpline::getSplineY(const Eigen::VectorXd& pts) const {
    validateSplineData();
    const Eigen::MatrixXd basis = generateSplineBasisInternal(pts, degree_, knots_);
    return basis * y_;
}

Eigen::VectorXd BSpline::getSplineZ(const Eigen::VectorXd& pts) const {
    validateSplineData();
    const Eigen::MatrixXd basis = generateSplineBasisInternal(pts, degree_, knots_);
    return basis * z_;
}

Eigen::VectorXd BSpline::interpolate(const Eigen::VectorXd& u) const {
    const Eigen::VectorXd xx = getSplineX(u);
    const Eigen::VectorXd zz = getSplineZ(u);
    Eigen::VectorXd v(u.size());

    for (int i = 0; i < u.size(); ++i) {
        int idx = static_cast<int>(std::upper_bound(xx.data(), xx.data() + xx.size(), u[i]) - xx.data()) - 1;
        if (idx < 0) {
            idx = 0;
        }
        if (idx >= xx.size() - 1) {
            idx = xx.size() - 2;
        }

        const double denominator = xx[idx + 1] - xx[idx];
        double alpha = 0.0;
        if (std::abs(denominator) > 1e-12) {
            alpha = (u[i] - xx[idx]) / denominator;
        }

        v[i] = (1.0 - alpha) * zz[idx] + alpha * zz[idx + 1];
    }

    return v;
}
