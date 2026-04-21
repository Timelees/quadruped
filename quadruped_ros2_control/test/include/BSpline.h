#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

class BSpline {
public:
    explicit BSpline(int degree = 3);

    void setKnots(const Eigen::VectorXd& knots);
    void setControlPoints(const Eigen::VectorXd& x, const Eigen::VectorXd& y, const Eigen::VectorXd& z);

    Eigen::VectorXd generateKnots(int numControlPoints, double start, double end, int method = 0) const;
    Eigen::MatrixXd generateSplineBasis(const Eigen::VectorXd& pts) const;

    Eigen::Vector3d getPosition(double t) const;
    Eigen::Vector3d getVelocity(double t) const;
    Eigen::Vector3d getAcceleration(double t) const;

    Eigen::VectorXd getSplineX(const Eigen::VectorXd& pts) const;
    Eigen::VectorXd getSplineY(const Eigen::VectorXd& pts) const;
    Eigen::VectorXd getSplineZ(const Eigen::VectorXd& pts) const;
    Eigen::VectorXd interpolate(const Eigen::VectorXd& xQuery) const;

private:
    Eigen::MatrixXd generateSplineBasisInternal(const Eigen::VectorXd& pts,
                                                int degree,
                                                const Eigen::VectorXd& knots) const;
    Eigen::Vector3d evaluateCurve(const Eigen::VectorXd& xControl,
                                  const Eigen::VectorXd& yControl,
                                  const Eigen::VectorXd& zControl,
                                  int degree,
                                  const Eigen::VectorXd& knots,
                                  double t) const;
    Eigen::Vector3d evaluateDerivative(int derivativeOrder, double t) const;
    double sanitizeParameter(double t, const Eigen::VectorXd& knots) const;
    void validateSplineData() const;

    int degree_;
    Eigen::VectorXd knots_;
    Eigen::VectorXd x_;
    Eigen::VectorXd y_;
    Eigen::VectorXd z_;
};
