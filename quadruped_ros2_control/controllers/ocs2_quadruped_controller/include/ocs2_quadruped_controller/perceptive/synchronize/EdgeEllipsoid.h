#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <limits>
#include <vector>

namespace ocs2::legged_robot {

struct EdgeEllipsoid {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Matrix3d orientation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d radii = Eigen::Vector3d::Ones();

  Eigen::Vector3d inflatedRadii(double inflation) const {
    return (radii.array() + inflation).max(1e-3).matrix();
  }

  double levelSetValue(const Eigen::Vector3d& position, double inflation = 0.0) const {
    const Eigen::Vector3d scaled = orientation.transpose() * (position - center);
    const Eigen::Vector3d denom = inflatedRadii(inflation);
    return scaled.cwiseQuotient(denom).squaredNorm() - 1.0;
  }

  Eigen::Vector3d gradient(const Eigen::Vector3d& position, double inflation = 0.0) const {
    const Eigen::Vector3d local = orientation.transpose() * (position - center);
    const Eigen::Vector3d denom = inflatedRadii(inflation);
    const Eigen::Vector3d localGradient = 2.0 * local.cwiseQuotient(denom.cwiseProduct(denom));
    return orientation * localGradient;
  }
};

using EdgeEllipsoidArray = std::vector<EdgeEllipsoid>;

inline size_t findClosestEllipsoid(const EdgeEllipsoidArray& ellipsoids, const Eigen::Vector3d& position,
                                   double inflation = 0.0) {
  size_t bestIndex = 0;
  double bestValue = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < ellipsoids.size(); ++i) {
    const double value = ellipsoids[i].levelSetValue(position, inflation);
    if (value < bestValue) {
      bestValue = value;
      bestIndex = i;
    }
  }
  return bestIndex;
}

}  // namespace ocs2::legged_robot