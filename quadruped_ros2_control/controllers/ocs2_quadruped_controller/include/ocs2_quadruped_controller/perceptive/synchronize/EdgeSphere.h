#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace ocs2::legged_robot {

struct EdgeSphere {
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  double radius = 0.03;  // sphere radius in meters

  /// Level-set: positive outside, negative inside the sphere.
  double levelSetValue(const Eigen::Vector3d& position, double inflation = 0.0) const {
    const double r = std::max(radius + inflation, 1e-6);
    return (position - center).norm() / r - 1.0;
  }

  /// Gradient of the level-set in world frame.
  Eigen::Vector3d gradient(const Eigen::Vector3d& position, double inflation = 0.0) const {
    const Eigen::Vector3d diff = position - center;
    const double dist = diff.norm();
    if (dist < 1e-10) {
      return Eigen::Vector3d::Zero();
    }
    const double r = std::max(radius + inflation, 1e-6);
    return diff / (dist * r);
  }
};

using EdgeSphereArray = std::vector<EdgeSphere>;

inline size_t findClosestSphere(const EdgeSphereArray& spheres, const Eigen::Vector3d& position,
                             double inflation = 0.0) {
  size_t bestIndex = 0;
  double bestValue = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < spheres.size(); ++i) {
    const double value = spheres[i].levelSetValue(position, inflation);
    if (value < bestValue) {
      bestValue = value;
      bestIndex = i;
    }
  }
  return bestIndex;
}

}  // namespace ocs2::legged_robot
