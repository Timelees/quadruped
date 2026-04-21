#include "ocs2_quadruped_controller/perceptive/constraint/EdgeAwareFootholdScorer.h"

#include <cmath>
#include <limits>
#include <iostream>

namespace ocs2::legged_robot {

EdgeAwareFootholdScorer::EdgeAwareFootholdScorer(
    Config config, std::shared_ptr<EdgeSphereArray> edgeSpheresPtr)
    : config_(config), edgeSpheresPtr_(std::move(edgeSpheresPtr)) {}

double EdgeAwareFootholdScorer::computePenalty(const Eigen::Vector3d& candidatePosition) const {
    if (!edgeSpheresPtr_ || edgeSpheresPtr_->empty()) {
        return 0.0;
    }

    // 找到距候选点最近的 EdgeSphere
    double minDist = std::numeric_limits<double>::infinity();

    for (const auto& sphere : *edgeSpheresPtr_) {
        double dist;
        if (config_.useXYDistanceOnly) {
            // 仅考虑 XY 平面距离
            const Eigen::Vector2d diff(
                candidatePosition.x() - sphere.center.x(),
                candidatePosition.y() - sphere.center.y());
            dist = diff.norm();
        } else {
            dist = (candidatePosition - sphere.center).norm();
        }
        minDist = std::min(minDist, dist);
    }

    // std::cout << "current nominal foothold: " << candidatePosition.transpose()
    //           << ", minDist to edge: " << minDist << std::endl;

    // 安全距离内施加平滑二次惩罚
    if (minDist >= config_.safeMargin) {
        return 0.0;
    }

    const double t = 1.0 - minDist / config_.safeMargin;  // t ∈ (0, 1]
    return config_.penaltyWeight * t * t;
}

std::function<double(const Eigen::Vector3d&)> EdgeAwareFootholdScorer::asPenaltyFunction() const {
    // 捕获 this（scorer 生命周期由调用方保证）
    return [this](const Eigen::Vector3d& position) -> double {
        return computePenalty(position);
    };
}

}  // namespace ocs2::legged_robot
