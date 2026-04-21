#include "ocs2_quadruped_controller/perceptive/constraint/EdgeAwareFootholdScorer.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

namespace ocs2::legged_robot {

namespace {

struct SphereClearanceSample {
    double clearance = std::numeric_limits<double>::infinity();
    double weight = 1.0;
};

double smoothPositivePart(double value, double smoothingDistance) {
    const double epsilon = std::max(smoothingDistance, 1e-6);
    return 0.5 * (value + std::sqrt(value * value + epsilon * epsilon));
}

}  // namespace

EdgeAwareFootholdScorer::EdgeAwareFootholdScorer(
    Config config, std::shared_ptr<EdgeSphereArray> edgeSpheresPtr)
    : config_(config), edgeSpheresPtr_(std::move(edgeSpheresPtr)) {}

double EdgeAwareFootholdScorer::computePenalty(const Eigen::Vector3d& candidatePosition) const {
    if (!edgeSpheresPtr_ || edgeSpheresPtr_->empty()) {
        return 0.0;
    }

    std::vector<SphereClearanceSample> candidates;
    candidates.reserve(edgeSpheresPtr_->size());

    for (const auto& sphere : *edgeSpheresPtr_) {
        if (!sphere.center.allFinite() || !std::isfinite(sphere.radius)) {
            continue;
        }

        const double clearance = computeClearance(sphere, candidatePosition);
        const double weight = computeVerticalWeight(sphere, candidatePosition);
        if (!std::isfinite(clearance) || weight <= 0.0) {
            continue;
        }

        candidates.push_back({clearance, weight});
    }

    if (candidates.empty()) {
        return 0.0;
    }

    const size_t maxConsidered = std::max<size_t>(1, config_.maxConsideredSpheres);
    if (candidates.size() > maxConsidered) {
        std::nth_element(candidates.begin(), candidates.begin() + static_cast<std::ptrdiff_t>(maxConsidered),
                         candidates.end(),
                         [](const SphereClearanceSample& lhs, const SphereClearanceSample& rhs) {
                             return lhs.clearance < rhs.clearance;
                         });
        candidates.resize(maxConsidered);
    }

    const double smoothingDistance = std::max(config_.smoothingDistance, 1e-4);
    const double influenceDistance = config_.safeMargin + 2.0 * smoothingDistance;
    double weightedPenaltySum = 0.0;
    double weightSum = 0.0;
    for (const auto& candidate : candidates) {
        if (candidate.clearance > influenceDistance) {
            continue;
        }

        const double distanceBeyondContact = std::max(candidate.clearance, 0.0);
        const double localityWeight = std::exp(
            -0.5 * distanceBeyondContact * distanceBeyondContact / (smoothingDistance * smoothingDistance));
        const double normalizedWeight = std::max(candidate.weight * localityWeight, 1e-6);
        const double smoothViolation = smoothPositivePart(config_.safeMargin - candidate.clearance, smoothingDistance);
        weightedPenaltySum += normalizedWeight * smoothViolation * smoothViolation;
        weightSum += normalizedWeight;
    }

    if (weightedPenaltySum <= 0.0 || weightSum <= 0.0) {
        return 0.0;
    }

    const double penalty = config_.penaltyWeight * (weightedPenaltySum / weightSum);
    return std::min(std::max(penalty, 0.0), std::max(config_.maxPenalty, 0.0));
}

std::function<double(const Eigen::Vector3d&)> EdgeAwareFootholdScorer::asPenaltyFunction() const {
    // 捕获 this（scorer 生命周期由调用方保证）
    return [this](const Eigen::Vector3d& position) -> double {
        return computePenalty(position);
    };
}

double EdgeAwareFootholdScorer::computeClearance(const EdgeSphere& sphere,
                                                 const Eigen::Vector3d& candidatePosition) const {
    double distance = 0.0;
    if (config_.useXYDistanceOnly) {
        const Eigen::Vector2d diff = candidatePosition.head<2>() - sphere.center.head<2>();
        distance = diff.norm();
    } else {
        distance = (candidatePosition - sphere.center).norm();
    }

    const double inflatedRadius = std::max(sphere.radius + config_.sphereRadiusInflation, 1e-6);
    return distance - inflatedRadius;
}

double EdgeAwareFootholdScorer::computeVerticalWeight(const EdgeSphere& sphere,
                                                      const Eigen::Vector3d& candidatePosition) const {
    if (!config_.useXYDistanceOnly) {
        return 1.0;
    }

    const double scale = std::max(config_.verticalWeightScale, 1e-6);
    const double dz = candidatePosition.z() - sphere.center.z();
    return std::exp(-0.5 * dz * dz / (scale * scale));
}

}  // namespace ocs2::legged_robot
