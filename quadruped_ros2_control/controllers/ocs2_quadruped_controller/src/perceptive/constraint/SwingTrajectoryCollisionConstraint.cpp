#include "ocs2_quadruped_controller/perceptive/constraint/SwingTrajectoryCollisionChecker.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace ocs2::legged_robot {

SwingTrajectoryCollisionChecker::SwingTrajectoryCollisionChecker(
    Config config, std::shared_ptr<EdgeSphereArray> edgeSpheresPtr)
    : config_(config), edgeSpheresPtr_(std::move(edgeSpheresPtr)) {}

std::vector<const EdgeSphere*> SwingTrajectoryCollisionChecker::filterNearbySpheres(
    const Eigen::Vector3d& liftOffPos3d,
    const Eigen::Vector3d& touchDownPos3d) const
{
    const double xMin = std::min(liftOffPos3d.x(), touchDownPos3d.x());
    const double xMax = std::max(liftOffPos3d.x(), touchDownPos3d.x());
    const double yMin = std::min(liftOffPos3d.y(), touchDownPos3d.y());
    const double yMax = std::max(liftOffPos3d.y(), touchDownPos3d.y());

    std::vector<const EdgeSphere*> nearbySpheres;
    for (const auto& sphere : *edgeSpheresPtr_) {
        const double margin = sphere.radius + config_.safetyMargin;
        if (sphere.center.x() >= xMin - margin && sphere.center.x() <= xMax + margin &&
            sphere.center.y() >= yMin - margin && sphere.center.y() <= yMax + margin) {
            nearbySpheres.push_back(&sphere);
        }
    }
    return nearbySpheres;
}

SwingTrajectoryCollisionChecker::CollisionFreeResult
SwingTrajectoryCollisionChecker::computeCollisionFreeTrajectory(
    const Eigen::Vector3d& liftOffPos3d,
    const Eigen::Vector3d& touchDownPos3d,
    scalar_t swingStartTime,
    scalar_t swingFinalTime,
    scalar_t liftOffHeight,
    scalar_t touchDownHeight,
    scalar_t liftOffVelocity,
    scalar_t touchDownVelocity,
    scalar_t originalMidHeight,
    DebugInfo* debugInfo) const
{
    CollisionFreeResult result;
    result.midHeight = originalMidHeight;
    result.liftOffHeightOffset = 0.0;
    result.touchDownHeightOffset = 0.0;

    if (debugInfo != nullptr) {
        *debugInfo = DebugInfo{};
    }

    if (!edgeSpheresPtr_ || edgeSpheresPtr_->empty()) {
        return result;
    }

    // 当前工作参数（迭代过程中会被修改）
    scalar_t currentMidHeight = originalMidHeight;
    scalar_t currentLiftOffHeight = liftOffHeight;
    scalar_t currentTouchDownHeight = touchDownHeight;

    for (int iter = 0; iter < config_.maxIterations; ++iter) {
        CollisionDetail detail = checkCollisionDetailed(
            liftOffPos3d, touchDownPos3d,
            swingStartTime, swingFinalTime,
            currentLiftOffHeight, currentTouchDownHeight,
            liftOffVelocity, touchDownVelocity,
            currentMidHeight, (iter == 0) ? debugInfo : nullptr);

        if (debugInfo != nullptr) {
            debugInfo->iterationsUsed = iter + 1;
        }

        // 更新 debugInfo 中的碰撞信息（取最后一次有碰撞的迭代）
        if (debugInfo != nullptr) {
            debugInfo->hasCollision = true;
            debugInfo->maxPenetration = detail.maxPenetration;
            debugInfo->collisionAlpha = detail.worstAlpha;
        }

        const double amp = config_.amplificationFactor;
        const double margin = config_.safetyMargin;

        // Phase 2: 根据碰撞区域分别调整
        // 早期碰撞 (alpha < earlyPhaseThreshold): 提高 liftOff 端点
        if (detail.earlyMaxPenetration > 0.0) {
            currentLiftOffHeight += detail.earlyMaxPenetration * amp + margin;
        }
        // 晚期碰撞 (alpha > latePhaseThreshold): 提高 touchDown 端点
        if (detail.lateMaxPenetration > 0.0) {
            currentTouchDownHeight += detail.lateMaxPenetration * amp + margin;
        }
        // 中间碰撞: 提升 midHeight
        if (detail.midMaxPenetration > 0.0) {
            currentMidHeight += detail.midMaxPenetration * amp + margin;
        }

        // 如果碰撞仅发生在端点区域但 mid 区域无碰撞，仍需确保 midHeight 不低于端点
        // （端点抬高后 maxHeight 基准也会变化）
        const scalar_t newMaxBase = std::max(currentLiftOffHeight, currentTouchDownHeight);
        if (currentMidHeight < newMaxBase) {
            currentMidHeight = newMaxBase;
        }
    }

    result.midHeight = currentMidHeight;
    result.liftOffHeightOffset = currentLiftOffHeight - liftOffHeight;
    result.touchDownHeightOffset = currentTouchDownHeight - touchDownHeight;


    return result;
}

SwingTrajectoryCollisionChecker::CollisionDetail
SwingTrajectoryCollisionChecker::checkCollisionDetailed(
    const Eigen::Vector3d& liftOffPos3d,
    const Eigen::Vector3d& touchDownPos3d,
    scalar_t swingStartTime,
    scalar_t swingFinalTime,
    scalar_t liftOffHeight,
    scalar_t touchDownHeight,
    scalar_t liftOffVelocity,
    scalar_t touchDownVelocity,
    scalar_t midHeight,
    DebugInfo* debugInfo) const
{
    CollisionDetail detail;

    // 构建临时五次样条来获取 Z(t)
    const QuinticSpline::Node liftOff{swingStartTime, liftOffHeight, liftOffVelocity, 0.0};
    const QuinticSpline::Node touchDown{swingFinalTime, touchDownHeight, touchDownVelocity, 0.0};
    const QuinticSplineCpg spline(liftOff, midHeight, touchDown);

    const double dt = swingFinalTime - swingStartTime;
    const int N = config_.numSamples;

    // 预筛选轨迹附近的 sphere
    const auto nearbySpheres = filterNearbySpheres(liftOffPos3d, touchDownPos3d);

    if (debugInfo != nullptr) {
        debugInfo->hasNearbySphere = !nearbySpheres.empty();
        debugInfo->nearbySphereCount = static_cast<int>(nearbySpheres.size());
    }

    if (nearbySpheres.empty()) {
        return detail;
    }

    for (int i = 0; i <= N; ++i) {
        const double alpha = static_cast<double>(i) / N;
        const double t = swingStartTime + alpha * dt;

        // XY 线性插值，Z 从五次样条取
        const Eigen::Vector3d footPos(
            liftOffPos3d.x() + alpha * (touchDownPos3d.x() - liftOffPos3d.x()),
            liftOffPos3d.y() + alpha * (touchDownPos3d.y() - liftOffPos3d.y()),
            spline.position(t));

        const EdgeSphere* closestSphere = nullptr;
        double minDistSquared = std::numeric_limits<double>::infinity();

        for (const auto* sphere : nearbySpheres) {
            const double distSquared = (footPos - sphere->center).squaredNorm();
            if (distSquared < minDistSquared) {
                minDistSquared = distSquared;
                closestSphere = sphere;
            }
        }

        if (closestSphere == nullptr) {
            continue;
        }

        const double dist = std::sqrt(minDistSquared);
        const double penetration = closestSphere->radius - dist;

        if (penetration <= 0.0) {
            continue;
        }

        // 每个采样点仅使用离该轨迹点最近的 sphere 进行碰撞统计。
        if (alpha < config_.earlyPhaseThreshold) {
            detail.earlyMaxPenetration = std::max(detail.earlyMaxPenetration, penetration);
        } else if (alpha > config_.latePhaseThreshold) {
            detail.lateMaxPenetration = std::max(detail.lateMaxPenetration, penetration);
        } else {
            detail.midMaxPenetration = std::max(detail.midMaxPenetration, penetration);
        }

        if (penetration > detail.maxPenetration) {
            detail.maxPenetration = penetration;
            detail.worstAlpha = alpha;
            if (debugInfo != nullptr) {
                debugInfo->hasCollision = true;
                debugInfo->collidingSampleIndex = i;
                debugInfo->maxPenetration = penetration;
                debugInfo->collisionTime = t;
                debugInfo->collisionAlpha = alpha;
                debugInfo->collisionPosition = footPos;
                debugInfo->sphereCenter = closestSphere->center;
                debugInfo->sphereRadius = closestSphere->radius;
            }
        }
    }
    return detail;
}

}  // namespace ocs2::legged_robot
