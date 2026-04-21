#pragma once

#include <memory>
#include <Eigen/Core>
#include <ocs2_legged_robot/common/Types.h>
#include "ocs2_quadruped_controller/perceptive/synchronize/EdgeSphere.h"
#include "ocs2_quadruped_controller/interface/constraint/QuinticSplineCpg.h"
#include "ocs2_quadruped_controller/interface/constraint/QuinticSpline.h"

namespace ocs2::legged_robot {

/**
 * 摆动相轨迹碰撞检测器。
 *
 * 将摆动相轨迹离散化为若干采样点（XY线性插值，Z取自五次样条），
 * 检测每个采样点是否落入地形边缘的 EdgeSphere 内部。
 * 若碰撞，则通过迭代式调整 midHeight 和端点高度偏移使轨迹无碰撞。
 *
 * Phase 1: 迭代碰撞验证 — 调整 midHeight 后重建样条再次检测，最多迭代 maxIterations 次。
 * Phase 2: 端点感知 — 碰撞发生在端点附近时，额外输出 liftOff/touchDown 高度偏移。
 */
class SwingTrajectoryCollisionChecker {
public:
    struct Config {
        double safetyMargin = 0.03;       // 安全裕度 (m)
        int numSamples = 20;              // 离散采样数
        int maxIterations = 5;            // 迭代碰撞检测最大次数
        double amplificationFactor = 1.5; // midHeight 调整放大系数
        double earlyPhaseThreshold = 0.25;  // 早期区域阈值 (alpha < 此值)
        double latePhaseThreshold = 0.75;   // 晚期区域阈值 (alpha > 此值)
    };

    struct DebugInfo {
        bool hasNearbySphere = false;
        bool hasCollision = false;
        int nearbySphereCount = 0;
        int collidingSampleIndex = -1;
        double maxPenetration = 0.0;
        double collisionTime = 0.0;
        double collisionAlpha = 0.0;    // 碰撞处的归一化时间
        Eigen::Vector3d collisionPosition = Eigen::Vector3d::Zero();
        Eigen::Vector3d sphereCenter = Eigen::Vector3d::Zero();
        double sphereRadius = 0.0;
        int iterationsUsed = 0;         // 迭代收敛所用次数
    };

    /** 碰撞避障调整结果（midHeight + 端点高度偏移） */
    struct CollisionFreeResult {
        scalar_t midHeight;
        scalar_t liftOffHeightOffset = 0.0;   // 抬脚高度补偿 (>=0)
        scalar_t touchDownHeightOffset = 0.0;  // 落脚高度补偿 (>=0)
    };

    SwingTrajectoryCollisionChecker(Config config, std::shared_ptr<EdgeSphereArray> edgeSpheresPtr);

    /**
     * 对单个摆动段进行碰撞检测与轨迹调整。
     *
     * 迭代式调整 midHeight，同时对端点附近碰撞产生高度偏移。
     *
     * @return CollisionFreeResult 包含调整后的 midHeight 及端点高度偏移量
     */
    CollisionFreeResult computeCollisionFreeTrajectory(
        const Eigen::Vector3d& liftOffPos3d,
        const Eigen::Vector3d& touchDownPos3d,
        scalar_t swingStartTime,
        scalar_t swingFinalTime,
        scalar_t liftOffHeight,
        scalar_t touchDownHeight,
        scalar_t liftOffVelocity,
        scalar_t touchDownVelocity,
        scalar_t originalMidHeight,
        DebugInfo* debugInfo = nullptr) const;

private:
    /** 碰撞检测内部结果 */
    struct CollisionDetail {
        double maxPenetration = 0.0;
        double worstAlpha = 0.0;           // 最大穿透点的归一化时间
        double earlyMaxPenetration = 0.0;  // 早期区域 (alpha < earlyPhaseThreshold) 最大穿透
        double lateMaxPenetration = 0.0;   // 晚期区域 (alpha > latePhaseThreshold) 最大穿透
        double midMaxPenetration = 0.0;    // 中间区域最大穿透
    };

    /**
     * 检测给定参数的轨迹是否与 edge spheres 碰撞。
     * 返回分区域穿透信息。
     */
    CollisionDetail checkCollisionDetailed(
        const Eigen::Vector3d& liftOffPos3d,
        const Eigen::Vector3d& touchDownPos3d,
        scalar_t swingStartTime,
        scalar_t swingFinalTime,
        scalar_t liftOffHeight,
        scalar_t touchDownHeight,
        scalar_t liftOffVelocity,
        scalar_t touchDownVelocity,
        scalar_t midHeight,
        DebugInfo* debugInfo = nullptr) const;

    /**
     * 预筛选轨迹 XY 包围盒附近的 edge spheres。
     */
    std::vector<const EdgeSphere*> filterNearbySpheres(
        const Eigen::Vector3d& liftOffPos3d,
        const Eigen::Vector3d& touchDownPos3d) const;

    Config config_;
    std::shared_ptr<EdgeSphereArray> edgeSpheresPtr_;
};

}  // namespace ocs2::legged_robot
