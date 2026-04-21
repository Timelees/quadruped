#pragma once

#include <memory>
#include <functional>
#include <Eigen/Core>
#include "ocs2_quadruped_controller/perceptive/synchronize/EdgeSphere.h"

namespace ocs2::legged_robot {

/**
 * 边缘感知的落足点评分器。
 *
 * 为候选落足点计算"边缘惩罚"值，鼓励落足点远离地形边缘 (EdgeSphere)。
 * 设计为解耦模块，可插入 ConvexRegionSelector 的 penaltyFunction。
 *
 * 惩罚逻辑：
 *   对候选点 p，找到最近的 EdgeSphere i，计算水平距离 d_xy。
 *   若 d_xy < safeMargin，则施加平滑二次惩罚:
 *     penalty = weight * (1 - d_xy / safeMargin)²
 *   否则 penalty = 0
 */
class EdgeAwareFootholdScorer {
public:
    struct Config {
        double safeMargin = 0.02;     // 安全距离阈值 (m)，小于此距离开始惩罚
        double penaltyWeight = 2.0;   // 惩罚权重 (需要 >> 距离代价²，以有效阻止边缘落足)
        bool useXYDistanceOnly = true; // 仅使用 XY 平面距离（忽略高度差）
    };

    EdgeAwareFootholdScorer(Config config, std::shared_ptr<EdgeSphereArray> edgeSpheresPtr);

    /**
     * 计算候选落足点的边缘惩罚值。
     * @param candidatePosition 候选落足点世界坐标 (3D)
     * @return 惩罚值 >= 0，越大表示越接近边缘
     */
    double computePenalty(const Eigen::Vector3d& candidatePosition) const;

    /**
     * 生成可直接传给 getBestPlanarRegionAtPositionInWorld 的 penaltyFunction lambda。
     * 返回的 lambda 签名: double(const Eigen::Vector3d&)
     */
    std::function<double(const Eigen::Vector3d&)> asPenaltyFunction() const;

    /** 更新 EdgeSphere 数据 */
    void updateEdgeSpheres(std::shared_ptr<EdgeSphereArray> edgeSpheresPtr) {
        edgeSpheresPtr_ = std::move(edgeSpheresPtr);
    }

private:
    Config config_;
    std::shared_ptr<EdgeSphereArray> edgeSpheresPtr_;
};

}  // namespace ocs2::legged_robot
