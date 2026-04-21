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
        double safeMargin = 0.03;          // 到障碍表面的目标安全距离阈值 (m)
        double penaltyWeight = 0.6;        // 距离平方代价缩放，需与投影距离项同量级
        double sphereRadiusInflation = 0.005; // 对 EdgeSphere 半径做额外膨胀，提升保守性
        double smoothingDistance = 0.015;  // 平滑正部和邻域权重的过渡距离 (m)
        double maxPenalty = 0.003;         // 惩罚上限，避免边缘代价压倒投影距离代价
        double verticalWeightScale = 0.05; // 与候选点高度相差较大的球体权重衰减尺度
        size_t maxConsideredSpheres = 3;   // 最多参与聚合的近邻球体数量
        bool useXYDistanceOnly = true;     // 仅使用 XY 平面距离（忽略高度差）
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
    double computeClearance(const EdgeSphere& sphere, const Eigen::Vector3d& candidatePosition) const;
    double computeVerticalWeight(const EdgeSphere& sphere, const Eigen::Vector3d& candidatePosition) const;

    Config config_;
    std::shared_ptr<EdgeSphereArray> edgeSpheresPtr_;
};

}  // namespace ocs2::legged_robot
