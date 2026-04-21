/******************************************************************************
 * CLF-CBF QP Swing Trajectory Optimizer
 *
 * 将摆动相足端视为双积分器(质点)模型，以预设的五次样条参考轨迹构建 CLF（稳定性），
 * 以地形高程信息构建 HOCBF（安全约束），通过求解小规模二次规划问题(QP)获得
 * 修正后的加速度，从而在线生成既跟踪参考又绝对避碰的轨迹。
 *
 * 系统动力学：      ṗ = v,  v̇ = a  (双积分器)
 * CBF：            h(p) = p_z - z_terrain(p_x, p_y) - d_safe
 * HOCBF 约束：     ∇h^T a + v^T ∇²h v + k₂ ∇h^T v + k₁ h ≥ 0
 * CLF 约束：       ṽ^T Kv a ≤ -γV + δ + ṽ^T Kv a_ref - p̃^T Kp ṽ  (Kp, Kv 为对角矩阵)
 * QP 目标：        min  ½‖a - a_ref‖² + ρ δ²
 ******************************************************************************/

#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <memory>
#include <vector>
#include <qpOASES.hpp>
#include <grid_map_filters_rsl/GridMapDerivative.hpp>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <ocs2_core/Types.h>
#include <ocs2_legged_robot/common/Types.h>
#include <grid_map_core/GridMap.hpp>
#include <convex_plane_decomposition/PlanarRegion.h>

#include "ocs2_quadruped_controller/interface/constraint/QuinticSplineCpg.h"
#include "ocs2_quadruped_controller/interface/constraint/BSpline.h"
#include "ocs2_quadruped_controller/perceptive/synchronize/EdgeSphere.h"

namespace ocs2::legged_robot {

/**
 * @brief 摆动轨迹 QP 参数配置
 */
struct SwingTrajectoryQpConfig {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // B 样条参数
    int numControlPoints = 15;
    int numCurveSamples = 300;
    int splineOrder = 3;


    double sphereActivationDist = 0.01;  // 激活球约束的距离阈值 (m)
    int maxSpheresPerQp = 5;          // 每个 QP 中考虑的最大球约束数量 


    // QP 权重
    double controlPointTrackingWeight = 0.2;    // 抬升量 L2 惩罚，避免无约束区域整体抬高
    double positionTrackingWeight = 0.5;        // 平滑抬升包络跟踪权重
    double velocityTrackingWeight = 0.15;       // 抬升包络一阶差分跟踪权重
    double accelerationTrackingWeight = 0.05;  // 抬升包络二阶差分跟踪权重
    double jerkSmoothnessWeight = 0.01;         // 抬升曲线三阶差分平滑权重
    double controlPointSmoothnessWeight = 0.1; // 控制点抬升量二阶差分平滑权重
    double controlPointJerkWeight = 0.02;      // 控制点抬升量三阶差分平滑权重
    double controlPointZMin = -0.0;
    double controlPointZMax = 0.3;
    double groundClearance = 1e-3;
    double sphereSafetyMargin = 0.02;         // 球障碍额外安全裕度
    double hessianRegularization = 1e-8;      // Hessian 对角正则项

    int ignoreSamples = 5;        ///< 忽略起始和结束的采样点数，避免过于接近地面或触地点导致的数值问题
    int maxActiveSetIterations = 80;
    int qpMaxWorkingSetRecalculations = 4000;
    int constraintSeedRadius = 2;      // 先激活最难约束点附近的样点
    int constraintExpandStep = 1;      // 约束区域向两侧扩展的步长
    int constraintBandRadius = 1;      // 每次激活违反约束点时，同时激活其邻域样点
    int liftPropagationWindow = 12;    // 将热点约束向两侧传播成平滑抬升包络的样点半径
    double liftPropagationSigma = 4.0; // 热点抬升向两侧衰减的高斯尺度（样点）

    // 地形梯度计算
    double epsilon = 0.005;      ///< 导数器不可用时的数值差分步长 (m)
    std::string elevationLayer = "smooth_planar";  ///< 高程图层名

    // 调试
    bool verbose = true;           ///< 是否输出调试日志
};

struct CandidateConstraint {
    Eigen::RowVectorXd row;
    double lowerBound = 0.0;
    int sampleIndex = -1;
    int type = 0;
    double referenceDeficit = 0.0;
};

/**
 * @brief 地形信息接口 — 提供高程、梯度和曲率查询
 */
class TerrainInterface {
public:
    virtual ~TerrainInterface() = default;

    /// 查询 (x, y) 处的地形高度
    virtual double getHeight(double x, double y) const = 0;

    /// 查询 (x, y) 处的地形梯度 (dz/dx, dz/dy)
    virtual Eigen::Vector2d getGradient(double x, double y) const = 0;

    /// 查询 (x, y) 处的地形曲率（Hessian 矩阵的对角元素近似）
    virtual Eigen::Matrix2d getHessian(double x, double y) const = 0;
};

/**
 * @brief 基于 GridMap 的地形接口实现
 */
class GridMapTerrainInterface : public TerrainInterface {
public:
    /// 使用 PlanarTerrain 共享指针构造 (地形更新自动可见)
    GridMapTerrainInterface(std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
                            const std::string& elevationLayer,
                            double epsilon = 0.005);

    double getHeight(double x, double y) const override;
    Eigen::Vector2d getGradient(double x, double y) const override;
    Eigen::Matrix2d getHessian(double x, double y) const override;

private:
    std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
    std::string elevationLayer_;
    double epsilon_;
};

/**
 * @brief CLF-CBF QP 摆动轨迹优化器
 *
 * 给定参考轨迹(由 QuinticSplineCpg 生成)和地形信息，通过逐时间步求解 QP
 * 获得修正后的加速度序列，再积分得到平滑且避碰的优化轨迹。
 */
class SwingTrajectoryQpOptimizer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 优化后的轨迹数据
    struct OptimizedTrajectory {
        std::shared_ptr<BSpline> bspline;     ///< 优化后的 B 样条
        scalar_t startTime = 0.0;
        scalar_t endTime = 0.0;
        bool collisionFree = false;
        bool valid() const { return collisionFree && static_cast<bool>(bspline); }
    };


    SwingTrajectoryQpOptimizer(SwingTrajectoryQpConfig config, std::shared_ptr<TerrainInterface> terrainInterface);

    /// 注入 EdgeSphere 数据 (从 PerceptiveLeggedInterface 获取)
    void setEdgeSpheres(std::shared_ptr<EdgeSphereArray> edgeSpheresPtr) { edgeSpheresPtr_ = std::move(edgeSpheresPtr); }

    /**
     * @brief 优化单条腿的摆动轨迹 
     */
    OptimizedTrajectory optimizeSwingTrajectory(
        const QuinticSplineCpg& refSpline,
        scalar_t swingStartTime,
        scalar_t swingFinalTime,
        const Eigen::Vector3d& liftOffPos3d,
        const Eigen::Vector3d& touchDownPos3d,
        const std::vector<const EdgeSphere*>& nearbySpheres) const;

    
    bool solveQp(const std::vector<int> &activeSet, Eigen::VectorXd &solution,
                                          const Eigen::MatrixXd &H_eigen,
                                          const Eigen::VectorXd &g_eigen,
                                          const Eigen::VectorXd &refZ, const int nV,
                                          const Eigen::VectorXd &lb_eigen, const Eigen::VectorXd &ub_eigen,
                                          const std::vector<CandidateConstraint> &candidates) const;


    const SwingTrajectoryQpConfig& getConfig() const { return config_; }


    // 筛选轨迹路径附近的 EdgeSphere
    std::vector<const EdgeSphere*> filterNearbySpheres(
        const Eigen::Vector3d& liftOffPos3d,
        const Eigen::Vector3d& touchDownPos3d) const;

        
    std::shared_ptr<TerrainInterface> terrainInterface_;
    SwingTrajectoryQpConfig config_;


private:

    /// 生成 XY 参考轨迹 (线性插值 liftOff → touchDown)
    Eigen::Vector3d interpolateXY(const Eigen::Vector3d& liftOff,
                                  const Eigen::Vector3d& touchDown,
                                  scalar_t t, scalar_t t0, scalar_t tf) const;
    
    void sampleReferenceTrajectoryToControlPoints(const QuinticSplineCpg &refSpline,
                                                scalar_t swingStartTime, scalar_t swingFinalTime,
                                                const Eigen::Vector3d &liftOffPos3d, const Eigen::Vector3d &touchDownPos3d,
                                                std::vector<Eigen::Vector3d> &controlPoints) const;

    Eigen::VectorXd computeTerrainHeightProfile(const Eigen::VectorXd& sampleX,
                                           const Eigen::VectorXd& sampleY) const;

    Eigen::VectorXd computeSphereHeightProfile(const Eigen::VectorXd& sampleX,
                                           const Eigen::VectorXd& sampleY, 
                                           const std::vector<const EdgeSphere*>& nearbySpheres) const;
    
    Eigen::VectorXd projectReferenceToBounds(const Eigen::VectorXd& refZ, const int nV, 
                                            const Eigen::VectorXd& lb_eigen, const Eigen::VectorXd& ub_eigen) const;

    bool optimizeControlPointsZ(const Eigen::MatrixXd& basisMat,
                            const Eigen::VectorXd& refControlZ,
                            const Eigen::VectorXd& refCurveZ,
                            const Eigen::VectorXd& sampleX,
                            const Eigen::VectorXd& terrainHeights,
                            const Eigen::VectorXd& sphereHeights,
                            Eigen::VectorXd& xOpt) const;


    std::shared_ptr<EdgeSphereArray> edgeSpheresPtr_;  ///< 地形边缘球体
};

}  // namespace ocs2::legged_robot
