/******************************************************************************
 * CLF-CBF QP 摆动轨迹优化测试
 *
 * 场景 (XZ 平面):
 *   - liftOff  = (0, 0, 0)       起飞位置
 *   - touchDown = (0.02, 0, 0.10) 着地位置 (10cm 台阶上方)
 *   - 地形: x < 0.01 时 z=0, x >= 0.01 时 z=0.10 (台阶边缘在 x=0.01)
 *   - 台阶边缘处放置 EdgeSphere 作为碰撞约束
 *   - 参考轨迹 midHeight 设得较低，使参考轨迹穿越台阶边缘
 *   - CLF-CBF QP 优化器应生成一条绕开边缘球的安全轨迹
 *
 *            |----.-------|  z=0.10
 *            |        (台阶面)
 *  ____._____|               z=0.00
 *         x=0.01
 *
 * 输出:
 *   - 终端打印碰撞分析统计
 *   - 生成 CSV + Python 可视化 (参考轨迹 / 优化轨迹 / 地形 / EdgeSphere)
 ******************************************************************************/

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <qpOASES.hpp>
#include <algorithm>
#include <cmath>
#include <limits>

#include "ocs2_quadruped_controller/interface/constraint/QuinticSplineCpg.h"
#include "ocs2_quadruped_controller/perceptive/synchronize/EdgeSphere.h"

// ============================================================================
// 数据结构
// ============================================================================

struct FootState {
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
};

struct QpResult {
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    double slack = 0.0;
    bool feasible = false;
};

struct OptimizedTrajectory {
    std::vector<double> times;
    std::vector<Eigen::Vector3d> positions;
    std::vector<Eigen::Vector3d> velocities;
    std::vector<Eigen::Vector3d> accelerations;
    bool collisionFree = false;
};

struct TrajectorySample {
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
};

struct TrajectoryValidation {
    bool collisionFree = true;
    double maxHeightDeficit = 0.0;
};

struct ClfCbfConfig {
    // CBF 参数  alpha1接近2根号alpha0
    double alpha0 = 2.0;        // 对距离障碍本身的惩罚强度
    double alpha1 = 2.828;        // 对接近障碍速度的惩罚强度


    // CLF 参数
    double Kp = 6.0;      // clf位置误差权重
    double Kv = 2.0;      // clf速度误差权重
    double gamma = 0.5;   // clf指数衰减率

    // Qp  权重
    double accelTrackWeight = 1.0;   // 跟参考加速度贴得多紧
    double accelSmoothWeight = 20.0;    // 加速度平滑权重
    double rho = 200.0;                 // 松弛变量权重

    // 约束参数
    double a_min = -1.0;
    double a_max = 1.0;



    double dt = 0.001;      // 时间步长
    int numSamples = 100;   // 离散采样点数量


    double sphereActivationDist = 0.05;     // 球障碍激活距离

    int postSmoothIterations = 300;
    double postSmoothTrackWeight = 1.0;
    double postSmoothNeighborWeight = 12.0;


    bool verbose = true;
};

// ============================================================================
// 台阶地形: x < edgeX 时 z=0, x >= edgeX 时 z=stepHeight
// ============================================================================
class StepTerrain {
public:
    StepTerrain(double edgeX, double stepHeight)
        : edgeX_(edgeX), stepHeight_(stepHeight) {}

    double getHeight(double x, double /*y*/) const {
        return (x >= edgeX_) ? stepHeight_ : 0.0;
    }

    Eigen::Vector2d getGradient(double x, double /*y*/) const {
        (void)x;
        return Eigen::Vector2d::Zero();
    }

    Eigen::Matrix2d getHessian(double x, double /*y*/) const {
        (void)x;
        return Eigen::Matrix2d::Zero();
    }

    double edgeX() const { return edgeX_; }
    double stepHeight() const { return stepHeight_; }

private:
    double edgeX_, stepHeight_;
};

// ============================================================================
// 全局状态
// ============================================================================
static ClfCbfConfig config_;
static StepTerrain* terrain = nullptr;
static ocs2::legged_robot::EdgeSphere stepEdgeSphere;

// ============================================================================
// 辅助函数
// ============================================================================

Eigen::Vector3d interpolateXY(const Eigen::Vector3d& liftOff,
                               const Eigen::Vector3d& touchDown,
                               double t, double t0, double tf) {
    const double alpha = (tf > t0) ? (t - t0) / (tf - t0) : 0.0;
    return liftOff + alpha * (touchDown - liftOff);
}


double smoothStep(double x) {
    const double clamped = std::max(0.0, std::min(x, 1.0));
    return clamped * clamped * (3.0 - 2.0 * clamped);
}


double computeExactSafeHeightAtXY(const Eigen::Vector2d& xy, double terrainHeight,
                                  const std::vector<const ocs2::legged_robot::EdgeSphere*>& edgeSpheres) {
    double safeHeight = terrainHeight;
    for (const auto* sphere : edgeSpheres) {
        const Eigen::Vector2d deltaXY = xy - sphere->center.head<2>();
        const double horizontalDist2 = deltaXY.squaredNorm();
        const double radius2 = sphere->radius * sphere->radius;
        if (horizontalDist2 >= radius2) {
            continue;
        }

        const double upperSurface = sphere->center.z() + std::sqrt(radius2 - horizontalDist2);
        safeHeight = std::max(safeHeight, upperSurface);
    }
    return safeHeight;
}


double computeSmoothBiasHeightAtXY(const Eigen::Vector2d& xy, double terrainHeight,
                                   const std::vector<const ocs2::legged_robot::EdgeSphere*>& edgeSpheres) {
    double safeHeight = terrainHeight;
    for (const auto* sphere : edgeSpheres) {
        const Eigen::Vector2d deltaXY = xy - sphere->center.head<2>();
        const double horizontalDist = deltaXY.norm();
        const double influenceRadius = sphere->radius + config_.sphereActivationDist;
        if (horizontalDist >= influenceRadius) {
            continue;
        }

        const double normalizedDist = horizontalDist / std::max(influenceRadius, 1e-6);
        const double bellWeight = 0.5 * (1.0 + std::cos(M_PI * normalizedDist));
        const double peakHeight = std::max(terrainHeight, sphere->center.z() + sphere->radius);
        const double blendedHeight = terrainHeight + bellWeight * (peakHeight - terrainHeight);
        safeHeight = std::max(safeHeight, blendedHeight);
    }
    return safeHeight;
}

TrajectorySample sampleTrajectoryAtTime(const OptimizedTrajectory& traj, double time) {
    TrajectorySample sample;
    if (traj.times.empty() || traj.positions.empty() || traj.velocities.empty() || traj.accelerations.empty()) {
        return sample;
    }

    if (time <= traj.times.front()) {
        sample.position = traj.positions.front().z();
        sample.velocity = traj.velocities.front().z();
        sample.acceleration = traj.accelerations.front().z();
        return sample;
    }

    if (time >= traj.times.back()) {
        sample.position = traj.positions.back().z();
        sample.velocity = traj.velocities.back().z();
        sample.acceleration = traj.accelerations.back().z();
        return sample;
    }

    const auto upper = std::upper_bound(traj.times.begin(), traj.times.end(), time);
    const size_t upperIndex = static_cast<size_t>(std::distance(traj.times.begin(), upper));
    const size_t lowerIndex = upperIndex - 1;
    const double t0 = traj.times[lowerIndex];
    const double t1 = traj.times[upperIndex];
    const double alpha = (t1 > t0) ? (time - t0) / (t1 - t0) : 0.0;

    sample.position = traj.positions[lowerIndex].z() + alpha * (traj.positions[upperIndex].z() - traj.positions[lowerIndex].z());
    sample.velocity = traj.velocities[lowerIndex].z() + alpha * (traj.velocities[upperIndex].z() - traj.velocities[lowerIndex].z());
    sample.acceleration = traj.accelerations[lowerIndex].z() + alpha * (traj.accelerations[upperIndex].z() - traj.accelerations[lowerIndex].z());
    return sample;
}

double computeRequiredSafeHeightAtTime(double time,
                                       double swingStartTime,
                                       double swingFinalTime,
                                       const Eigen::Vector3d& liftOffPos3d,
                                       const Eigen::Vector3d& touchDownPos3d,
                                       const std::vector<const ocs2::legged_robot::EdgeSphere*>& edgeSpheres) {
    const Eigen::Vector3d xyPoint = interpolateXY(liftOffPos3d, touchDownPos3d, time, swingStartTime, swingFinalTime);
    const double terrainHeight = terrain->getHeight(xyPoint.x(), xyPoint.y());
    return computeExactSafeHeightAtXY(xyPoint.head<2>(), terrainHeight, edgeSpheres);
}

OptimizedTrajectory samplePiecewiseQuinticTrajectory(
    const ocs2::legged_robot::QuinticSpline& leftSpline,
    const ocs2::legged_robot::QuinticSpline& rightSpline,
    double swingStartTime,
    double swingFinalTime,
    double midTime,
    const Eigen::Vector3d& liftOffPos3d,
    const Eigen::Vector3d& touchDownPos3d,
    int numSteps) {

    OptimizedTrajectory traj;
    if (numSteps <= 0 || swingFinalTime <= swingStartTime) {
        return traj;
    }

    const double dt = (swingFinalTime - swingStartTime) / numSteps;
    traj.times.reserve(numSteps + 1);
    traj.positions.reserve(numSteps + 1);
    traj.velocities.reserve(numSteps + 1);
    traj.accelerations.reserve(numSteps + 1);

    const Eigen::Vector3d xyDisplacement = touchDownPos3d - liftOffPos3d;
    const double duration = swingFinalTime - swingStartTime;
    const Eigen::Vector3d xyVelocity(xyDisplacement.x() / duration,
                                     xyDisplacement.y() / duration,
                                     0.0);

    for (int step = 0; step <= numSteps; ++step) {
        const double time = swingStartTime + step * dt;
        const double clampedTime = std::min(time, swingFinalTime);
        const Eigen::Vector3d xyPoint = interpolateXY(liftOffPos3d, touchDownPos3d, clampedTime, swingStartTime, swingFinalTime);
        const bool useLeftSpline = (clampedTime <= midTime);
        const auto& spline = useLeftSpline ? leftSpline : rightSpline;

        Eigen::Vector3d position = xyPoint;
        position.z() = spline.position(clampedTime);
        Eigen::Vector3d velocity(xyVelocity.x(), xyVelocity.y(), spline.velocity(clampedTime));
        Eigen::Vector3d acceleration(0.0, 0.0, spline.acceleration(clampedTime));

        traj.times.push_back(clampedTime);
        traj.positions.push_back(position);
        traj.velocities.push_back(velocity);
        traj.accelerations.push_back(acceleration);
    }

    if (!traj.positions.empty()) {
        traj.positions.front() = liftOffPos3d;
        traj.positions.back() = touchDownPos3d;
        traj.velocities.front().x() = xyVelocity.x();
        traj.velocities.front().y() = xyVelocity.y();
        traj.velocities.back().x() = xyVelocity.x();
        traj.velocities.back().y() = xyVelocity.y();
        traj.velocities.back().z() = 0.0;
        traj.accelerations.front().x() = 0.0;
        traj.accelerations.front().y() = 0.0;
        traj.accelerations.back().x() = 0.0;
        traj.accelerations.back().y() = 0.0;
        traj.accelerations.back().z() = 0.0;
    }

    return traj;
}

TrajectoryValidation validateTrajectoryAgainstSafety(
    const OptimizedTrajectory& traj,
    const std::vector<const ocs2::legged_robot::EdgeSphere*>& edgeSpheres) {

    TrajectoryValidation validation;
    for (const auto& p : traj.positions) {
        const double minSafeHeight = computeExactSafeHeightAtXY(p.head<2>(), terrain->getHeight(p.x(), p.y()), edgeSpheres);
        const double deficit = minSafeHeight - p.z();
        validation.maxHeightDeficit = std::max(validation.maxHeightDeficit, deficit);
        if (deficit > 1e-6) {
            validation.collisionFree = false;
            break;
        }

        for (const auto* sphere : edgeSpheres) {
            if ((p - sphere->center).norm() < sphere->radius - 1e-6) {
                validation.collisionFree = false;
                validation.maxHeightDeficit = std::max(validation.maxHeightDeficit, sphere->radius - (p - sphere->center).norm());
                break;
            }
        }

        if (!validation.collisionFree) {
            break;
        }
    }

    return validation;
}

OptimizedTrajectory reconstructCollisionFreeQuinticTrajectory(
    const OptimizedTrajectory& sampledTrajectory,
    const ocs2::legged_robot::QuinticSplineCpg& refSpline,
    double swingStartTime,
    double swingFinalTime,
    const Eigen::Vector3d& liftOffPos3d,
    const Eigen::Vector3d& touchDownPos3d,
    const std::vector<const ocs2::legged_robot::EdgeSphere*>& edgeSpheres) {

    if (sampledTrajectory.times.size() < 2) {
        return sampledTrajectory;
    }

    const double midTime = 0.5 * (swingStartTime + swingFinalTime);
    const TrajectorySample midSample = sampleTrajectoryAtTime(sampledTrajectory, midTime);
    double candidateMidHeight = std::max({midSample.position,
                                          refSpline.position(midTime),
                                          liftOffPos3d.z(),
                                          touchDownPos3d.z() + 1e-3});

    for (int i = 0; i <= config_.numSamples; ++i) {
        const double alpha = static_cast<double>(i) / std::max(config_.numSamples, 1);
        const double time = swingStartTime + alpha * (swingFinalTime - swingStartTime);
        candidateMidHeight = std::max(candidateMidHeight,
                                      computeRequiredSafeHeightAtTime(time, swingStartTime, swingFinalTime,
                                                                      liftOffPos3d, touchDownPos3d, edgeSpheres));
    }

    const ocs2::legged_robot::QuinticSpline::Node liftOffNode{
        swingStartTime, liftOffPos3d.z(), sampledTrajectory.velocities.front().z(), 0.0};
    const ocs2::legged_robot::QuinticSpline::Node touchDownNode{
        swingFinalTime, touchDownPos3d.z(), 0.0, 0.0};

    OptimizedTrajectory bestTrajectory = sampledTrajectory;
    TrajectoryValidation bestValidation = validateTrajectoryAgainstSafety(bestTrajectory, edgeSpheres);

    for (int iteration = 0; iteration < 8; ++iteration) {
        const ocs2::legged_robot::QuinticSpline::Node midNode{midTime, candidateMidHeight, 0.0, 0.0};
        const ocs2::legged_robot::QuinticSpline leftSpline(liftOffNode, midNode);
        const ocs2::legged_robot::QuinticSpline rightSpline(midNode, touchDownNode);

        OptimizedTrajectory candidateTrajectory = samplePiecewiseQuinticTrajectory(
            leftSpline, rightSpline, swingStartTime, swingFinalTime, midTime,
            liftOffPos3d, touchDownPos3d, static_cast<int>(sampledTrajectory.times.size() - 1));

        const TrajectoryValidation validation = validateTrajectoryAgainstSafety(candidateTrajectory, edgeSpheres);
        if (validation.collisionFree) {
            candidateTrajectory.collisionFree = true;
            std::cout << "[Spline fit] midTime=" << midTime
                      << " midHeight=" << candidateMidHeight
                      << " sourceMidHeight=" << midSample.position
                      << " sourceMidVel=" << midSample.velocity << std::endl;
            return candidateTrajectory;
        }

        bestTrajectory = candidateTrajectory;
        bestValidation = validation;
        candidateMidHeight += std::max(0.005, 1.25 * validation.maxHeightDeficit + 1e-3);
    }

    std::cout << "[Spline fit] failed to produce collision-free quintic spline, fallback to sampled QP trajectory. maxDeficit="
              << bestValidation.maxHeightDeficit << std::endl;
    bestTrajectory = sampledTrajectory;
    bestTrajectory.collisionFree = bestValidation.collisionFree && sampledTrajectory.collisionFree;
    return bestTrajectory;
}

double computeCBF(const Eigen::Vector3d& position,
                  const std::vector<const ocs2::legged_robot::EdgeSphere*>& edgeSpheres,
                  ocs2::legged_robot::EdgeSphere& nearestSphere) {
    double h_p = std::numeric_limits<double>::infinity();
    for (const auto* sphere : edgeSpheres) {
        const double dist = (position - sphere->center).norm() - sphere->radius;
        if (dist < h_p) {
            h_p = dist;
            nearestSphere = *sphere;
        }
    }
    return h_p;
}

Eigen::Vector3d computeCBFGradient(const Eigen::Vector3d& position,
                                   const ocs2::legged_robot::EdgeSphere& sphere) {
    return (position - sphere.center).normalized();
}

Eigen::Matrix3d computeCBFHessian(const Eigen::Vector3d& position,
                                  const ocs2::legged_robot::EdgeSphere& sphere) {
    const double dist = (position - sphere.center).norm();
    if (dist < 1e-6) {
        return Eigen::Matrix3d::Zero();
    }

    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d unit = (position - sphere.center) / dist;
    return (I - unit * unit.transpose()) / dist;
}

void smoothTrajectoryWithSafetyFloor(OptimizedTrajectory& traj,
                                     const std::vector<double>& lowerBounds) {
    if (traj.positions.size() < 3 || traj.positions.size() != lowerBounds.size() || traj.times.size() != traj.positions.size()) {
        return;
    }

    std::vector<double> smoothedZ;
    smoothedZ.reserve(traj.positions.size());
    for (const auto& position : traj.positions) {
        smoothedZ.push_back(position.z());
    }
    const std::vector<double> originalZ = smoothedZ;

    for (int iter = 0; iter < config_.postSmoothIterations; ++iter) {
        for (size_t i = 1; i + 1 < smoothedZ.size(); ++i) {
            const double neighborMean = 0.5 * (smoothedZ[i - 1] + smoothedZ[i + 1]);
            const double weightedZ = (config_.postSmoothTrackWeight * originalZ[i] +
                                      config_.postSmoothNeighborWeight * neighborMean) /
                                     (config_.postSmoothTrackWeight + config_.postSmoothNeighborWeight);
            smoothedZ[i] = std::max(lowerBounds[i], weightedZ);
        }
        smoothedZ.front() = originalZ.front();
        smoothedZ.back() = originalZ.back();
    }

    for (size_t i = 0; i < traj.positions.size(); ++i) {
        traj.positions[i].z() = smoothedZ[i];
    }

    for (size_t i = 0; i < traj.velocities.size(); ++i) {
        if (i == 0) {
            const double dtForward = std::max(traj.times[i + 1] - traj.times[i], 1e-6);
            traj.velocities[i] = (traj.positions[i + 1] - traj.positions[i]) / dtForward;
        } else if (i + 1 == traj.velocities.size()) {
            const double dtBackward = std::max(traj.times[i] - traj.times[i - 1], 1e-6);
            traj.velocities[i] = (traj.positions[i] - traj.positions[i - 1]) / dtBackward;
        } else {
            const double dtSpan = std::max(traj.times[i + 1] - traj.times[i - 1], 1e-6);
            traj.velocities[i] = (traj.positions[i + 1] - traj.positions[i - 1]) / dtSpan;
        }
    }

    if (traj.accelerations.size() != traj.positions.size()) {
        traj.accelerations.resize(traj.positions.size(), Eigen::Vector3d::Zero());
    }
    for (size_t i = 0; i < traj.accelerations.size(); ++i) {
        if (i == 0) {
            const double dtForward = std::max(traj.times[i + 1] - traj.times[i], 1e-6);
            traj.accelerations[i] = (traj.velocities[i + 1] - traj.velocities[i]) / dtForward;
        } else if (i + 1 == traj.accelerations.size()) {
            const double dtBackward = std::max(traj.times[i] - traj.times[i - 1], 1e-6);
            traj.accelerations[i] = (traj.velocities[i] - traj.velocities[i - 1]) / dtBackward;
        } else {
            const double dtSpan = std::max(traj.times[i + 1] - traj.times[i - 1], 1e-6);
            traj.accelerations[i] = (traj.velocities[i + 1] - traj.velocities[i - 1]) / dtSpan;
        }
    }
}

void computeCLF(const FootState& state, const Eigen::Vector3d& refPos,
                  const Eigen::Vector3d& refVel,
                  const Eigen::Vector3d& refAccel,
                  Eigen::Vector3d& c_clf, double& d_clf) {
    const Eigen::Vector3d pErr = state.position - refPos;
    const Eigen::Vector3d vErr = state.velocity - refVel;
    double V = config_.Kp * pErr.squaredNorm() + config_.Kv * vErr.squaredNorm();

    c_clf = 2.0 * config_.Kv * vErr;
    d_clf = -config_.gamma * V - 2.0 * config_.Kp * pErr.dot(vErr) + 2.0 * config_.Kv * vErr.dot(refAccel);
    return ;
}

// ============================================================================
// QP 求解 (地形 HOCBF + CLF + 球障碍 HOCBF)
// ============================================================================
QpResult solveQp(const FootState& state,
                 double dt,
                 const Eigen::Vector3d& prevAccel,
                 const Eigen::Vector3d& refAccel,
                 const Eigen::Vector3d& refPos,
                 const Eigen::Vector3d& refVel,
                 double minSafeHeight,
                 const std::vector<const ocs2::legged_robot::EdgeSphere*>& edgeSpheres) {
    QpResult result;

    (void)dt;

    const bool hasSphereCbfConstraint = !edgeSpheres.empty();
    Eigen::Vector3d cSphere = Eigen::Vector3d::Zero();
    double dSphere = 0.0;
    if (hasSphereCbfConstraint) {
        ocs2::legged_robot::EdgeSphere nearestSphere;
        const double hSphere = computeCBF(state.position, edgeSpheres, nearestSphere);
        const Eigen::Vector3d gradSphere = computeCBFGradient(state.position, nearestSphere);
        const Eigen::Matrix3d hessSphere = computeCBFHessian(state.position, nearestSphere);
        const double hDotSphere = gradSphere.dot(state.velocity);
        const double driftSphere = state.velocity.dot(hessSphere * state.velocity);

        cSphere = gradSphere;
        dSphere = -config_.alpha1 * hDotSphere - config_.alpha0 * hSphere - driftSphere;
    }

    // CLF 相关计算
    // 跟踪误差
    const Eigen::Vector3d pErr = state.position - refPos;
    const Eigen::Vector3d vErr = state.velocity - refVel;
    Eigen::Vector3d c_clf;
    double d_clf;
    computeCLF(state, refPos, refVel, refAccel, c_clf, d_clf);

    const double hFloor = state.position.z() - minSafeHeight;
    const double hDotFloor = state.velocity.z();
    const double dFloor = -config_.alpha1 * hDotFloor - config_.alpha0 * hFloor;

    //  // 变量: [ux, uy, uz, delta]
    const int nVar = 4;
    const int nCon = 1 + 1 + (hasSphereCbfConstraint ? 1 : 0) + 6 + 1; // CLF + 上方偏置高度约束 + 可选sphere CBF + 加速度上界 + delta >= 0
    Eigen::Matrix<double, nVar, nVar, Eigen::RowMajor> H = Eigen::Matrix<double, nVar, nVar, Eigen::RowMajor>::Zero();
    const double accelDiag = 2.0 * (config_.accelTrackWeight + config_.accelSmoothWeight);
    H.topLeftCorner(3, 3) = accelDiag * Eigen::Matrix3d::Identity();
    H(3, 3) = config_.rho;

    Eigen::Vector4d g = Eigen::Vector4d::Zero();
    g.head<3>() = -2.0 * (config_.accelTrackWeight * refAccel + config_.accelSmoothWeight * prevAccel);
    g(3) = 0.0;


    // 约束矩阵 A_lb <= A*x <= A_ub
    // 依次：CLF: c_clf'*u - delta <= d_clf  =>  [c_clf, -1] * [u;delta] <= d_clf
    //       sphere CBF: cSphere'*u >= dSphere => -[cSphere, 0] * [u;delta] <= -dSphere
    //       acc min: u >= amin -> -I * u <= -amin
    //       acc max: u <= amax -> I * u <= amax
    //       delta >= 0 -> -delta <= 0

    int nConstraints_total = nCon;
    Eigen::Matrix<double, Eigen::Dynamic, 4, Eigen::RowMajor> A(nConstraints_total, 4);
    Eigen::VectorXd lbA(nConstraints_total), ubA(nConstraints_total);
    int row = 0;

    // CLF约束
    A.row(row) << c_clf.transpose(), -1.0;
    lbA(row) = -1e10; ubA(row) = d_clf; row++;

    // 最低安全高度约束: z >= minSafeHeight
    A.row(row).setZero();
    A(row, 2) = -1.0;
    lbA(row) = -1e10;
    ubA(row) = -dFloor;
    row++;

    if (hasSphereCbfConstraint) {
        A.row(row) << -cSphere.transpose(), 0.0;
        lbA(row) = -1e10;
        ubA(row) = -dSphere;
        row++;
    }

    // 加速度上下界约束
    for(int i = 0; i < 3; ++i) {
        A.row(row).setZero();
        A(row, i) = -1.0; // u_i >= a_min -> -u_i <= -a_min
        A(row, 3) = 0.0; // delta 不参与加速度约束
        lbA(row) = -1e10; ubA(row) = -config_.a_min; row++;
    }

    for(int i = 0; i < 3; ++i) {
        A.row(row).setZero();
        A(row, i) = 1.0; // u_i <= a_max -> u_i <= a_max
        A(row, 3) = 0.0; // delta 不参与加速度约束
        lbA(row) = -1e10; ubA(row) = config_.a_max; row++;
    }

    // delta >= 0
    A.row(row).setZero();
    A(row, 3) = -1.0;
    lbA(row) = -1e10; ubA(row) = 0.0; row++;

    // 求解
    qpOASES::QProblem qpSolver(nVar, nCon);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    qpSolver.setOptions(options);

    qpOASES::int_t nWSR = 200;
    auto status = qpSolver.init(H.data(), g.data(), A.data(), nullptr, nullptr, lbA.data(), ubA.data(), nWSR);

    std::vector<qpOASES::real_t> xOpt(nVar);
    if (status == qpOASES::SUCCESSFUL_RETURN) {
        qpSolver.getPrimalSolution(xOpt.data());
        result.acceleration = Eigen::Vector3d(xOpt[0], xOpt[1], xOpt[2]);
        result.acceleration = result.acceleration.cwiseMax(Eigen::Vector3d::Constant(config_.a_min))
                                                 .cwiseMin(Eigen::Vector3d::Constant(config_.a_max));
        result.slack = xOpt[3];
        result.feasible = true;
    } else {
        result.acceleration = prevAccel;
        result.slack = 0.0;
        result.feasible = false;
    }
    return result;
}



// ============================================================================
// 参考轨迹生成 (QuinticSplineCpg)
// ============================================================================
ocs2::legged_robot::QuinticSplineCpg generateRefSpline(
    double liftOffZ, double touchDownZ, double midHeight,
    double swingStartTime, double swingFinalTime,
    double liftOffVelocity = 0.0, double touchDownVelocity = 0.0) {

    using Node = ocs2::legged_robot::QuinticSpline::Node;
    const double dur = swingFinalTime - swingStartTime;
    const double scaling = std::min(1.0, dur / 0.15);

    Node liftOff{swingStartTime, liftOffZ, scaling * liftOffVelocity, 0.0};
    Node touchDown{swingFinalTime, touchDownZ, scaling * touchDownVelocity, 0.0};
    return ocs2::legged_robot::QuinticSplineCpg(liftOff, midHeight, touchDown);
}

// ============================================================================
// CLF-CBF QP 优化完整摆动轨迹
// ============================================================================
OptimizedTrajectory optimizeSwingTrajectory(
    const ocs2::legged_robot::QuinticSplineCpg& refSpline,
    double swingStartTime, double swingFinalTime,
    const Eigen::Vector3d& liftOffPos3d,
    const Eigen::Vector3d& touchDownPos3d,
    const std::vector<const ocs2::legged_robot::EdgeSphere*>& nearbySpheres) {

    OptimizedTrajectory traj;
    const double duration = swingFinalTime - swingStartTime;
    if (duration <= 0) return traj;

    const int numSteps = std::max(config_.numSamples,
                                  static_cast<int>(std::ceil(duration / config_.dt)));
    const double dt = duration / numSteps;

    double refMaxZ = -1e10;
    for (int i = 0; i <= numSteps; ++i) {
        refMaxZ = std::max(refMaxZ, refSpline.position(swingStartTime + i * dt));
    }
    std::cout << "参考轨迹最大高度: " << refMaxZ << std::endl;
    // const double heightCeiling = refMaxZ + config_.heightCeilingMargin;

    // 初始状态
    FootState state;
    // 足端状态位置初始为 liftOff 的 XY 和参考轨迹的 Z，确保初始状态满足地形约束
    state.position = liftOffPos3d;
    state.position.z() = refSpline.position(swingStartTime);

    // 足端速度初始为参考轨迹的速度，确保初始状态满足 CLF 约束
    const Eigen::Vector3d xyDisplacement = touchDownPos3d - liftOffPos3d;
    const Eigen::Vector3d xyVelocity(xyDisplacement.x() / duration,
                                     xyDisplacement.y() / duration, 0.0);
    state.velocity = Eigen::Vector3d(xyVelocity.x(), xyVelocity.y(),
                                     refSpline.velocity(swingStartTime));

    traj.times.push_back(swingStartTime);
    traj.positions.push_back(state.position);
    traj.velocities.push_back(state.velocity);

    Eigen::Vector3d prevAccel = Eigen::Vector3d::Zero();
    int infeasibleCount = 0, clampCount = 0;

    // 筛选参考轨迹附近的地形边缘球障碍
    std::vector<const ocs2::legged_robot::EdgeSphere*> stepSpheres;
    for(int step = 0; step <= numSteps; ++step){
        const double t = swingStartTime + step*dt;
        const Eigen::Vector3d xyRef = interpolateXY(liftOffPos3d, touchDownPos3d, t, swingStartTime, swingFinalTime);
        const Eigen::Vector3d refPos(xyRef.x(), xyRef.y(), refSpline.position(t));
        for (const auto* s : nearbySpheres) {
            const bool isNearRef = (refPos - s->center).norm() < s->radius + config_.sphereActivationDist;
            const bool alreadyAdded = std::find(stepSpheres.begin(), stepSpheres.end(), s) != stepSpheres.end();
            if (isNearRef && !alreadyAdded){
                stepSpheres.push_back(s);
                std::cout << "激活球障碍: center=" << s->center.transpose() << " radius=" << s->radius 
                << "附近参考轨迹点：" << refPos.transpose() 
                << "参考轨迹点与球心距离：" << (refPos - s->center).norm() << std::endl;
            }
        }
    }

    for (int step = 0; step < numSteps; ++step) {
        const double t = swingStartTime + step * dt;
        const double tNext = t + dt;

        const Eigen::Vector3d xyRef = interpolateXY(liftOffPos3d, touchDownPos3d, t, swingStartTime, swingFinalTime);
        const Eigen::Vector3d xyRefNext = interpolateXY(liftOffPos3d, touchDownPos3d, tNext, swingStartTime, swingFinalTime);
        Eigen::Vector3d refPos(xyRef.x(), xyRef.y(), refSpline.position(t));    // 参考轨迹的位置
        Eigen::Vector3d refVel(xyVelocity.x(), xyVelocity.y(), refSpline.velocity(t));  // 参考轨迹的速度
        Eigen::Vector3d refAccel(0.0, 0.0, refSpline.acceleration(t));  // 参考轨迹的加速度

        // XY 在下一步会被强制跟踪到参考，因此地形约束按下一时刻落点来预瞄。
        const double zTerrainNext = terrain->getHeight(xyRefNext.x(), xyRefNext.y());
        const double minSafeHeightNext = computeSmoothBiasHeightAtXY(xyRefNext.head<2>(), zTerrainNext, stepSpheres);


        QpResult qpResult = solveQp(state, dt, prevAccel, refAccel, refPos, refVel, minSafeHeightNext, stepSpheres);
        if (!qpResult.feasible) ++infeasibleCount;


        std::cout << "t = " << t << " | x_pos = " << state.position.x() << ": optimized z_pos = " << state.position.z() << " | ref z_pos = " << refPos.z()
                  << " | optimized z_vel = " << state.velocity.z() << " | ref z_vel = " << refVel.z()
                  << " | optimized z_acc = " << qpResult.acceleration.z() << " | ref z_acc = " << refAccel.z()  
                 << std::endl;

        // 前向欧拉积分
        state.position += state.velocity * dt + 0.5 * qpResult.acceleration * dt * dt;
        state.velocity += qpResult.acceleration * dt;

        // XY 强制跟踪
        state.position.x() = xyRefNext.x();
        state.position.y() = xyRefNext.y();
        state.velocity.x() = xyVelocity.x();
        state.velocity.y() = xyVelocity.y();

        const double zExactSafe = computeExactSafeHeightAtXY(state.position.head<2>(), terrain->getHeight(state.position.x(), state.position.y()), stepSpheres);
        if (state.position.z() < zExactSafe) {
            state.position.z() = zExactSafe;
            state.velocity.z() = std::max(0.0, state.velocity.z());
            ++clampCount;
        }


        prevAccel = qpResult.acceleration;
        traj.times.push_back(tNext);
        traj.positions.push_back(state.position);
        traj.velocities.push_back(state.velocity);
        traj.accelerations.push_back(qpResult.acceleration);
    }

    if (!traj.accelerations.empty())
        traj.accelerations.push_back(traj.accelerations.back());
    else
        traj.accelerations.push_back(Eigen::Vector3d::Zero());

    traj.positions.back() = touchDownPos3d;

    std::vector<double> lowerBounds;
    lowerBounds.reserve(traj.positions.size());
    for (const auto& p : traj.positions) {
        lowerBounds.push_back(computeExactSafeHeightAtXY(p.head<2>(), terrain->getHeight(p.x(), p.y()), nearbySpheres));
    }
    smoothTrajectoryWithSafetyFloor(traj, lowerBounds);

    traj.velocities.front().x() = xyVelocity.x();
    traj.velocities.front().y() = xyVelocity.y();
    traj.velocities.back().x() = xyVelocity.x();
    traj.velocities.back().y() = xyVelocity.y();
    traj.velocities.back().z() = 0.0;
    traj.accelerations.back().z() = 0.0;



    traj.collisionFree = true;
    for (const auto& p : traj.positions) {
        const double minSafeHeight = computeExactSafeHeightAtXY(p.head<2>(), terrain->getHeight(p.x(), p.y()), nearbySpheres);
        if (p.z() < minSafeHeight - 1e-6) {
            traj.collisionFree = false;
            break;
        }

        for (const auto* sphere : nearbySpheres) {
            if ((p - sphere->center).norm() < sphere->radius - 1e-6) {
                traj.collisionFree = false;
                break;
            }
        }
        if (!traj.collisionFree) {
            break;
        }
    }

    if (config_.verbose) {
        double maxZ = -1e10, minZ = 1e10;
        for (const auto& p : traj.positions) { maxZ = std::max(maxZ, p.z()); minZ = std::min(minZ, p.z()); }
        std::cerr << "[CLF-CBF] z=[" << minZ << ", " << maxZ
                  << "] " 
                  << " spheres=" << nearbySpheres.size()
                  << " infeas=" << infeasibleCount << "/" << numSteps
                  << " clamp=" << clampCount
                  << " validated=" << (traj.collisionFree ? "YES" : "NO") << "\n";
    }

    OptimizedTrajectory reconstructedTrajectory = reconstructCollisionFreeQuinticTrajectory(
        traj, refSpline, swingStartTime, swingFinalTime, liftOffPos3d, touchDownPos3d, nearbySpheres);
    if (!reconstructedTrajectory.collisionFree && traj.collisionFree) {
        return traj;
    }
    return reconstructedTrajectory;
}

// ============================================================================
// 写入 CSV 并调用 Python 绘图
// ============================================================================
void writeCSVAndPlot(const ocs2::legged_robot::QuinticSplineCpg& refSpline,
                     const OptimizedTrajectory& optTraj,
                     const Eigen::Vector3d& liftOff, const Eigen::Vector3d& touchDown,
                     double tStart, double tEnd, int nSamples) {

    const std::string csvPath = "/tmp/clf_cbf_test_result.csv";
    const double duration = tEnd - tStart;

    std::ofstream csv(csvPath);
    csv << "type,x,z\n";

    // 地形
    for (int i = 0; i <= nSamples; ++i) {
        double x = liftOff.x() + (terrain->edgeX() - liftOff.x()) * i / nSamples;
        if (x < terrain->edgeX()) {
            csv << "terrain," << x << ",0\n";
        }
    }
    csv << "terrain," << terrain->edgeX() << ",0\n";
    csv << "terrain," << terrain->edgeX() << "," << terrain->stepHeight() << "\n";
    for (int i = 0; i <= nSamples; ++i) {
        double x = terrain->edgeX() + (touchDown.x() - terrain->edgeX()) * i / nSamples;
        if (x > terrain->edgeX()) {
            csv << "terrain," << x << "," << terrain->stepHeight() << "\n";
        }
    }

    // 参考轨迹
    for (int i = 0; i <= nSamples; ++i) {
        double t = tStart + duration * i / nSamples;
        double alpha = (duration > 0) ? (t - tStart) / duration : 0.0;
        double x = liftOff.x() + alpha * (touchDown.x() - liftOff.x());
        csv << "reference," << x << "," << refSpline.position(t) << "\n";
    }

    // 优化轨迹
    for (size_t i = 0; i < optTraj.positions.size(); ++i) {
        csv << "optimized," << optTraj.positions[i].x() << "," << optTraj.positions[i].z() << "\n";
    }

    // EdgeSphere (圆形轮廓)
    const int circPts = 64;
    for (int i = 0; i <= circPts; ++i) {
        double theta = 2.0 * M_PI * i / circPts;
        double sx = stepEdgeSphere.center.x() + stepEdgeSphere.radius * std::cos(theta);
        double sz = stepEdgeSphere.center.z() + stepEdgeSphere.radius * std::sin(theta);
        csv << "sphere," << sx << "," << sz << "\n";
    }

    csv.close();
    std::cout << "[CSV saved: " << csvPath << "]\n";

    // Python 绘图脚本
    const std::string pyScript = "/tmp/clf_cbf_plot.py";
    {
        std::ofstream py(pyScript);
        py << "#!/usr/bin/env python3\n"
           << "\"\"\"CLF-CBF QP swing trajectory optimization visualization\"\"\"\n"
           << "import pandas as pd\n"
           << "import matplotlib.pyplot as plt\n"
           << "import matplotlib\n"
           << "from matplotlib.ticker import MultipleLocator\n"
           << "matplotlib.rcParams['font.sans-serif'] = ['WenQuanYi Micro Hei', 'SimHei', 'DejaVu Sans']\n"
           << "matplotlib.rcParams['axes.unicode_minus'] = False\n"
           << "\n"
           << "df = pd.read_csv('/tmp/clf_cbf_test_result.csv')\n"
              << "fig, (ax_overview, ax_zoom) = plt.subplots(1, 2, figsize=(15.5, 5.4), gridspec_kw={'width_ratios': [1.9, 1.1]})\n"
           << "\n"
           << "t = df[df['type'] == 'terrain']\n"
           << "r = df[df['type'] == 'reference']\n"
           << "o = df[df['type'] == 'optimized']\n"
           << "s = df[df['type'] == 'sphere']\n"
              << "\n"
              << "def draw_scene(ax, equal_aspect=False, show_legend=False):\n"
              << "    ax.fill_between(t['x'], -0.02, t['z'], alpha=0.3, color='brown', label='terrain')\n"
              << "    ax.plot(t['x'], t['z'], 'k-', linewidth=2)\n"
              << "    ax.plot(r['x'], r['z'], 'b--', linewidth=1.5, label='ref trajectory (collision)')\n"
              << "    if len(o) > 0:\n"
              << "        ax.plot(o['x'], o['z'], 'r-', linewidth=2.0, label='CLF-CBF optimized')\n"
              << "    if len(s) > 0:\n"
              << "        ax.plot(s['x'], s['z'], 'm-', linewidth=1.5, label='EdgeSphere')\n"
              << "        ax.fill(s['x'], s['z'], alpha=0.2, color='magenta')\n"
              << "    ax.plot(0.0, 0.0, 'go', markersize=10, zorder=5, label='LiftOff')\n"
              << "    ax.plot(0.10, 0.10, 'rs', markersize=10, zorder=5, label='TouchDown')\n"
              << "    ax.axvline(x=0.05, color='gray', linestyle=':', alpha=0.5, label='step edge')\n"
              << "    ax.set_xlabel('X (m)', fontsize=12)\n"
              << "    ax.set_ylabel('Z (m)', fontsize=12)\n"
              << "    ax.grid(True, alpha=0.3)\n"
              << "    if equal_aspect:\n"
              << "        ax.set_aspect('equal', adjustable='box')\n"
              << "    if show_legend:\n"
              << "        ax.legend(loc='upper right', fontsize=10)\n"
              << "\n"
              << "draw_scene(ax_overview, equal_aspect=True, show_legend=True)\n"
              << "ax_overview.set_title('Overview', fontsize=13)\n"
              << "ax_overview.set_xlim(-0.00, 0.10)\n"
              << "ax_overview.set_ylim(-0.0, 0.16)\n"
              << "ax_overview.xaxis.set_major_locator(MultipleLocator(0.01))\n"
              << "ax_overview.yaxis.set_major_locator(MultipleLocator(0.02))\n"
              << "\n"
              << "draw_scene(ax_zoom, equal_aspect=True, show_legend=False)\n"
              << "ax_zoom.set_title('Equal-Scale Collision View', fontsize=13)\n"
              << "ax_zoom.set_xlim(0.00, 0.10)\n"
              << "ax_zoom.set_ylim(0.00, 0.16)\n"
              << "ax_zoom.xaxis.set_major_locator(MultipleLocator(0.01))\n"
              << "ax_zoom.yaxis.set_major_locator(MultipleLocator(0.02))\n"
              << "\n"
              << "fig.suptitle('CLF-CBF QP Swing Trajectory Optimization', fontsize=16)\n"
           << "\n"
           << "plt.tight_layout()\n"
           << "plt.savefig('/tmp/clf_cbf_test_result.png', dpi=150)\n"
           << "print('[Saved /tmp/clf_cbf_test_result.png]')\n"
           << "plt.show()\n";
    }

    std::cout << "[Plot script: " << pyScript << "]\n";
    int ret = std::system("python3 /tmp/clf_cbf_plot.py 2>/dev/null");
    if (ret != 0) {
        std::cout << "[Auto-plot failed, run manually: python3 " << pyScript << "]\n";
    }
}

// ============================================================================
// main
// ============================================================================
int main(int /*argc*/, char* /*argv*/[]) {
    std::cout << "========================================\n"
              << "  CLF-CBF QP Swing Trajectory Test\n"
              << "========================================\n\n";

    const double edgeX = 0.05;
    const double stepHeight = 0.10;

    const double swingStartTime = 0.0;
    const double swingFinalTime = 0.4;

    const Eigen::Vector3d liftOffPos(0.025, 0.0, 0.0);
    const Eigen::Vector3d touchDownPos(0.10, 0.0, stepHeight);

    // 创建地形
    StepTerrain terrainObj(edgeX, stepHeight);
    terrain = &terrainObj;

    std::cout << "[Terrain check]\n"
              << "  z(0.000) = " << terrain->getHeight(0.0, 0.0)   << " (expect ~0)\n"
              << "  z(0.050) = " << terrain->getHeight(0.05, 0.0)  << " (expect 0.10)\n"
              << "  z(0.100) = " << terrain->getHeight(0.10, 0.0)  << " (expect 0.10)\n\n";

    // 台阶边缘 EdgeSphere
    stepEdgeSphere.center = Eigen::Vector3d(edgeX, 0.0, stepHeight);
    stepEdgeSphere.radius = 0.02;

    std::vector<const ocs2::legged_robot::EdgeSphere*> nearbySpheres = {&stepEdgeSphere};

    // 生成参考轨迹 (故意设低 midHeight 使其穿越台阶)
    const double midHeight = 0.12;
    auto refSpline = generateRefSpline(liftOffPos.z(), touchDownPos.z(), midHeight,
                                       swingStartTime, swingFinalTime);

    // 分析参考轨迹碰撞
    const int nSamples = 200;
    const double dur = swingFinalTime - swingStartTime;
    int refCollisions = 0;
    double refMinClearance = 1e10;
    for (int i = 0; i <= nSamples; ++i) {
        double t = swingStartTime + dur * i / nSamples;
        double alpha = dur > 0 ? (t - swingStartTime) / dur : 0.0;
        double x = liftOffPos.x() + alpha * (touchDownPos.x() - liftOffPos.x());
        double zRef = refSpline.position(t);
        double zTerrain = terrain->getHeight(x, 0.0);
        double clearance = zRef - zTerrain;
        refMinClearance = std::min(refMinClearance, clearance);
        if (clearance < 0) ++refCollisions;
    }

    std::cout << "[Reference trajectory]\n"
              << "  midHeight = " << midHeight << " m\n"
              << "  collision samples: " << refCollisions << " / " << (nSamples + 1) << "\n"
              << "  min clearance: " << refMinClearance << " m\n";
    if (refCollisions > 0)
        std::cout << "  >>> Reference has collisions! CLF-CBF optimization needed <<<\n\n";
    else
        std::cout << "  >>> No collisions (lower midHeight to create collision)\n\n";

    // CLF-CBF 优化
    std::cout << "[Running CLF-CBF QP optimization...]\n";
    auto optTraj = optimizeSwingTrajectory(refSpline, swingStartTime, swingFinalTime,
                                           liftOffPos, touchDownPos, nearbySpheres);

    if (optTraj.times.empty()) {
        std::cout << "  Optimizer returned empty trajectory\n";
    } else {
        int optCollisions = 0;
        double optMinClearance = 1e10, optMaxZ = -1e10;
        for (const auto& p : optTraj.positions) {
            double zT = terrain->getHeight(p.x(), p.y());
            double cl = p.z() - zT;
            optMinClearance = std::min(optMinClearance, cl);
            optMaxZ = std::max(optMaxZ, p.z());
            if (cl < 0) ++optCollisions;
        }
        std::cout << "\n[Optimized trajectory]\n"
                  << "  points: " << optTraj.positions.size() << "\n"
                  << "  collisions: " << optCollisions << "\n"
                  << "  min clearance: " << optMinClearance << " m\n"
                  << "  max height: " << optMaxZ << " m\n"
                  << "  validated: " << (optTraj.collisionFree ? "YES" : "NO") << "\n\n";
    }

    // 可视化
    writeCSVAndPlot(refSpline, optTraj, liftOffPos, touchDownPos,
                    swingStartTime, swingFinalTime, nSamples);

    std::cout << "\n========================================\n"
              << "  Test complete\n"
              << "========================================\n";
    return 0;
}
