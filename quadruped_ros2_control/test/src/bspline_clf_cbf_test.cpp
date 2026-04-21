/******************************************************************************
 * 参考轨迹控制点采样 + B 样条重建测试
 *
 * 流程:
 *   1. 生成一条 QuinticSplineCpg 参考摆动高度轨迹
 *   2. 沿参考轨迹均匀采样一组三维控制点 (x, y, z)
 *   3. 使用三次 B 样条生成离散曲线
 *   4. 导出 CSV 并调用 Python 绘图
 ******************************************************************************/

#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

#include <qpOASES.hpp>

#include "BSpline.h"
#include "ocs2_quadruped_controller/interface/constraint/QuinticSplineCpg.h"
#include "ocs2_quadruped_controller/perceptive/synchronize/EdgeSphere.h"


struct BSplineConfig {
    // B 样条参数
    int numControlPoints = 15;
    int numCurveSamples = 300;
    int splineOrder = 3;

    // 摆动参数
    double swingStartTime = 0.0;
    double swingFinalTime = 0.4;
    double midHeight = 0.08;

    // 地形参数
    double edgeX = 0.0;
    double stepHeight = 0.00;

    double sphereRadius = 0.0;
    double groundClearance = 1e-3;
    double sphereClearance = 0.01;
    double controlPointSmoothnessWeight = 5e-2;
    double controlPointZMin = -0.02;
    double controlPointZMax = 0.22;

    // 起飞和着陆位置
    Eigen::Vector3d liftOffPos = Eigen::Vector3d(0.0, 0.0, 0.0);
    Eigen::Vector3d touchDownPos = Eigen::Vector3d(0.10, 0.0, 0.0);
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

struct CurvePoint {
    double time = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
};

struct CollisionStats {
    int terrainCollisionCount = 0;
    int sphereCollisionCount = 0;
    double minTerrainClearance = std::numeric_limits<double>::infinity();
    double minSphereSignedDistance = std::numeric_limits<double>::infinity();
};

static StepTerrain* terrain = nullptr;
static ocs2::legged_robot::EdgeSphere stepEdgeSphere;

Eigen::Vector3d interpolateFootPosition(const Eigen::Vector3d& liftOff,
                                        const Eigen::Vector3d& touchDown,
                                        double t,
                                        double t0,
                                        double tf) {
    const double alpha = (tf > t0) ? (t - t0) / (tf - t0) : 0.0;
    return liftOff + alpha * (touchDown - liftOff);
}

double normalizedToTime(double u, const BSplineConfig& cfg) {
    return cfg.swingStartTime + u * (cfg.swingFinalTime - cfg.swingStartTime);
}

void sampleReferenceTrajectory(const ocs2::legged_robot::QuinticSplineCpg& refSpline,
                               const BSplineConfig& cfg,
                               int numSamples,
                               std::vector<CurvePoint>& samples) {
    samples.clear();
    samples.reserve(numSamples);

    const Eigen::VectorXd params = Eigen::VectorXd::LinSpaced(numSamples, 0.0, 1.0);
    for (int i = 0; i < numSamples; ++i) {
        const double t = normalizedToTime(params[i], cfg);
        const Eigen::Vector3d footPos = interpolateFootPosition(
            cfg.liftOffPos, cfg.touchDownPos, t, cfg.swingStartTime, cfg.swingFinalTime);

        CurvePoint point;
        point.time = t;
        point.position = Eigen::Vector3d(footPos.x(), footPos.y(), refSpline.position(t));
        samples.push_back(point);
    }
}

void sampleReferenceTrajectoryToControlPoints(const ocs2::legged_robot::QuinticSplineCpg& refSpline,
                                              const BSplineConfig& cfg,
                                              std::vector<Eigen::Vector3d>& controlPoints) {
    controlPoints.clear();
    controlPoints.reserve(cfg.numControlPoints);

    const Eigen::VectorXd params = Eigen::VectorXd::LinSpaced(cfg.numControlPoints, 0.0, 1.0);
    for (int i = 0; i < cfg.numControlPoints; ++i) {
        const double t = normalizedToTime(params[i], cfg);
        const Eigen::Vector3d footPos = interpolateFootPosition(
            cfg.liftOffPos, cfg.touchDownPos, t, cfg.swingStartTime, cfg.swingFinalTime);
        controlPoints.emplace_back(footPos.x(), footPos.y(), refSpline.position(t));
    }
}

void evaluateBSplineCurve(const BSpline& bspline,
                          const BSplineConfig& cfg,
                          std::vector<CurvePoint>& curveSamples) {
    curveSamples.clear();
    curveSamples.reserve(cfg.numCurveSamples);

    const Eigen::VectorXd params = Eigen::VectorXd::LinSpaced(cfg.numCurveSamples, 0.0, 1.0);
    const Eigen::VectorXd splineX = bspline.getSplineX(params);
    const Eigen::VectorXd splineY = bspline.getSplineY(params);
    const Eigen::VectorXd splineZ = bspline.getSplineZ(params);

    for (int i = 0; i < params.size(); ++i) {
        CurvePoint point;
        point.time = normalizedToTime(params[i], cfg);
        point.position = Eigen::Vector3d(splineX[i], splineY[i], splineZ[i]);
        curveSamples.push_back(point);
    }
}

CollisionStats analyzeCurveCollisions(const std::vector<CurvePoint>& curve) {
    CollisionStats stats;

    for (const auto& point : curve) {
        const double terrainHeight = terrain->getHeight(point.position.x(), point.position.y());
        const double terrainClearance = point.position.z() - terrainHeight;
        stats.minTerrainClearance = std::min(stats.minTerrainClearance, terrainClearance);
        if (terrainClearance < -1e-6) {
            ++stats.terrainCollisionCount;
        }

        if (stepEdgeSphere.radius > 0.0) {
            const double sphereSignedDistance = (point.position - stepEdgeSphere.center).norm() - stepEdgeSphere.radius;
            stats.minSphereSignedDistance = std::min(stats.minSphereSignedDistance, sphereSignedDistance);
            if (sphereSignedDistance < -1e-6) {
                ++stats.sphereCollisionCount;
            }
        }
    }

    return stats;
}

Eigen::VectorXd computeTerrainHeightProfile(const Eigen::VectorXd& sampleX) {
    Eigen::VectorXd terrainHeights(sampleX.size());
    for (int i = 0; i < sampleX.size(); ++i) {
        terrainHeights[i] = terrain->getHeight(sampleX[i], 0.0);
    }
    return terrainHeights;
}

Eigen::VectorXd computeTerrainHeightProfile(const Eigen::VectorXd& sampleX,
                                           const Eigen::VectorXd& sampleY) {
    Eigen::VectorXd terrainHeights(sampleX.size());
    for (int i = 0; i < sampleX.size(); ++i) {
        terrainHeights[i] = terrain->getHeight(sampleX[i], sampleY[i]);
    }
    return terrainHeights;
}

Eigen::VectorXd computeSphereHeightProfile(const Eigen::VectorXd& sampleX,
                                           const Eigen::VectorXd& sampleY,
                                           const BSplineConfig& cfg) {
    Eigen::VectorXd sphereHeights = Eigen::VectorXd::Constant(
        sampleX.size(), -std::numeric_limits<double>::infinity());

    if (stepEdgeSphere.radius <= 0.0) {
        return sphereHeights;
    }

    const double safeRadius = stepEdgeSphere.radius + cfg.sphereClearance;

    for (int i = 0; i < sampleX.size(); ++i) {
        const double dx = sampleX[i] - stepEdgeSphere.center.x();
        const double dy = sampleY[i] - stepEdgeSphere.center.y();
        const double radialTerm = safeRadius * safeRadius - dx * dx - dy * dy;
        if (radialTerm > 0.0) {
            sphereHeights[i] = stepEdgeSphere.center.z() + std::sqrt(radialTerm);
        }
    }

    return sphereHeights;
}

bool optimizeControlPointsZ(
    const Eigen::MatrixXd& basisMat,
    const Eigen::VectorXd& refZ,
    const Eigen::VectorXd& sampleX,
    const Eigen::VectorXd& terrainHeights,
    const Eigen::VectorXd& sphereHeights,
    double groundClearance,
    double smoothnessWeight,
    double zMin,
    double zMax,
    Eigen::VectorXd& xOpt)
{
    const int nV = basisMat.cols();
    const int numSamples = basisMat.rows();
    if (refZ.size() != nV || sampleX.size() != numSamples || terrainHeights.size() != numSamples ||
        sphereHeights.size() != numSamples) {
        std::cerr << "Invalid dimensions for optimizeControlPointsZ." << std::endl;
        return false;
    }

    Eigen::MatrixXd regularization = Eigen::MatrixXd::Zero(nV, nV);
    if (nV >= 3) {
        Eigen::MatrixXd secondDiff = Eigen::MatrixXd::Zero(nV - 2, nV);
        for (int i = 0; i < nV - 2; ++i) {
            secondDiff(i, i) = 1.0;
            secondDiff(i, i + 1) = -2.0;
            secondDiff(i, i + 2) = 1.0;
        }
        regularization = secondDiff.transpose() * secondDiff;
    }

    const Eigen::MatrixXd H_eigen = 2.0 * (Eigen::MatrixXd::Identity(nV, nV) + smoothnessWeight * regularization);
    const Eigen::VectorXd g_eigen = -2.0 * refZ;

    Eigen::VectorXd lb_eigen = Eigen::VectorXd::Constant(nV, zMin);
    Eigen::VectorXd ub_eigen = Eigen::VectorXd::Constant(nV, zMax);
    lb_eigen[0] = refZ[0];
    ub_eigen[0] = refZ[0];
    lb_eigen[nV - 1] = refZ[nV - 1];
    ub_eigen[nV - 1] = refZ[nV - 1];

    struct CandidateConstraint {
        Eigen::RowVectorXd row;
        double lowerBound = 0.0;
        int sampleIndex = -1;
        int type = 0;
    };

    std::vector<CandidateConstraint> candidates;
    candidates.reserve(2 * numSamples);
    for (int j = 1; j < numSamples - 1; ++j) {
        candidates.push_back(CandidateConstraint{basisMat.row(j), terrainHeights[j] + groundClearance, j, 0});
    }
    for (int j = 1; j < numSamples - 1; ++j) {
        if (std::isfinite(sphereHeights[j])) {
            candidates.push_back(CandidateConstraint{basisMat.row(j), sphereHeights[j], j, 1});
        }
    }

    auto upperAchievableFor = [&](const CandidateConstraint& candidate) {
        double upperAchievable = 0.0;
        for (int j = 0; j < nV; ++j) {
            const double a = candidate.row[j];
            upperAchievable += a * ((a >= 0.0) ? ub_eigen[j] : lb_eigen[j]);
        }
        return upperAchievable;
    };

    double worstViolation = 0.0;
    int worstConstraint = -1;
    double worstLowerBound = 0.0;
    double worstUpperAchievable = 0.0;
    for (size_t i = 0; i < candidates.size(); ++i) {
        const double upperAchievable = upperAchievableFor(candidates[i]);
        const double violation = candidates[i].lowerBound - upperAchievable;
        if (violation > worstViolation) {
            worstViolation = violation;
            worstConstraint = static_cast<int>(i);
            worstLowerBound = candidates[i].lowerBound;
            worstUpperAchievable = upperAchievable;
        }
    }

    if (worstViolation > 1e-8) {
        std::cerr << "QP infeasible before solve. Worst constraint=" << worstConstraint
                  << " sampleX=" << sampleX[candidates[worstConstraint].sampleIndex]
                  << " type=" << (candidates[worstConstraint].type == 0 ? "terrain" : "sphere")
                  << " lowerBound=" << worstLowerBound
                  << " upperAchievable=" << worstUpperAchievable
                  << " violation=" << worstViolation << std::endl;
        return false;
    }

    std::vector<qpOASES::real_t> H(H_eigen.size());
    std::vector<qpOASES::real_t> g(g_eigen.size());
    std::vector<qpOASES::real_t> lb(lb_eigen.size());
    std::vector<qpOASES::real_t> ub(ub_eigen.size());

    int idx = 0;
    for (int i = 0; i < nV; ++i) {
        for (int j = 0; j < nV; ++j) {
            H[idx++] = static_cast<qpOASES::real_t>(H_eigen(i, j));
        }
        g[i] = static_cast<qpOASES::real_t>(g_eigen[i]);
        lb[i] = static_cast<qpOASES::real_t>(lb_eigen[i]);
        ub[i] = static_cast<qpOASES::real_t>(ub_eigen[i]);
    }

    auto projectReferenceToBounds = [&]() {
        Eigen::VectorXd projected = refZ;
        for (int i = 0; i < nV; ++i) {
            projected[i] = std::min(std::max(projected[i], lb_eigen[i]), ub_eigen[i]);
        }
        return projected;
    };

    auto solveSubproblem = [&](const std::vector<int>& activeSet, Eigen::VectorXd& solution) {
        if (activeSet.empty()) {
            solution = projectReferenceToBounds();
            return true;
        }

        const int nC = static_cast<int>(activeSet.size());
        Eigen::MatrixXd A_eigen(nC, nV);
        Eigen::VectorXd lbA_eigen(nC);
        Eigen::VectorXd ubA_eigen = Eigen::VectorXd::Constant(nC, 1e20);
        for (int i = 0; i < nC; ++i) {
            A_eigen.row(i) = candidates[activeSet[i]].row;
            lbA_eigen[i] = candidates[activeSet[i]].lowerBound;
        }

        std::vector<qpOASES::real_t> A(A_eigen.size());
        std::vector<qpOASES::real_t> lbA(lbA_eigen.size());
        std::vector<qpOASES::real_t> ubA(ubA_eigen.size());
        int aIdx = 0;
        for (int i = 0; i < nC; ++i) {
            for (int j = 0; j < nV; ++j) {
                A[aIdx++] = static_cast<qpOASES::real_t>(A_eigen(i, j));
            }
            lbA[i] = static_cast<qpOASES::real_t>(lbA_eigen[i]);
            ubA[i] = static_cast<qpOASES::real_t>(ubA_eigen[i]);
        }

        qpOASES::QProblem problem(nV, nC);
        qpOASES::Options options;
        options.setToReliable();
        options.printLevel = qpOASES::PL_NONE;
        problem.setOptions(options);

        qpOASES::int_t nWSR = 4000;
        const auto ret = problem.init(H.data(), g.data(), A.data(), lb.data(), ub.data(),
                                      lbA.data(), ubA.data(), nWSR);

        if (ret != qpOASES::SUCCESSFUL_RETURN) {
            std::cerr << "qpOASES initialization failed with code " << ret;
            if (ret == qpOASES::RET_INIT_FAILED_INFEASIBILITY) {
                std::cerr << " (RET_INIT_FAILED_INFEASIBILITY)";
            } else if (ret == qpOASES::RET_MAX_NWSR_REACHED) {
                std::cerr << " (RET_MAX_NWSR_REACHED)";
            } else if (ret == qpOASES::RET_INIT_FAILED_CHOLESKY) {
                std::cerr << " (RET_INIT_FAILED_CHOLESKY)";
            }
            std::cerr << ", activeConstraints=" << nC << std::endl;
            return false;
        }

        std::vector<qpOASES::real_t> xOptVec(nV);
        problem.getPrimalSolution(xOptVec.data());
        solution.resize(nV);
        for (int i = 0; i < nV; ++i) {
            solution[i] = static_cast<double>(xOptVec[i]);
        }
        return true;
    };

    std::vector<int> activeSet;
    activeSet.reserve(32);
    std::vector<bool> isActive(candidates.size(), false);
    Eigen::VectorXd currentSolution = projectReferenceToBounds();

    for (int outerIter = 0; outerIter < 80; ++outerIter) {
        if (!solveSubproblem(activeSet, currentSolution)) {
            return false;
        }

        const Eigen::VectorXd curveZ = basisMat * currentSolution;
        double maxViolation = 0.0;
        int violatingCandidate = -1;
        for (size_t i = 0; i < candidates.size(); ++i) {
            const int sampleIndex = candidates[i].sampleIndex;
            const double violation = candidates[i].lowerBound - curveZ[sampleIndex];
            if (violation > maxViolation + 1e-8) {
                maxViolation = violation;
                violatingCandidate = static_cast<int>(i);
            }
        }

        if (violatingCandidate < 0) {
            xOpt = currentSolution;
            return true;
        }

        if (isActive[violatingCandidate]) {
            std::cerr << "Active-set QP stalled at sample " << candidates[violatingCandidate].sampleIndex
                      << " with remaining violation " << maxViolation << std::endl;
            return false;
        }

        activeSet.push_back(violatingCandidate);
        isActive[violatingCandidate] = true;
    }

    std::cerr << "Active-set QP exceeded outer iterations." << std::endl;
    return false;
}


void writeCsvAndPlot(const std::vector<CurvePoint>& referenceCurve,
                     const std::vector<Eigen::Vector3d>& controlPoints,
                     const std::vector<CurvePoint>& bsplineCurve,
                     const BSplineConfig& cfg) {
    const std::string csvPath = "/tmp/bspline_reference_sampling.csv";
    const std::string scriptPath = "/tmp/bspline_reference_sampling_plot.py";

    std::ofstream csv(csvPath);
    csv << "type,index,time,x,z\n";

    for (int i = 0; i <= cfg.numCurveSamples; ++i) {
        const double alpha = static_cast<double>(i) / std::max(cfg.numCurveSamples, 1);
        const double x = cfg.liftOffPos.x() + alpha * (cfg.edgeX - cfg.liftOffPos.x());
        if (x < cfg.edgeX) {
            csv << "terrain," << i << ",-1," << x << ",0\n";
        }
    }
    csv << "terrain," << cfg.numCurveSamples + 1 << ",-1," << cfg.edgeX << ",0\n";
    csv << "terrain," << cfg.numCurveSamples + 2 << ",-1," << cfg.edgeX << "," << cfg.stepHeight << "\n";
    for (int i = 0; i <= cfg.numCurveSamples; ++i) {
        const double alpha = static_cast<double>(i) / std::max(cfg.numCurveSamples, 1);
        const double x = cfg.edgeX + alpha * (cfg.touchDownPos.x() - cfg.edgeX);
        if (x > cfg.edgeX) {
            csv << "terrain," << cfg.numCurveSamples + 3 + i << ",-1," << x << "," << cfg.stepHeight << "\n";
        }
    }

    const int circleSamples = 64;
    if (stepEdgeSphere.radius > 0.0) {
        for (int i = 0; i <= circleSamples; ++i) {
            const double theta = 2.0 * std::acos(-1.0) * static_cast<double>(i) / circleSamples;
            const double sphereX = stepEdgeSphere.center.x() + stepEdgeSphere.radius * std::cos(theta);
            const double sphereZ = stepEdgeSphere.center.z() + stepEdgeSphere.radius * std::sin(theta);
            csv << "sphere," << i << ",-1," << sphereX << "," << sphereZ << "\n";
        }
    }

    for (size_t i = 0; i < referenceCurve.size(); ++i) {
        csv << "reference," << i << "," << referenceCurve[i].time << ","
            << referenceCurve[i].position.x() << "," << referenceCurve[i].position.z() << "\n";
    }

    for (size_t i = 0; i < controlPoints.size(); ++i) {
        csv << "control_point," << i << ",-1,"
            << controlPoints[i].x() << "," << controlPoints[i].z() << "\n";
    }

    for (size_t i = 0; i < bsplineCurve.size(); ++i) {
        csv << "bspline," << i << "," << bsplineCurve[i].time << ","
            << bsplineCurve[i].position.x() << "," << bsplineCurve[i].position.z() << "\n";
    }
    csv.close();

    std::ofstream py(scriptPath);
    py << "import csv\n"
       << "from collections import defaultdict\n"
         << "import matplotlib.pyplot as plt\n\n"
         << "data = defaultdict(lambda: {'x': [], 'z': [], 'idx': [], 'time': []})\n"
       << "with open('" << csvPath << "', newline='') as f:\n"
       << "    reader = csv.DictReader(f)\n"
       << "    for row in reader:\n"
       << "        key = row['type']\n"
       << "        data[key]['time'].append(float(row['time']))\n"
       << "        data[key]['x'].append(float(row['x']))\n"
       << "        data[key]['z'].append(float(row['z']))\n"
       << "        data[key]['idx'].append(int(row['index']))\n\n"
    << "fig, axes = plt.subplots(1, 2, figsize=(15.5, 5.4), gridspec_kw={'width_ratios': [1.9, 1.1]})\n"
    << "ax_curve, ax_cp = axes\n"
    << "if len(data['terrain']['x']) > 0:\n"
    << "    ax_curve.fill_between(data['terrain']['x'], -0.02, data['terrain']['z'], alpha=0.3, color='brown', label='Terrain')\n"
    << "    ax_curve.plot(data['terrain']['x'], data['terrain']['z'], 'k-', linewidth=2.0)\n"
    << "if len(data['sphere']['x']) > 0:\n"
    << "    ax_curve.plot(data['sphere']['x'], data['sphere']['z'], 'm-', linewidth=1.5, label='EdgeSphere')\n"
    << "    ax_curve.fill(data['sphere']['x'], data['sphere']['z'], color='magenta', alpha=0.18)\n"
       << "ax_curve.plot(data['reference']['x'], data['reference']['z'], 'k--', linewidth=2.0, label='Reference trajectory')\n"
    << "ax_curve.plot(data['bspline']['x'], data['bspline']['z'], 'r-', linewidth=2.2, label='Optimized B-spline curve')\n"
    << "ax_curve.plot(data['control_point']['x'], data['control_point']['z'], 'bo--', markersize=4.5, linewidth=1.0, label='Optimized control points')\n"
    << "ax_curve.plot(" << cfg.liftOffPos.x() << ", " << cfg.liftOffPos.z() << ", 'go', markersize=8, zorder=5, label='LiftOff')\n"
    << "ax_curve.plot(" << cfg.touchDownPos.x() << ", " << cfg.touchDownPos.z() << ", 'rs', markersize=8, zorder=5, label='TouchDown')\n"
    << "ax_curve.axvline(x=" << cfg.edgeX << ", color='gray', linestyle=':', alpha=0.5, label='Step edge')\n"
    << "ax_curve.set_title('Reference / B-spline / Environment')\n"
       << "ax_curve.set_xlabel('X (m)')\n"
       << "ax_curve.set_ylabel('Z (m)')\n"
       << "ax_curve.grid(True, alpha=0.3)\n"
       << "ax_curve.legend()\n"
       << "ax_curve.set_aspect('equal', adjustable='box')\n\n"
       << "ax_cp.plot(data['control_point']['idx'], data['control_point']['x'], 'o-', linewidth=1.4, markersize=5, label='x')\n"
       << "ax_cp.plot(data['control_point']['idx'], data['control_point']['z'], 'o-', linewidth=1.4, markersize=5, label='z')\n"
       << "ax_cp.set_title('Sampled Control Points')\n"
       << "ax_cp.set_xlabel('Control point index')\n"
       << "ax_cp.set_ylabel('Value (m)')\n"
       << "ax_cp.grid(True, alpha=0.3)\n"
       << "ax_cp.legend()\n\n"
       << "plt.tight_layout()\n"
       << "plt.savefig('/tmp/bspline_reference_sampling.png', dpi=150, bbox_inches='tight')\n"
       << "print('[Saved /tmp/bspline_reference_sampling.png]')\n";
    py.close();

    std::cout << "[CSV] " << csvPath << "\n";
    std::cout << "[Plot script] " << scriptPath << "\n";

    const int ret = std::system("python3 /tmp/bspline_reference_sampling_plot.py 2>/dev/null");
    if (ret != 0) {
        std::cout << "[Auto-plot failed] run manually: python3 " << scriptPath << "\n";
    }
}

int main(int /*argc*/, char* /*argv*/[]) {
    BSplineConfig cfg;

    std::cout << "========================================\n"
              << "  Reference Sampling + B-Spline Test\n"
              << "========================================\n\n"
              << "[Config]\n"
              << "  numControlPoints = " << cfg.numControlPoints << "\n"
              << "  numCurveSamples  = " << cfg.numCurveSamples << "\n"
              << "  splineOrder      = " << cfg.splineOrder << " (cubic)\n"
              << "  swingTime        = [" << cfg.swingStartTime << ", " << cfg.swingFinalTime << "]\n"
              << "  midHeight        = " << cfg.midHeight << " m\n"
              << "  stepEdgeX        = " << cfg.edgeX << " m\n"
              << "  stepHeight       = " << cfg.stepHeight << " m\n"
              << "  sphereRadius     = " << cfg.sphereRadius << " m\n\n";

    StepTerrain terrainObj(cfg.edgeX, cfg.stepHeight);
    terrain = &terrainObj;
    stepEdgeSphere.center = Eigen::Vector3d(cfg.edgeX, 0.0, cfg.stepHeight);
    stepEdgeSphere.radius = cfg.sphereRadius;

    using Node = ocs2::legged_robot::QuinticSpline::Node;
    const Node liftOff{cfg.swingStartTime, cfg.liftOffPos.z(), 0.0, 0.0};
    const Node touchDown{cfg.swingFinalTime, cfg.touchDownPos.z(), 0.0, 0.0};
    const ocs2::legged_robot::QuinticSplineCpg refSpline(liftOff, cfg.midHeight, touchDown);

    std::vector<CurvePoint> referenceCurve;
    sampleReferenceTrajectory(refSpline, cfg, cfg.numCurveSamples, referenceCurve);

    std::vector<Eigen::Vector3d> controlPoints;
    sampleReferenceTrajectoryToControlPoints(refSpline, cfg, controlPoints);

    Eigen::VectorXd x(cfg.numControlPoints);
    Eigen::VectorXd y(cfg.numControlPoints);
    Eigen::VectorXd z(cfg.numControlPoints);
    for (int i = 0; i < cfg.numControlPoints; ++i) {
        x[i] = controlPoints[i].x();
        y[i] = controlPoints[i].y();
        z[i] = controlPoints[i].z();
        // std::cout << "Control point " << i << ": x = " << x[i] << ", y = " << y[i] << ", z = " << z[i] << "\n";
    }

    BSpline bspline(cfg.splineOrder);
    bspline.setControlPoints(x, y, z);
    bspline.setKnots(bspline.generateKnots(cfg.numControlPoints, 0.0, 1.0, 0));

    const Eigen::VectorXd sampleParams = Eigen::VectorXd::LinSpaced(cfg.numCurveSamples, 0.0, 1.0);
    const Eigen::MatrixXd basisMat = bspline.generateSplineBasis(sampleParams);
    const Eigen::VectorXd sampleX = basisMat * x;
    const Eigen::VectorXd sampleY = basisMat * y;
    const Eigen::VectorXd terrainHeights = computeTerrainHeightProfile(sampleX, sampleY);
    const Eigen::VectorXd sphereHeights = computeSphereHeightProfile(sampleX, sampleY, cfg);

    Eigen::VectorXd optimizedControlZ;
    const bool qpSolved = optimizeControlPointsZ(
        basisMat,
        z,
        sampleX,
        terrainHeights,
        sphereHeights,
        cfg.groundClearance,
        cfg.controlPointSmoothnessWeight,
        cfg.controlPointZMin,
        cfg.controlPointZMax,
        optimizedControlZ);

    if (!qpSolved) {
        std::cerr << "Failed to optimize B-spline control points. Falling back to sampled reference control points." << std::endl;
        optimizedControlZ = z;
    }

    for (int i = 0; i < cfg.numControlPoints; ++i) {
        controlPoints[i].z() = optimizedControlZ[i];
    }

    bspline.setControlPoints(x, y, optimizedControlZ);

    std::vector<CurvePoint> bsplineCurve;
    evaluateBSplineCurve(bspline, cfg, bsplineCurve);

    const CollisionStats refCollisionStats = analyzeCurveCollisions(referenceCurve);
    const CollisionStats bsplineCollisionStats = analyzeCurveCollisions(bsplineCurve);

    std::cout << std::fixed << std::setprecision(4)
              << "[Generated] reference samples = " << referenceCurve.size()
              << ", control points = " << controlPoints.size()
              << ", bspline samples = " << bsplineCurve.size() << "\n";

    std::cout << "[QP] solved = " << (qpSolved ? "YES" : "NO")
              << ", smoothnessWeight = " << cfg.controlPointSmoothnessWeight
              << ", groundClearance = " << cfg.groundClearance
              << ", sphereClearance = " << cfg.sphereClearance << "\n";

    std::cout << "[Reference collisions] terrain = " << refCollisionStats.terrainCollisionCount
              << ", sphere = " << refCollisionStats.sphereCollisionCount
              << ", min terrain clearance = " << refCollisionStats.minTerrainClearance;
    if (stepEdgeSphere.radius > 0.0) {
        std::cout << ", min sphere signed distance = " << refCollisionStats.minSphereSignedDistance;
    }
    std::cout << "\n";

    std::cout << "[B-spline collisions] terrain = " << bsplineCollisionStats.terrainCollisionCount
              << ", sphere = " << bsplineCollisionStats.sphereCollisionCount
              << ", min terrain clearance = " << bsplineCollisionStats.minTerrainClearance;
    if (stepEdgeSphere.radius > 0.0) {
        std::cout << ", min sphere signed distance = " << bsplineCollisionStats.minSphereSignedDistance;
    }
    std::cout << "\n";

    writeCsvAndPlot(referenceCurve, controlPoints, bsplineCurve, cfg);
    return 0;
}