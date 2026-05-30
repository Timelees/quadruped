/******************************************************************************
 * B-spline swing trajectory CBF-QP demo.
 *
 * The test intentionally builds a nominal swing B-spline that intersects a
 * stair step. A conservative CBF-QP then raises only the z control points while
 * preserving the original curve shape as much as possible. The program exports
 * CSV data and a paper-ready comparison figure.
 ******************************************************************************/

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <qpOASES.hpp>

#include "BSpline.h"

namespace {

constexpr double kConstraintTolerance = 1e-8;
constexpr double kCollisionReportTolerance = 1e-4;

struct DemoConfig {
    int numControlPoints = 15;
    int numCurveSamples = 240;
    int splineOrder = 3;

    double swingStartTime = 0.0;
    double swingFinalTime = 0.42;
    Eigen::Vector3d liftOff = Eigen::Vector3d(0.05, 0.0, 0.0);
    Eigen::Vector3d touchDown = Eigen::Vector3d(0.24, 0.0, 0.08);

    double stepEdgeX = 0.10;
    double stepHeight = 0.08;
    double nominalMidHeight = 0.055;
    double groundClearance = 0.018;
    double collisionActivationMargin = 0.004;

    double cbfGamma = 3.0;
    double cbfStartPhase = 0.18;
    double cbfEndPhase = 0.82;

    int liftPropagationWindow = 24;
    double liftPropagationSigma = 8.0;
    int maxTerrainConstraints = 14;
    int maxCbfConstraints = 4;
    int qpMaxWorkingSetRecalculations = 160;

    double controlPointTrackingWeight = 0.9;
    double positionTrackingWeight = 0.9;
    double velocityTrackingWeight = 0.35;
    double accelerationTrackingWeight = 0.25;
    double jerkSmoothnessWeight = 0.10;
    double controlPointSmoothnessWeight = 0.50;
    double controlPointJerkWeight = 0.15;
    double hessianRegularization = 1e-8;
    double controlPointZMin = -0.02;
    double controlPointZMax = 0.24;
};

struct CurveSample {
    double phase = 0.0;
    double time = 0.0;
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    double terrainHeight = 0.0;
    double safetyHeight = 0.0;
    bool collision = false;
};

struct CollisionStats {
    int collisionCount = 0;
    double minClearance = std::numeric_limits<double>::infinity();
    double maxViolation = 0.0;
};

struct CandidateConstraint {
    Eigen::RowVectorXd row;
    double lowerBound = 0.0;
    int sampleIndex = -1;
    int type = 0;  // 0: terrain height, 1: CBF
    double score = 0.0;
};

double phaseToTime(double phase, const DemoConfig& cfg) {
    return cfg.swingStartTime + phase * (cfg.swingFinalTime - cfg.swingStartTime);
}

double terrainHeight(double x, const DemoConfig& cfg) {
    return (x >= cfg.stepEdgeX) ? cfg.stepHeight : 0.0;
}

double terrainGradientX(double x, const DemoConfig& cfg) {
    (void)x;
    (void)cfg;
    return 0.0;
}

double phaseClearance(double phase, double maxClearance) {
    constexpr double kPi = 3.14159265358979323846;
    const double clampedPhase = std::min(std::max(phase, 0.0), 1.0);
    const double window = std::sin(kPi * clampedPhase);
    return maxClearance * window * window;
}

Eigen::MatrixXd makeForwardDifferenceMatrix(int rows) {
    if (rows < 2) {
        return Eigen::MatrixXd(0, std::max(0, rows));
    }
    Eigen::MatrixXd diff = Eigen::MatrixXd::Zero(rows - 1, rows);
    for (int i = 0; i + 1 < rows; ++i) {
        diff(i, i) = -1.0;
        diff(i, i + 1) = 1.0;
    }
    return diff;
}

Eigen::MatrixXd makeSecondDifferenceMatrix(int rows) {
    if (rows < 3) {
        return Eigen::MatrixXd(0, std::max(0, rows));
    }
    Eigen::MatrixXd diff = Eigen::MatrixXd::Zero(rows - 2, rows);
    for (int i = 0; i + 2 < rows; ++i) {
        diff(i, i) = 1.0;
        diff(i, i + 1) = -2.0;
        diff(i, i + 2) = 1.0;
    }
    return diff;
}

Eigen::MatrixXd makeThirdDifferenceMatrix(int rows) {
    if (rows < 4) {
        return Eigen::MatrixXd(0, std::max(0, rows));
    }
    Eigen::MatrixXd diff = Eigen::MatrixXd::Zero(rows - 3, rows);
    for (int i = 0; i + 3 < rows; ++i) {
        diff(i, i) = -1.0;
        diff(i, i + 1) = 3.0;
        diff(i, i + 2) = -3.0;
        diff(i, i + 3) = 1.0;
    }
    return diff;
}

Eigen::MatrixXd makeSampleDerivativeMatrix(const Eigen::VectorXd& phases) {
    const int n = static_cast<int>(phases.size());
    Eigen::MatrixXd derivative = Eigen::MatrixXd::Zero(n, n);
    if (n < 2) {
        return derivative;
    }

    for (int i = 0; i < n; ++i) {
        if (i == 0) {
            const double ds = std::max(phases[1] - phases[0], 1e-9);
            derivative(i, 0) = -1.0 / ds;
            derivative(i, 1) = 1.0 / ds;
        } else if (i == n - 1) {
            const double ds = std::max(phases[i] - phases[i - 1], 1e-9);
            derivative(i, i - 1) = -1.0 / ds;
            derivative(i, i) = 1.0 / ds;
        } else {
            const double ds = std::max(phases[i + 1] - phases[i - 1], 1e-9);
            derivative(i, i - 1) = -1.0 / ds;
            derivative(i, i + 1) = 1.0 / ds;
        }
    }
    return derivative;
}

Eigen::VectorXd propagateLiftProfile(const Eigen::VectorXd& requiredLift,
                                     int firstSample,
                                     int lastSample,
                                     int window,
                                     double sigma) {
    Eigen::VectorXd propagated = requiredLift;
    if (window <= 0 || sigma <= 1e-9) {
        return propagated;
    }

    const double sigmaSquared = sigma * sigma;
    for (int center = firstSample; center < lastSample; ++center) {
        if (requiredLift[center] <= kConstraintTolerance) {
            continue;
        }
        const int begin = std::max(firstSample, center - window);
        const int end = std::min(lastSample, center + window + 1);
        for (int sample = begin; sample < end; ++sample) {
            const double distance = static_cast<double>(sample - center);
            const double weight = std::exp(-0.5 * distance * distance / sigmaSquared);
            propagated[sample] = std::max(propagated[sample], requiredLift[center] * weight);
        }
    }
    return propagated;
}

Eigen::VectorXd smoothLiftProfile(const Eigen::VectorXd& lift, const Eigen::VectorXd& minimumLift) {
    Eigen::VectorXd smoothed = lift;
    for (int iteration = 0; iteration < 2; ++iteration) {
        Eigen::VectorXd next = smoothed;
        for (Eigen::Index i = 1; i + 1 < smoothed.size(); ++i) {
            next[i] = 0.25 * smoothed[i - 1] + 0.5 * smoothed[i] + 0.25 * smoothed[i + 1];
        }
        smoothed = next.cwiseMax(minimumLift);
    }
    return smoothed;
}

Eigen::Vector3d interpolateFoot(const DemoConfig& cfg, double phase) {
    return cfg.liftOff + phase * (cfg.touchDown - cfg.liftOff);
}

BSpline createNominalSwingSpline(const DemoConfig& cfg,
                                 Eigen::VectorXd& controlX,
                                 Eigen::VectorXd& controlY,
                                 Eigen::VectorXd& controlZ) {
    controlX.resize(cfg.numControlPoints);
    controlY.resize(cfg.numControlPoints);
    controlZ.resize(cfg.numControlPoints);

    for (int i = 0; i < cfg.numControlPoints; ++i) {
        const double phase = static_cast<double>(i) / static_cast<double>(cfg.numControlPoints - 1);
        const Eigen::Vector3d foot = interpolateFoot(cfg, phase);
        controlX[i] = foot.x();
        controlY[i] = foot.y();

        const double linearZ = (1.0 - phase) * cfg.liftOff.z() + phase * cfg.touchDown.z();
        const double arc = 4.0 * phase * (1.0 - phase) * cfg.nominalMidHeight;
        controlZ[i] = linearZ + arc;
    }

    BSpline spline(cfg.splineOrder);
    spline.setControlPoints(controlX, controlY, controlZ);
    spline.setKnots(spline.generateKnots(cfg.numControlPoints, 0.0, 1.0, 0));
    return spline;
}

std::vector<CurveSample> sampleSpline(const BSpline& spline, const DemoConfig& cfg) {
    std::vector<CurveSample> samples;
    samples.reserve(cfg.numCurveSamples);

    const Eigen::VectorXd phases = Eigen::VectorXd::LinSpaced(cfg.numCurveSamples, 0.0, 1.0);
    const Eigen::VectorXd xs = spline.getSplineX(phases);
    const Eigen::VectorXd ys = spline.getSplineY(phases);
    const Eigen::VectorXd zs = spline.getSplineZ(phases);
    for (int i = 0; i < cfg.numCurveSamples; ++i) {
        CurveSample sample;
        sample.phase = phases[i];
        sample.time = phaseToTime(phases[i], cfg);
        sample.position = Eigen::Vector3d(xs[i], ys[i], zs[i]);
        sample.terrainHeight = terrainHeight(xs[i], cfg);
        sample.safetyHeight = sample.terrainHeight + phaseClearance(phases[i], cfg.groundClearance);
        sample.collision = zs[i] + cfg.collisionActivationMargin < sample.safetyHeight;
        samples.push_back(sample);
    }
    return samples;
}

CollisionStats analyzeCollisions(const std::vector<CurveSample>& samples) {
    CollisionStats stats;
    for (const auto& sample : samples) {
        const double clearance = sample.position.z() - sample.safetyHeight;
        stats.minClearance = std::min(stats.minClearance, clearance);
        stats.maxViolation = std::max(stats.maxViolation, -clearance);
        if (clearance < -kCollisionReportTolerance) {
            ++stats.collisionCount;
        }
    }
    return stats;
}

bool referenceRequiresOptimization(const std::vector<CurveSample>& samples, const DemoConfig& cfg) {
    for (const auto& sample : samples) {
        if (sample.position.z() + cfg.collisionActivationMargin < sample.safetyHeight) {
            return true;
        }
    }
    return false;
}

Eigen::VectorXd projectReferenceToBounds(const Eigen::VectorXd& ref,
                                         const Eigen::VectorXd& lb,
                                         const Eigen::VectorXd& ub) {
    Eigen::VectorXd projected = ref;
    for (int i = 0; i < ref.size(); ++i) {
        projected[i] = std::min(std::max(projected[i], lb[i]), ub[i]);
    }
    return projected;
}

bool solveQp(const Eigen::MatrixXd& H,
             const Eigen::VectorXd& g,
             const Eigen::VectorXd& lb,
             const Eigen::VectorXd& ub,
             const std::vector<CandidateConstraint>& candidates,
             const std::vector<int>& activeSet,
             int maxWorkingSetRecalculations,
             const Eigen::VectorXd& reference,
             Eigen::VectorXd& solution) {
    const int nV = static_cast<int>(H.cols());
    if (activeSet.empty()) {
        solution = projectReferenceToBounds(reference, lb, ub);
        return true;
    }

    const int nC = static_cast<int>(activeSet.size());
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nC, nV);
    Eigen::VectorXd lbA(nC);
    Eigen::VectorXd ubA = Eigen::VectorXd::Constant(nC, 1e20);
    for (int i = 0; i < nC; ++i) {
        A.row(i) = candidates[activeSet[i]].row;
        lbA[i] = candidates[activeSet[i]].lowerBound;
    }

    std::vector<qpOASES::real_t> hRaw(H.size());
    std::vector<qpOASES::real_t> gRaw(g.size());
    std::vector<qpOASES::real_t> lbRaw(lb.size());
    std::vector<qpOASES::real_t> ubRaw(ub.size());
    std::vector<qpOASES::real_t> aRaw(A.size());
    std::vector<qpOASES::real_t> lbARaw(lbA.size());
    std::vector<qpOASES::real_t> ubARaw(ubA.size());

    int hIdx = 0;
    for (int i = 0; i < nV; ++i) {
        for (int j = 0; j < nV; ++j) {
            hRaw[hIdx++] = static_cast<qpOASES::real_t>(H(i, j));
        }
        gRaw[i] = static_cast<qpOASES::real_t>(g[i]);
        lbRaw[i] = static_cast<qpOASES::real_t>(lb[i]);
        ubRaw[i] = static_cast<qpOASES::real_t>(ub[i]);
    }

    int aIdx = 0;
    for (int i = 0; i < nC; ++i) {
        for (int j = 0; j < nV; ++j) {
            aRaw[aIdx++] = static_cast<qpOASES::real_t>(A(i, j));
        }
        lbARaw[i] = static_cast<qpOASES::real_t>(lbA[i]);
        ubARaw[i] = static_cast<qpOASES::real_t>(ubA[i]);
    }

    qpOASES::QProblem problem(nV, nC);
    qpOASES::Options options;
    options.setToMPC();
    options.enableRegularisation = qpOASES::BT_TRUE;
    options.printLevel = qpOASES::PL_NONE;
    problem.setOptions(options);

    qpOASES::int_t nWSR = static_cast<qpOASES::int_t>(std::max(1, maxWorkingSetRecalculations));
    const auto ret = problem.init(hRaw.data(), gRaw.data(), aRaw.data(), lbRaw.data(), ubRaw.data(),
                                  lbARaw.data(), ubARaw.data(), nWSR);
    if (ret != qpOASES::SUCCESSFUL_RETURN) {
        std::cerr << "qpOASES failed with code " << ret << ", activeConstraints=" << nC << "\n";
        return false;
    }

    std::vector<qpOASES::real_t> xOpt(nV);
    problem.getPrimalSolution(xOpt.data());
    solution.resize(nV);
    for (int i = 0; i < nV; ++i) {
        solution[i] = static_cast<double>(xOpt[i]);
    }
    return true;
}

std::vector<int> selectConstraints(const std::vector<CandidateConstraint>& candidates,
                                   int numSamples,
                                   int maxTerrainConstraints,
                                   int maxCbfConstraints) {
    std::vector<int> terrain;
    std::vector<int> cbf;
    for (int i = 0; i < static_cast<int>(candidates.size()); ++i) {
        if (candidates[i].type == 0) {
            terrain.push_back(i);
        } else {
            cbf.push_back(i);
        }
    }

    auto byScore = [&](int lhs, int rhs) {
        return candidates[lhs].score > candidates[rhs].score;
    };
    std::sort(terrain.begin(), terrain.end(), byScore);
    std::sort(cbf.begin(), cbf.end(), byScore);

    std::vector<int> selected;
    auto appendSeparated = [&](const std::vector<int>& source, int limit) {
        const int firstSelected = static_cast<int>(selected.size());
        const int minSpacing = std::max(1, numSamples / std::max(1, 2 * limit));
        for (const int index : source) {
            if (static_cast<int>(selected.size()) - firstSelected >= limit) {
                return;
            }
            const int sampleIndex = candidates[index].sampleIndex;
            const bool separated = std::none_of(selected.begin() + firstSelected, selected.end(), [&](int selectedIndex) {
                return std::abs(candidates[selectedIndex].sampleIndex - sampleIndex) < minSpacing;
            });
            if (separated) {
                selected.push_back(index);
            }
        }
        for (const int index : source) {
            if (static_cast<int>(selected.size()) - firstSelected >= limit) {
                return;
            }
            if (std::find(selected.begin() + firstSelected, selected.end(), index) == selected.end()) {
                selected.push_back(index);
            }
        }
    };

    appendSeparated(terrain, maxTerrainConstraints);
    appendSeparated(cbf, maxCbfConstraints);
    return selected;
}

bool optimizeControlPointsZ(const Eigen::MatrixXd& basis,
                            const Eigen::VectorXd& refControlZ,
                            const Eigen::VectorXd& refCurveZ,
                            const Eigen::VectorXd& sampleX,
                            const Eigen::VectorXd& phases,
                            const DemoConfig& cfg,
                            Eigen::VectorXd& optimizedControlZ,
                            std::vector<int>& selectedConstraintSamples) {
    const int nV = static_cast<int>(basis.cols());
    const int numSamples = static_cast<int>(basis.rows());
    const int firstSample = std::min(numSamples - 1, 4);
    const int lastSample = std::max(firstSample, numSamples - 4);
    const double duration = std::max(cfg.swingFinalTime - cfg.swingStartTime, 1e-3);

    Eigen::VectorXd terrain(numSamples);
    Eigen::VectorXd requiredCurveZ(numSamples);
    for (int i = 0; i < numSamples; ++i) {
        terrain[i] = terrainHeight(sampleX[i], cfg);
        requiredCurveZ[i] = terrain[i] + phaseClearance(phases[i], cfg.groundClearance);
    }

    const Eigen::VectorXd baselineCurveZ = basis * refControlZ;
    const Eigen::VectorXd requiredLift = (requiredCurveZ - baselineCurveZ).cwiseMax(0.0);
    const Eigen::VectorXd propagatedLift = propagateLiftProfile(
        requiredLift, firstSample, lastSample, cfg.liftPropagationWindow, cfg.liftPropagationSigma);
    const Eigen::VectorXd smoothLift = smoothLiftProfile(propagatedLift, requiredLift);
    const Eigen::VectorXd targetCurveZ = (baselineCurveZ + smoothLift).cwiseMax(refCurveZ);

    const Eigen::MatrixXd cpSecondDiff = makeSecondDifferenceMatrix(nV);
    const Eigen::MatrixXd cpThirdDiff = makeThirdDifferenceMatrix(nV);
    const Eigen::MatrixXd forwardDiff = makeForwardDifferenceMatrix(numSamples);
    const Eigen::MatrixXd secondDiff = makeSecondDifferenceMatrix(numSamples);
    const Eigen::MatrixXd thirdDiff = makeThirdDifferenceMatrix(numSamples);

    Eigen::MatrixXd objective = Eigen::MatrixXd::Zero(nV, nV);
    Eigen::VectorXd vector = Eigen::VectorXd::Zero(nV);

    objective.diagonal().array() += cfg.controlPointTrackingWeight;
    vector += cfg.controlPointTrackingWeight * refControlZ;

    objective += cfg.positionTrackingWeight * basis.transpose() * basis;
    vector += cfg.positionTrackingWeight * basis.transpose() * targetCurveZ;

    if (cpSecondDiff.rows() > 0) {
        objective += cfg.controlPointSmoothnessWeight * cpSecondDiff.transpose() * cpSecondDiff;
    }
    if (cpThirdDiff.rows() > 0) {
        objective += cfg.controlPointJerkWeight * cpThirdDiff.transpose() * cpThirdDiff;
    }
    if (forwardDiff.rows() > 0) {
        const Eigen::MatrixXd mapping = forwardDiff * basis;
        objective += cfg.velocityTrackingWeight * mapping.transpose() * mapping;
        vector += cfg.velocityTrackingWeight * mapping.transpose() * (forwardDiff * targetCurveZ);
    }
    if (secondDiff.rows() > 0) {
        const Eigen::MatrixXd mapping = secondDiff * basis;
        objective += cfg.accelerationTrackingWeight * mapping.transpose() * mapping;
        vector += cfg.accelerationTrackingWeight * mapping.transpose() * (secondDiff * targetCurveZ);
    }
    if (thirdDiff.rows() > 0) {
        const Eigen::MatrixXd mapping = thirdDiff * basis;
        objective += cfg.jerkSmoothnessWeight * mapping.transpose() * mapping;
    }

    Eigen::MatrixXd H = 2.0 * objective;
    H = 0.5 * (H + H.transpose()).eval();
    H.diagonal().array() += 2.0 * cfg.hessianRegularization;
    const Eigen::VectorXd g = -2.0 * vector;

    Eigen::VectorXd lb = Eigen::VectorXd::Constant(nV, cfg.controlPointZMin);
    Eigen::VectorXd ub = Eigen::VectorXd::Constant(nV, cfg.controlPointZMax);
    lb[0] = refControlZ[0];
    ub[0] = refControlZ[0];
    lb[nV - 1] = refControlZ[nV - 1];
    ub[nV - 1] = refControlZ[nV - 1];

    const Eigen::MatrixXd derivativeBasis = makeSampleDerivativeMatrix(phases) * basis;
    const Eigen::VectorXd sampleXPrime = makeSampleDerivativeMatrix(phases) * sampleX;
    std::vector<CandidateConstraint> candidates;
    candidates.reserve(2 * numSamples);

    for (int j = 1; j + 1 < numSamples; ++j) {
        const double terrainLowerBound = requiredCurveZ[j];
        if (terrainLowerBound > refCurveZ[j] + kConstraintTolerance) {
            const double violation = std::max(0.0, terrainLowerBound - refCurveZ[j]);
            candidates.push_back(CandidateConstraint{
                basis.row(j), terrainLowerBound, j, 0,
                std::max(violation, targetCurveZ[j] - refCurveZ[j])});
        }

        if (j >= firstSample &&
            j < lastSample &&
            phases[j] >= cfg.cbfStartPhase &&
            phases[j] <= cfg.cbfEndPhase) {
            const double terrainDerivative = terrainGradientX(sampleX[j], cfg) * sampleXPrime[j];
            const Eigen::RowVectorXd cbfRow = derivativeBasis.row(j) + cfg.cbfGamma * duration * basis.row(j);
            const double cbfLowerBound = terrainDerivative + cfg.cbfGamma * duration * terrainLowerBound;
            const double violation = std::max(0.0, cbfLowerBound - cbfRow.dot(refControlZ));
            if (violation > kConstraintTolerance) {
                candidates.push_back(CandidateConstraint{cbfRow, cbfLowerBound, j, 1, violation});
            }
        }
    }

    const std::vector<int> activeSet = selectConstraints(
        candidates, numSamples, cfg.maxTerrainConstraints, cfg.maxCbfConstraints);
    selectedConstraintSamples.clear();
    for (const int candidateIndex : activeSet) {
        selectedConstraintSamples.push_back(candidates[candidateIndex].sampleIndex);
    }

    if (!solveQp(H, g, lb, ub, candidates, activeSet, cfg.qpMaxWorkingSetRecalculations,
                 refControlZ, optimizedControlZ)) {
        return false;
    }

    return true;
}

void writeCsvAndPlot(const std::vector<CurveSample>& nominal,
                     const std::vector<CurveSample>& optimized,
                     const Eigen::VectorXd& controlX,
                     const Eigen::VectorXd& nominalControlZ,
                     const Eigen::VectorXd& optimizedControlZ) {
    const std::string csvPath = "/tmp/bspline_cbf_qp_comparison.csv";
    const std::string scriptPath = "/tmp/bspline_cbf_qp_comparison_plot.py";
    const std::string pngPath = "/tmp/bspline_cbf_qp_comparison.png";

    std::ofstream csv(csvPath);
    csv << "type,index,phase,time,x,z,terrain,safety,collision\n";
    for (size_t i = 0; i < nominal.size(); ++i) {
        csv << "nominal," << i << "," << nominal[i].phase << "," << nominal[i].time << ","
            << nominal[i].position.x() << "," << nominal[i].position.z() << ","
            << nominal[i].terrainHeight << "," << nominal[i].safetyHeight << ","
            << (nominal[i].collision ? 1 : 0) << "\n";
    }
    for (size_t i = 0; i < optimized.size(); ++i) {
        csv << "optimized," << i << "," << optimized[i].phase << "," << optimized[i].time << ","
            << optimized[i].position.x() << "," << optimized[i].position.z() << ","
            << optimized[i].terrainHeight << "," << optimized[i].safetyHeight << ","
            << (optimized[i].collision ? 1 : 0) << "\n";
    }
    for (int i = 0; i < controlX.size(); ++i) {
        csv << "nominal_control," << i << ",-1,-1," << controlX[i] << "," << nominalControlZ[i]
            << ",nan,nan,0\n";
        csv << "optimized_control," << i << ",-1,-1," << controlX[i] << "," << optimizedControlZ[i]
            << ",nan,nan,0\n";
    }
    csv.close();

    std::ofstream py(scriptPath);
    py << "import csv\n"
       << "from collections import defaultdict\n"
       << "import math\n"
       << "import os\n"
       << "os.environ.setdefault('MPLCONFIGDIR', '/tmp/matplotlib')\n"
       << "import matplotlib.pyplot as plt\n\n"
       << "from matplotlib import font_manager as fm\n"
       << "font_candidates = [\n"
       << "    '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',\n"
       << "    '/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc',\n"
       << "    '/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf',\n"
       << "]\n"
       << "font_prop = None\n"
       << "for font_path in font_candidates:\n"
       << "    if os.path.exists(font_path):\n"
       << "        fm.fontManager.addfont(font_path)\n"
       << "        font_prop = fm.FontProperties(fname=font_path)\n"
       << "        plt.rcParams['font.sans-serif'] = [font_prop.get_name(), 'DejaVu Sans']\n"
       << "        break\n"
       << "plt.rcParams['axes.unicode_minus'] = False\n\n"
       << "csv_path = '" << csvPath << "'\n"
       << "png_path = '" << pngPath << "'\n"
       << "data = defaultdict(lambda: defaultdict(list))\n"
       << "with open(csv_path, newline='') as f:\n"
       << "    for row in csv.DictReader(f):\n"
       << "        key = row['type']\n"
       << "        for field in ['index','phase','time','x','z','terrain','safety','collision']:\n"
       << "            value = row[field]\n"
       << "            try:\n"
       << "                data[key][field].append(float(value))\n"
       << "            except ValueError:\n"
       << "                data[key][field].append(float('nan'))\n\n"
       << "fig, ax = plt.subplots(figsize=(9.2, 4.8))\n"
       << "x = data['nominal']['x']\n"
       << "terrain = data['nominal']['terrain']\n"
       << "ax.fill_between(x, [-0.025]*len(x), terrain, color='#8d6e63', alpha=0.28, label='楼梯地形')\n"
       << "ax.plot(x, terrain, color='black', linewidth=1.8, label='地形高度')\n"
       << "ax.plot(data['nominal']['x'], data['nominal']['z'], color='#d62728', linestyle='--', linewidth=2.2, label='原始B样条轨迹（碰撞）')\n"
       << "ax.plot(data['optimized']['x'], data['optimized']['z'], color='#1f77b4', linewidth=2.4, label='优化后CBF-QP B样条轨迹')\n"
       << "ax.plot(data['nominal_control']['x'], data['nominal_control']['z'], 'o--', color='#ef9a9a', markersize=4.0, linewidth=1.0, label='原始控制点')\n"
       << "ax.plot(data['optimized_control']['x'], data['optimized_control']['z'], 'o-', color='#90caf9', markersize=4.0, linewidth=1.0, label='优化后控制点')\n"
       << "ax.set_xlabel('x 位置 [m]', fontproperties=font_prop)\n"
       << "ax.set_ylabel('z 高度 [m]', fontproperties=font_prop)\n"
       << "ax.set_title('楼梯地形下摆动足B样条轨迹优化对比', fontproperties=font_prop)\n"
       << "ax.grid(True, alpha=0.28)\n"
       << "ax.set_aspect('equal', adjustable='box')\n"
       << "ax.legend(loc='lower right', fontsize=8.5, prop=font_prop, framealpha=0.88)\n"
       << "plt.tight_layout()\n"
       << "plt.savefig(png_path, dpi=220, bbox_inches='tight')\n"
       << "print('[Saved ' + png_path + ']')\n";
    py.close();

    std::cout << "[CSV] " << csvPath << "\n";
    std::cout << "[Plot script] " << scriptPath << "\n" << std::flush;
    const int ret = std::system(("python3 " + scriptPath).c_str());
    if (ret != 0) {
        std::cout << "[Auto-plot failed] run manually: python3 " << scriptPath << "\n";
    }
}

}  // namespace

int main() {
    const DemoConfig cfg;

    std::cout << "========================================\n"
              << "  B-spline CBF-QP stair collision demo\n"
              << "========================================\n"
              << std::fixed << std::setprecision(4)
              << "step edge x      : " << cfg.stepEdgeX << " m\n"
              << "step height      : " << cfg.stepHeight << " m\n"
              << "phase clearance  : " << cfg.groundClearance << " m\n"
              << "activation margin: " << cfg.collisionActivationMargin << " m\n\n";

    Eigen::VectorXd controlX;
    Eigen::VectorXd controlY;
    Eigen::VectorXd nominalControlZ;
    BSpline nominalSpline = createNominalSwingSpline(cfg, controlX, controlY, nominalControlZ);
    std::vector<CurveSample> nominalCurve = sampleSpline(nominalSpline, cfg);
    const CollisionStats nominalStats = analyzeCollisions(nominalCurve);
    const bool shouldOptimize = referenceRequiresOptimization(nominalCurve, cfg);

    std::cout << "[Nominal] collision samples = " << nominalStats.collisionCount
              << ", min clearance = " << nominalStats.minClearance
              << ", max violation = " << nominalStats.maxViolation << "\n"
              << "[Trigger] QP optimization required = " << (shouldOptimize ? "YES" : "NO") << "\n";

    if (!shouldOptimize) {
        std::cerr << "The nominal trajectory did not collide. Increase stepHeight or reduce nominalMidHeight.\n";
        return 1;
    }

    const Eigen::VectorXd phases = Eigen::VectorXd::LinSpaced(cfg.numCurveSamples, 0.0, 1.0);
    const Eigen::MatrixXd basis = nominalSpline.generateSplineBasis(phases);
    const Eigen::VectorXd sampleX = basis * controlX;
    const Eigen::VectorXd nominalCurveZ = basis * nominalControlZ;

    Eigen::VectorXd optimizedControlZ;
    std::vector<int> selectedConstraintSamples;
    const bool solved = optimizeControlPointsZ(
        basis, nominalControlZ, nominalCurveZ, sampleX, phases,
        cfg, optimizedControlZ, selectedConstraintSamples);

    if (!solved) {
        std::cerr << "Failed to solve CBF-QP for demo trajectory.\n";
        return 2;
    }

    BSpline optimizedSpline(cfg.splineOrder);
    optimizedSpline.setControlPoints(controlX, controlY, optimizedControlZ);
    optimizedSpline.setKnots(optimizedSpline.generateKnots(cfg.numControlPoints, 0.0, 1.0, 0));
    std::vector<CurveSample> optimizedCurve = sampleSpline(optimizedSpline, cfg);
    const CollisionStats optimizedStats = analyzeCollisions(optimizedCurve);

    const Eigen::MatrixXd thirdDiff = makeThirdDifferenceMatrix(cfg.numCurveSamples);
    const double nominalJerk = (thirdDiff * nominalCurveZ).norm();
    const double optimizedJerk = (thirdDiff * (basis * optimizedControlZ)).norm();

    std::cout << "[Optimized] collision samples = " << optimizedStats.collisionCount
              << ", min clearance = " << optimizedStats.minClearance
              << ", max violation = " << optimizedStats.maxViolation << "\n"
              << "[QP] selected constraints = " << selectedConstraintSamples.size()
              << ", nominal jerk norm = " << nominalJerk
              << ", optimized jerk norm = " << optimizedJerk << "\n";

    writeCsvAndPlot(nominalCurve, optimizedCurve, controlX, nominalControlZ, optimizedControlZ);
    return optimizedStats.maxViolation <= kCollisionReportTolerance ? 0 : 3;
}
