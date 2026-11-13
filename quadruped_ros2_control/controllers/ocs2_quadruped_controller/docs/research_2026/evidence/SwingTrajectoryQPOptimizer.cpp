
#include "ocs2_quadruped_controller/interface/constraint/SwingTrajectoryQPOptimizer.h"
#include "ocs2_quadruped_controller/interface/constraint/BSpline.h"

#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

constexpr double kConstraintViolationTolerance = 1e-8;

double swingProgress(double s) { return s * s * s * (10.0 + s * (-15.0 + 6.0 * s)); }
double swingProgressDerivative(double s) { return 30.0 * s * s * (1.0 - s) * (1.0 - s); }

double computePhaseClearance(double phase, double maxClearance)
{
    constexpr double kPi = 3.14159265358979323846;
    const double clampedPhase = std::min(std::max(phase, 0.0), 1.0);
    const double window = std::sin(kPi * clampedPhase);
    return maxClearance * window * window;
}

// Keep the objective grid independent of the denser safety constraint grid.
Eigen::VectorXd makeConstraintPhases(int objectiveSamples, double cbfStart, double cbfEnd)
{
    const int count = 8 * (objectiveSamples - 1) + 1;
    std::vector<double> phases;
    phases.reserve(count + 2);
    for (int j = 0; j < count; ++j) {
        phases.push_back(static_cast<double>(j) / (count - 1));
    }
    phases.push_back(cbfStart);
    phases.push_back(cbfEnd);
    std::sort(phases.begin(), phases.end());
    phases.erase(std::unique(phases.begin(), phases.end()), phases.end());
    return Eigen::Map<const Eigen::VectorXd>(phases.data(), phases.size());
}

// Four boundary control points encode position and velocity for a clamped spline.
Eigen::VectorXd fitReferenceControlPointsToCurve(const Eigen::MatrixXd& basisMat,
                                                 const Eigen::VectorXd& referenceCurveZ,
                                                 Eigen::VectorXd boundaryControlZ)
{
    const int n = static_cast<int>(boundaryControlZ.size());
    const int numFree = n - 4;
    const Eigen::MatrixXd basisFree = basisMat.middleCols(2, numFree);
    const Eigen::VectorXd adjustedReference = referenceCurveZ - basisMat * boundaryControlZ;
    Eigen::MatrixXd normalMatrix = basisFree.transpose() * basisFree;
    normalMatrix.diagonal().array() += 1e-9;
    const Eigen::LDLT<Eigen::MatrixXd> decomposition(normalMatrix);
    if (decomposition.info() != Eigen::Success) {
        throw std::runtime_error("Swing reference fit factorization failed.");
    }
    boundaryControlZ.segment(2, numFree) = decomposition.solve(basisFree.transpose() * adjustedReference);
    if (!boundaryControlZ.allFinite()) {
        throw std::runtime_error("Swing reference fit produced non-finite control points.");
    }
    return boundaryControlZ;
}

} // namespace



namespace ocs2::legged_robot
{

    // ============================================================================
    // GridMapTerrainInterface 实现
    // ============================================================================

    GridMapTerrainInterface::GridMapTerrainInterface(
        std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
        const std::string &elevationLayer)
        : planarTerrainPtr_(std::move(planarTerrainPtr)), elevationLayer_(elevationLayer)
    {
        if (!planarTerrainPtr_) {
            throw std::invalid_argument("Swing terrain interface requires a terrain map.");
        }
    }

    // Use one bilinear surface for both height and its analytic spatial derivative.
    // guardrail: the four-cell stencil must be known; unknown terrain is not flat ground.
    Eigen::Vector3d GridMapTerrainInterface::sampleTerrain(double x, double y) const
    {
        const auto& map = planarTerrainPtr_->gridMap;
        const grid_map::Position position(x, y);
        grid_map::Index index;
        if (!position.allFinite() || !map.exists(elevationLayer_) || !map.getIndex(position, index)) {
            throw std::out_of_range("Swing terrain query outside a valid elevation layer.");
        }
        grid_map::Position center;
        map.getPosition(index, center);
        const double resolution = map.getResolution();
        const grid_map::Position lower = center - grid_map::Position(
            x < center.x() ? resolution : 0.0, y < center.y() ? resolution : 0.0);
        double heights[2][2];
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                const grid_map::Position corner = lower + grid_map::Position(i * resolution, j * resolution);
                if (!map.isInside(corner)) {
                    throw std::out_of_range("Swing terrain interpolation stencil exceeds map bounds.");
                }
                heights[i][j] = map.atPosition(elevationLayer_, corner);
                if (!std::isfinite(heights[i][j])) {
                    throw std::runtime_error("Swing terrain interpolation contains unknown elevation.");
                }
            }
        }
        const double u = (x - lower.x()) / resolution;
        const double v = (y - lower.y()) / resolution;
        const double height = (1.0 - u) * ((1.0 - v) * heights[0][0] + v * heights[0][1]) +
                              u * ((1.0 - v) * heights[1][0] + v * heights[1][1]);
        const double dx = ((1.0 - v) * (heights[1][0] - heights[0][0]) +
                           v * (heights[1][1] - heights[0][1])) / resolution;
        const double dy = ((1.0 - u) * (heights[0][1] - heights[0][0]) +
                           u * (heights[1][1] - heights[1][0])) / resolution;
        return Eigen::Vector3d(height, dx, dy);
    }

    double GridMapTerrainInterface::getHeight(double x, double y) const
    {
        return sampleTerrain(x, y)[0];
    }

    Eigen::Vector2d GridMapTerrainInterface::getGradient(double x, double y) const
    {
        return sampleTerrain(x, y).tail<2>();
    }

    SwingTrajectoryQpOptimizer::SwingTrajectoryQpOptimizer(
        SwingTrajectoryQpConfig config,
        std::shared_ptr<TerrainInterface> terrainInterface)
        : terrainInterface_(std::move(terrainInterface)),
          config_(std::move(config)) {
        if (!terrainInterface_ || config_.splineOrder < 3 || config_.numControlPoints < 5 ||
            config_.numControlPoints <= config_.splineOrder ||
            config_.numCurveSamples < config_.numControlPoints || config_.qpMaxWorkingSetRecalculations < 1) {
            throw std::invalid_argument("Swing QP requires terrain, degree >= 3, >= 5 control points and sufficient samples.");
        }
        const double nonnegative[] = {
            config_.controlPointTrackingWeight, config_.positionTrackingWeight,
            config_.velocityTrackingWeight, config_.accelerationTrackingWeight,
            config_.jerkSmoothnessWeight, config_.controlPointSmoothnessWeight,
            config_.controlPointJerkWeight, config_.groundClearance, config_.liftPropagationSigma};
        for (double value : nonnegative) {
            if (!std::isfinite(value) || value < 0.0) {
                throw std::invalid_argument("Swing QP weights, clearance and lift width must be finite and nonnegative.");
            }
        }
        if (!std::isfinite(config_.cbfGamma) || config_.cbfGamma <= 0.0 ||
            !std::isfinite(config_.hessianRegularization) || config_.hessianRegularization <= 0.0 ||
            !std::isfinite(config_.cbfStartPhase) || !std::isfinite(config_.cbfEndPhase) ||
            config_.cbfStartPhase < 0.0 || config_.cbfStartPhase >= config_.cbfEndPhase ||
            config_.cbfEndPhase > 1.0 || config_.liftPropagationWindow < 0) {
            throw std::invalid_argument("Invalid swing QP regularization, CBF window/gain or lift window.");
        }
    }


    // 有限预算内增补违反约束；只有全部约束通过才接受，不删除 CBF 降级。
    bool SwingTrajectoryQpOptimizer::solveQp(
        const Eigen::MatrixXd& hessian, const Eigen::VectorXd& gradient,
        const Eigen::MatrixXd& constraints, const Eigen::VectorXd& lowerBounds,
        const Eigen::VectorXd& refControlZ, Eigen::VectorXd& solution) const
    {
        const int nV = static_cast<int>(refControlZ.size()) - 4;
        const int nC = static_cast<int>(lowerBounds.size());
        // Eliminate fixed endpoint position/velocity control points exactly.
        Eigen::VectorXd boundary = refControlZ;
        boundary.segment(2, nV).setZero();
        using QpMatrix = Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
        using QpVector = Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1>;
        const QpMatrix H = hessian.block(2, 2, nV, nV).cast<qpOASES::real_t>();
        QpMatrix A = constraints.middleCols(2, nV).cast<qpOASES::real_t>();
        const QpVector g = (gradient + hessian * boundary).segment(2, nV).cast<qpOASES::real_t>();
        QpVector lbA = (lowerBounds - constraints * boundary).cast<qpOASES::real_t>();

        for (int row = 0; row < nC; ++row) {
            const double norm = A.row(row).norm();
            if (norm > 0.0) {
                A.row(row) /= norm;
                lbA[row] /= norm;
            }
        }
        // Constraint generation: solve a subset, then check EVERY row of the dense problem.
        // A subset optimum feasible for all rows is also optimal for the full convex QP.
        QpVector primal = H.ldlt().solve(-g);
        std::vector<int> workingRows;
        int budget = config_.qpMaxWorkingSetRecalculations;
        while (primal.allFinite()) {
            Eigen::Index worstRow;
            solution = boundary;
            solution.segment(2, nV) = primal.cast<double>();
            const Eigen::VectorXd residual = constraints * solution - lowerBounds;
            if (residual.minCoeff(&worstRow) >= -kConstraintViolationTolerance) {
                solution = boundary;
                solution.segment(2, nV) = primal.cast<double>();
                if (config_.verbose) {
                    std::cerr << "Swing QP solved: variables=" << nV << ", constraints=" << nC
                              << ", generated=" << workingRows.size()
                              << ", iterations=" << config_.qpMaxWorkingSetRecalculations - budget << '\n';
                }
                return solution.allFinite() &&
                       (constraints * solution - lowerBounds).minCoeff() >= -kConstraintViolationTolerance;
            }
            if (budget <= 0 || std::find(workingRows.begin(), workingRows.end(), worstRow) != workingRows.end()) {
                return false;
            }
            workingRows.push_back(static_cast<int>(worstRow));
            QpMatrix selectedA(workingRows.size(), nV);
            QpVector selectedLower(workingRows.size());
            for (size_t row = 0; row < workingRows.size(); ++row) {
                selectedA.row(row) = A.row(workingRows[row]);
                selectedLower[row] = lbA[workingRows[row]];
            }
            qpOASES::QProblem problem(nV, static_cast<int>(workingRows.size()));
            qpOASES::Options options;
            options.setToReliable();
            options.boundTolerance = 1e-10;
            options.terminationTolerance = 1e-10;
            options.enableRegularisation = qpOASES::BT_TRUE;
            options.printLevel = qpOASES::PL_NONE;
            problem.setOptions(options);
            qpOASES::int_t nWSR = budget;
            const auto status = problem.init(H.data(), g.data(), selectedA.data(), nullptr, nullptr,
                                             selectedLower.data(), nullptr, nWSR, nullptr, primal.data());
            budget -= std::max(1, static_cast<int>(nWSR));
            if (status != qpOASES::SUCCESSFUL_RETURN ||
                problem.getPrimalSolution(primal.data()) != qpOASES::SUCCESSFUL_RETURN) {
                if (config_.verbose) { std::cerr << "Swing QP failed: status=" << status << ", generated=" << workingRows.size() << ", remaining=" << budget << '\n'; }
                return false;
            }
        }
        return false;
    }

    // 将参考 B 样条重采样到 QP 使用的控制点数量。
    void SwingTrajectoryQpOptimizer::sampleReferenceTrajectoryToControlPoints(const BSpline &refSpline,
                                                                               scalar_t swingStartTime, scalar_t swingFinalTime,
                                                                               const Eigen::Vector3d &liftOffPos3d, const Eigen::Vector3d &touchDownPos3d,
                                                                               std::vector<Eigen::Vector3d> &controlPoints) const
    {
        controlPoints.clear();
        controlPoints.reserve(config_.numControlPoints);

        const int n = config_.numControlPoints;
        BSpline referenceFitSpline(config_.splineOrder);
        referenceFitSpline.setControlPoints(Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n));
        referenceFitSpline.setKnots(referenceFitSpline.generateKnots(n, 0.0, 1.0, 0));

        const int fitSampleCount = std::max(config_.numCurveSamples, 8 * config_.numControlPoints);
        const Eigen::VectorXd fitParams = Eigen::VectorXd::LinSpaced(fitSampleCount, 0.0, 1.0);
        const Eigen::MatrixXd fitBasis = referenceFitSpline.generateSplineBasis(fitParams);
        Eigen::VectorXd referenceCurveZ(fitSampleCount);
        for (int i = 0; i < fitSampleCount; ++i)
        {
            const double t = swingStartTime + fitParams[i] * (swingFinalTime - swingStartTime);
            referenceCurveZ[i] = refSpline.getPosition(t).z();
        }

        // For clamped knots: z_s(0)=degree*(c1-c0)/firstSpan (and similarly at s=1).
        const Eigen::VectorXd knots = referenceFitSpline.generateKnots(config_.numControlPoints, 0.0, 1.0, 0);
        const int degree = config_.splineOrder;
        const double duration = swingFinalTime - swingStartTime;
        Eigen::VectorXd boundaryControlZ = Eigen::VectorXd::Zero(n);
        boundaryControlZ[0] = liftOffPos3d.z();
        boundaryControlZ[1] = liftOffPos3d.z() + refSpline.getVelocity(swingStartTime).z() * duration *
                             (knots[degree + 1] - knots[degree]) / degree;
        boundaryControlZ[n - 1] = touchDownPos3d.z();
        boundaryControlZ[n - 2] = touchDownPos3d.z() - refSpline.getVelocity(swingFinalTime).z() * duration *
                                 (knots[n] - knots[n - 1]) / degree;
        const Eigen::VectorXd fittedControlZ = fitReferenceControlPointsToCurve(
            fitBasis, referenceCurveZ, boundaryControlZ);
        Eigen::MatrixXd horizontal(fitSampleCount, 2);
        for (int i = 0; i < fitSampleCount; ++i) {
            horizontal.row(i) = (liftOffPos3d + swingProgress(fitParams[i]) * (touchDownPos3d - liftOffPos3d)).head<2>();
        }
        Eigen::VectorXd xy[2];
        for (int axis = 0; axis < 2; ++axis) {
            Eigen::VectorXd boundary = Eigen::VectorXd::Zero(n);
            boundary[0] = boundary[1] = liftOffPos3d[axis];
            boundary[n - 2] = boundary[n - 1] = touchDownPos3d[axis];
            xy[axis] = fitReferenceControlPointsToCurve(fitBasis, horizontal.col(axis), boundary);
        }
        for (int i = 0; i < n; ++i) {
            controlPoints.emplace_back(xy[0][i], xy[1][i], fittedControlZ[i]);
        }

        if (config_.verbose)
        {
            const Eigen::VectorXd fittedCurveZ = fitBasis * fittedControlZ;
            const Eigen::VectorXd fitError = fittedCurveZ - referenceCurveZ;
            const double rmsFitError = std::sqrt(fitError.squaredNorm() / std::max(1, fitSampleCount));
            const double maxFitError = fitError.cwiseAbs().maxCoeff();
            if (maxFitError > 5e-4)
            {
                std::cerr << "Reference B-spline fit error: rms=" << rmsFitError
                          << ", max=" << maxFitError << std::endl;
            }
        }
    }

    Eigen::VectorXd SwingTrajectoryQpOptimizer::computeTerrainHeightProfile(const Eigen::VectorXd &sampleX,
                                                                         const Eigen::VectorXd &sampleY) const
    {
        Eigen::VectorXd terrainHeights(sampleX.size());
        for (int i = 0; i < sampleX.size(); ++i)
        {
            terrainHeights[i] = terrainInterface_->getHeight(sampleX[i], sampleY[i]);
        }
        return terrainHeights;
    }

    bool SwingTrajectoryQpOptimizer::optimizeControlPointsZ(const Eigen::MatrixXd &basisMat,
                                                         const BSpline &curve,
                                                         const Eigen::VectorXd &refControlZ,
                                                         const Eigen::VectorXd &refCurveZ,
                                                         const Eigen::VectorXd &sampleParams,
                                                         scalar_t swingDuration,
                                                         const Eigen::VectorXd &terrainHeights,
                                                         const Eigen::Vector2d &contactOffsets,
                                                         scalar_t startTime, const HeightBounds& heightBounds,
                                                         Eigen::VectorXd &xOpt) const
    {
        const int nV = basisMat.cols();         // 优化变量数量（控制点数量）
        const int numSamples = basisMat.rows(); // 采样点数量
        // Optimize a minimum-change correction. Derivative penalties act on the correction,
        // so a collision-free nominal spline is preserved exactly. Phase derivatives make
        // the cost independent of the collocation density and swing duration.
        Eigen::MatrixXd objectiveMatrix = config_.positionTrackingWeight * basisMat.transpose() * basisMat / numSamples;
        const double weights[] = {config_.velocityTrackingWeight, config_.accelerationTrackingWeight,
                                  config_.jerkSmoothnessWeight};
        for (int order = 1; order <= 3; ++order) {
            const Eigen::MatrixXd derivative = curve.generateSplineDerivativeBasis(sampleParams, order);
            objectiveMatrix.noalias() += weights[order - 1] / numSamples * derivative.transpose() * derivative;
        }
        objectiveMatrix.diagonal().array() += config_.controlPointTrackingWeight;
        const Eigen::VectorXd objectiveVector = objectiveMatrix * refControlZ;
        (void)refCurveZ;
        (void)terrainHeights;

        Eigen::MatrixXd H_eigen = 2.0 * objectiveMatrix;
        H_eigen = 0.5 * (H_eigen + H_eigen.transpose()).eval();
        H_eigen.diagonal().array() += 2.0 * config_.hessianRegularization;
        const Eigen::VectorXd g_eigen = -2.0 * objectiveVector;

        const Eigen::VectorXd constraintPhases = makeConstraintPhases(
            numSamples, config_.cbfStartPhase, config_.cbfEndPhase);
        const Eigen::MatrixXd constraintBasis = curve.generateSplineBasis(constraintPhases);
        const Eigen::MatrixXd constraintDerivative = curve.generateSplineDerivativeBasis(constraintPhases);
        const Eigen::VectorXd constraintX = curve.getSplineX(constraintPhases);
        const Eigen::VectorXd constraintY = curve.getSplineY(constraintPhases);
        Eigen::VectorXd xPrime(constraintPhases.size()), yPrime(constraintPhases.size());
        for (int j = 0; j < constraintPhases.size(); ++j) {
            const auto velocity = curve.getVelocity(constraintPhases[j]);
            xPrime[j] = velocity.x();
            yPrime[j] = velocity.y();
        }
        // IK workspace sections vary with the body target, not the dense terrain grid.
        // guardrail: interpolate these sections; nonlinear IK is checked at the section nodes.
        const int workspaceNodes = config_.numControlPoints;
        Eigen::MatrixXd workspace(2, workspaceNodes);
        if (heightBounds) {
            for (int k = 0; k < workspaceNodes; ++k) {
                const double phase = static_cast<double>(k) / (workspaceNodes - 1);
                const auto point = curve.getPosition(phase);
                workspace.col(k) = heightBounds(startTime + phase * swingDuration, point.head<2>());
            }
        }
        const int maxConstraints = 4 * (constraintPhases.size() - 2);
        Eigen::MatrixXd constraints(maxConstraints, nV);
        Eigen::VectorXd lowerBounds(maxConstraints);
        int count = 0;
        for (int j = 1; j + 1 < constraintPhases.size(); ++j) {
            const double phase = constraintPhases[j];
            const double requiredHeight = terrainInterface_->getHeight(constraintX[j], constraintY[j]) +
                                          (1.0 - swingProgress(phase)) * contactOffsets[0] + swingProgress(phase) * contactOffsets[1] +
                                          computePhaseClearance(phase, config_.groundClearance);
            constraints.row(count) = constraintBasis.row(j);
            lowerBounds[count++] = requiredHeight;
            if (heightBounds) {
                const double index = phase * (workspaceNodes - 1);
                const int k = std::min(static_cast<int>(index), workspaceNodes - 2);
                const double fraction = index - k;
                const Eigen::Vector2d bounds = (1.0 - fraction) * workspace.col(k) + fraction * workspace.col(k + 1);
                constraints.row(count) = constraintBasis.row(j);
                lowerBounds[count++] = bounds[0];
                constraints.row(count) = -constraintBasis.row(j);
                lowerBounds[count++] = -bounds[1];
            }
            if (config_.enableCbfConstraint && phase >= config_.cbfStartPhase && phase <= config_.cbfEndPhase) {
                const Eigen::Vector2d gradient = terrainInterface_->getGradient(constraintX[j], constraintY[j]);
                constexpr double kPi = 3.14159265358979323846;
                const double clearanceDerivative = kPi * config_.groundClearance * std::sin(2.0 * kPi * phase);
                // T * (h_dot + gamma*h) >= 0; curve is parameterized on [0,1].
                constraints.row(count) = constraintDerivative.row(j) +
                                         config_.cbfGamma * swingDuration * constraintBasis.row(j);
                lowerBounds[count++] = gradient.x() * xPrime[j] + gradient.y() * yPrime[j] + clearanceDerivative +
                                       swingProgressDerivative(phase) * (contactOffsets[1] - contactOffsets[0]) +
                                       config_.cbfGamma * swingDuration * requiredHeight;
            }
        }
        constraints.conservativeResize(count, Eigen::NoChange);
        lowerBounds.conservativeResize(count);
        return solveQp(H_eigen, g_eigen, constraints, lowerBounds, refControlZ, xOpt);
    }

    // 优化整段摆动高度，并验证所构造的离散地形/CBF 约束。
    SwingTrajectoryQpOptimizer::OptimizedTrajectory SwingTrajectoryQpOptimizer::optimizeSwingTrajectory(
        const BSpline &refSpline,
        scalar_t swingStartTime,
        scalar_t swingFinalTime,
        const Eigen::Vector3d &liftOffPos3d,
        const Eigen::Vector3d &touchDownPos3d, const HeightBounds& heightBounds) const
    {
        if (!std::isfinite(swingStartTime) || !std::isfinite(swingFinalTime) ||
            swingFinalTime <= swingStartTime || !liftOffPos3d.allFinite() || !touchDownPos3d.allFinite()) {
            throw std::invalid_argument("Swing endpoints must be finite and duration must be positive.");
        }
        std::vector<Eigen::Vector3d> controlPoints; // B样条控制点
        sampleReferenceTrajectoryToControlPoints(refSpline, swingStartTime, swingFinalTime, liftOffPos3d, touchDownPos3d, controlPoints);

        Eigen::VectorXd x(config_.numControlPoints);
        Eigen::VectorXd y(config_.numControlPoints);
        Eigen::VectorXd z(config_.numControlPoints);
        for (int i = 0; i < config_.numControlPoints; ++i)
        {
            x[i] = controlPoints[i].x();
            y[i] = controlPoints[i].y();
            z[i] = controlPoints[i].z();
        }

        BSpline bspline(config_.splineOrder);
        bspline.setControlPoints(x, y, z);
        bspline.setKnots(bspline.generateKnots(config_.numControlPoints, 0.0, 1.0, 0));

        const Eigen::VectorXd sampleParams = Eigen::VectorXd::LinSpaced(config_.numCurveSamples, 0.0, 1.0);
        const Eigen::MatrixXd basisMat = bspline.generateSplineBasis(sampleParams);
        const Eigen::VectorXd sampleX = basisMat * x;
        const Eigen::VectorXd sampleY = basisMat * y;
        Eigen::VectorXd refCurveZ(config_.numCurveSamples);
        for (int i = 0; i < config_.numCurveSamples; ++i)
        {
            const double t = swingStartTime + sampleParams[i] * (swingFinalTime - swingStartTime);
            refCurveZ[i] = refSpline.getPosition(t).z();
        }
        Eigen::VectorXd terrainHeights = computeTerrainHeightProfile(sampleX, sampleY);

        OptimizedTrajectory result;
        result.startTime = swingStartTime;
        result.endTime = swingFinalTime;

        // Register the queried terrain to the measured liftoff and projected touchdown planes.
        // Otherwise millimetre map/plane differences force arbitrarily large near-endpoint curvature.
        // An already airborne foot has positive clearance, not a registration error.
        const Eigen::Vector2d contactOffsets = Eigen::Vector2d(
            liftOffPos3d.z() - terrainHeights[0],
            touchDownPos3d.z() - terrainHeights[terrainHeights.size() - 1]).cwiseMin(0.0);
        for (int j = 0; j < sampleParams.size(); ++j) {
            terrainHeights[j] += (1.0 - swingProgress(sampleParams[j])) * contactOffsets[0] + swingProgress(sampleParams[j]) * contactOffsets[1];
        }

        Eigen::VectorXd optimizedControlZ; // 优化后的控制点高度
        const bool qpSolved = optimizeControlPointsZ(
            basisMat, bspline, z, refCurveZ, sampleParams,
            swingFinalTime - swingStartTime, terrainHeights, contactOffsets, swingStartTime, heightBounds, optimizedControlZ);

        if (!qpSolved) {
            return result;
        }

        bspline.setControlPoints(x, y, optimizedControlZ);
        bspline.setKnots(bspline.generateKnots(config_.numControlPoints, swingStartTime, swingFinalTime, 0));

        // The complete dense inequalities were checked by solveQp. Retime the same coefficients;
        // repeating those rows here would mix phase-residual and physical-time tolerances.
        if (config_.verbose) {
            std::cerr << "Swing optimized: t0=" << swingStartTime << ", tf=" << swingFinalTime
                      << ", max_lift=" << (basisMat * optimizedControlZ - refCurveZ).maxCoeff()
                      << ", peak_z=" << (basisMat * optimizedControlZ).maxCoeff() << '\n';
        }
        result.collisionFree = true;
        result.bspline = std::make_shared<BSpline>(std::move(bspline));
        return result;
    }

} // namespace ocs2::legged_robot
