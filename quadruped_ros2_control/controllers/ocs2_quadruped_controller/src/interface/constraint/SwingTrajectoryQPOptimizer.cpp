#include "ocs2_quadruped_controller/interface/constraint/SwingTrajectoryQPOptimizer.h"
#include "ocs2_quadruped_controller/interface/constraint/TerrainSafetyCorridor.h"

#include <Eigen/Cholesky>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <qpOASES.hpp>

namespace {
constexpr double kConstraintViolationTolerance = 1e-9;
double swingProgress(double s) { return s * s * s * (10.0 + s * (-15.0 + 6.0 * s)); }
// Three control points at each end fix position, velocity and acceleration.
Eigen::VectorXd fitReferenceControlPointsToCurve(const Eigen::MatrixXd &basisMat,
                                                 const Eigen::VectorXd &referenceCurveZ,
                                                 Eigen::VectorXd boundaryControlZ) {
    const int n = static_cast<int>(boundaryControlZ.size());
    const int numFree = n - 6;
    const Eigen::MatrixXd basisFree = basisMat.middleCols(3, numFree);
    const Eigen::VectorXd adjustedReference = referenceCurveZ - basisMat * boundaryControlZ;
    Eigen::MatrixXd normalMatrix = basisFree.transpose() * basisFree;
    normalMatrix.diagonal().array() += 1e-9;
    const Eigen::LDLT<Eigen::MatrixXd> decomposition(normalMatrix);
    if (decomposition.info() != Eigen::Success) {
        throw std::runtime_error("Swing reference fit factorization failed.");
    }
    boundaryControlZ.segment(3, numFree) = decomposition.solve(basisFree.transpose() * adjustedReference);
    if (!boundaryControlZ.allFinite()) {
        throw std::runtime_error("Swing reference fit produced non-finite control points.");
    }
    return boundaryControlZ;
}

double binomial(int n, int k) {
    double value = 1;
    for (int i = 1; i <= k; ++i) {
        value *= double(n - i + 1) / i;
    }
    return value;
}
// B-spline derivative control points are linear in the position control
// points, and the derivative curve lies in their convex hull. Bounding each of
// them therefore bounds the whole swing. The bound is applied to the correction
// (solution - reference), so the reference's own motion is never clipped.
void appendCorrectionDerivativeRows(const Eigen::VectorXd &knots, int derivativeOrder, double limit,
                                    const Eigen::VectorXd &refControlZ,
                                    std::vector<Eigen::RowVectorXd> &rows,
                                    std::vector<double> &bounds) {
    const int n = static_cast<int>(knots.size()) - 6;
    std::vector<Eigen::VectorXd> coefficients(n);
    for (int i = 0; i < n; ++i) {
        coefficients[i] = Eigen::VectorXd::Unit(n, i);
    }
    Eigen::VectorXd currentKnots = knots;
    int degree = 5;
    for (int order = 0; order < derivativeOrder; ++order) {
        std::vector<Eigen::VectorXd> next(coefficients.size() - 1);
        for (size_t i = 0; i + 1 < coefficients.size(); ++i) {
            const double span = currentKnots[static_cast<int>(i) + degree + 1] -
                                currentKnots[static_cast<int>(i) + 1];
            next[i] = degree * (coefficients[i + 1] - coefficients[i]) / span;
        }
        currentKnots = currentKnots.segment(1, currentKnots.size() - 2);
        coefficients = std::move(next);
        --degree;
    }
    for (const auto &coefficient : coefficients) {
        const Eigen::RowVectorXd row = coefficient.transpose();
        const double reference = row.dot(refControlZ);
        rows.push_back(row);
        bounds.push_back(reference - limit);
        rows.push_back(-row);
        bounds.push_back(-reference - limit);
    }
}
// C2 contact-compatible envelope: maximum*smoothstep(2*min(s,1-s)).
// Each half is quintic; split at 0.5 before changing basis.
Eigen::VectorXd clearanceBezier(double a, double b, double maximum) {
    Eigen::VectorXd power = Eigen::VectorXd::Zero(6), bezier = power;
    const double origin = a >= 0.5 ? 2 * (1 - a) : 2 * a;
    const double slope = a >= 0.5 ? -2 * (b - a) : 2 * (b - a);
    const double coefficients[6] = {0, 0, 0, 10 * maximum, -15 * maximum, 6 * maximum};
    for (int k = 0; k <= 5; ++k) {
        for (int j = 0; j <= k; ++j) {
            power[j] += coefficients[k] * binomial(k, j) * std::pow(origin, k - j) * std::pow(slope, j);
        }
    }
    for (int i = 0; i <= 5; ++i) {
        for (int k = 0; k <= i; ++k) {
            bezier[i] += binomial(i, k) / binomial(5, k) * power[k];
        }
    }
    return bezier;
}
}  // namespace
namespace ocs2::legged_robot {
// Lower bound under one hull point. Two bodies share the phase: the foot sphere
// at (xy) and the trailing calf, which reaches back along the motion direction
// and therefore meets a riser the foot has already passed. Both bounds are
// taken with the same corridor margins, so neither body is assumed to dominate.
// Only the corridor margin fades at a contact. The terrain itself stays hard:
// fading it too would silently weaken the obstacle bound and the QP would answer
// by raising the whole swing instead. The contact height is a floor, because a
// foot lifting off on the ground next to a smeared map cell must not be asked
// to climb that cell's height.
double SwingTrajectoryQpOptimizer::terrainLowerBound(const Eigen::Vector2d &xy,
                                                     const Eigen::Vector2d &heading,
                                                     double envelope, double contactHeight,
                                                     double footTerrain) const {
    double calfBound = footTerrain;
    for (int k = 1; k <= config_.calfSamples; ++k) {
        const double distance = config_.calfLength * k / config_.calfSamples;
        const Eigen::Vector2d tail = xy - distance * heading;
        // The shank rises going back from the foot, so the same clearance at
        // distance d is a lower foot height by distance * slope.
        const double shankHeight =
            terrainInterface_->getHeight(tail.x(), tail.y()) - config_.calfSlope * distance;
        calfBound = std::max(calfBound, shankHeight);
    }
    const double footSphere = terrainInterface_->getSphereClearanceUpperBound(xy, xy, config_.footRadius);
    // Contact height is a floor, not another terrain elevation. Adding it
    // counted a raised landing surface twice (and zero clipped negative maps).
    return std::max(std::max(footSphere, calfBound) +
                        envelope * (config_.groundClearance + config_.trackingMargin), contactHeight);
}

SwingTrajectoryQpOptimizer::SwingTrajectoryQpOptimizer(SwingTrajectoryQpConfig config,
                                                       std::shared_ptr<TerrainInterface> terrain)
    : terrainInterface_(std::move(terrain)), config_(std::move(config)) {
    if (!terrainInterface_ || config_.splineOrder != 5 || config_.numControlPoints < 7 ||
        config_.numCurveSamples < config_.numControlPoints || config_.subdivisionsPerSpan < 1 ||
        config_.qpMaxWorkingSetRecalculations < 1 || static_cast<int>(config_.geometryMode) < 0 ||
        static_cast<int>(config_.geometryMode) > 2) {
        throw std::invalid_argument("Invalid quintic swing geometry configuration.");
    }
    if (config_.safetyCorridorEnabled &&
        (config_.geometryMode == SwingGeometryMode::Discrete ||
         std::abs(config_.corridorConfig.footRadius - config_.footRadius) > 1e-12)) {
        throw std::invalid_argument("Corridors require continuous hull mode and the same physical foot radius.");
    }
    const double values[] = {config_.controlPointTrackingWeight,
                             config_.positionTrackingWeight,
                             config_.velocityTrackingWeight,
                             config_.accelerationTrackingWeight,
                             config_.jerkSmoothnessWeight,
                             config_.groundClearance,
                             config_.footRadius,
                             config_.hessianRegularization};
    for (double value : values) {
        if (!std::isfinite(value) || value < 0) {
            throw std::invalid_argument("Swing weights/geometry must be finite and nonnegative.");
        }
    }
    if (config_.hessianRegularization == 0) {
        throw std::invalid_argument("Positive QP regularization is required.");
    }
}
// Generate violated rows within one budget; terrain and workspace remain hard.
bool SwingTrajectoryQpOptimizer::solveQp(const Eigen::MatrixXd &hessian, const Eigen::MatrixXd &constraints,
                                         const Eigen::VectorXd &lowerBounds,
                                         const Eigen::VectorXd &refControlZ, Eigen::VectorXd &solution,
                                         int &iterations) const {
    const int nV = static_cast<int>(refControlZ.size()) - 6;
    const int nC = static_cast<int>(lowerBounds.size());
    const int nQpVariables = nV;
    // Solve for a correction to the nominal spline. Fixed endpoint corrections
    // are zero. This avoids cancellation between H*c_ref and the tracking
    // gradient with jerk costs.
    using QpMatrix = Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using QpVector = Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, 1>;
    const double objectiveScale = hessian.block(3, 3, nV, nV).diagonal().maxCoeff();
    QpMatrix H = QpMatrix::Zero(nQpVariables, nQpVariables);
    H.topLeftCorner(nV, nV) = (hessian.block(3, 3, nV, nV) / objectiveScale).cast<qpOASES::real_t>();
    // Add regularization to the scaled Hessian to ensure numerical stability.
    // Use a larger regularization when constraints are tight to prevent divergence.
    // The regularization should be relative to the scaled Hessian diagonal.
    // For high obstacles, increase regularization to prevent large corrections.
    const double baseRegularization = std::max(config_.hessianRegularization, 1e-4);
    // Note: We cannot easily detect obstacle height here, so keep regularization
    // moderate. The divergence is caught by kMaxPrimalNorm below.
    for (int i = 0; i < nQpVariables; ++i) {
        H(i, i) += static_cast<qpOASES::real_t>(baseRegularization);
    }
    // Maximum allowed primal correction norm to prevent divergence.
    // If the QP solution exceeds this, we fall back to the reference.
    // 1.5 meters accommodates 8cm+ obstacle corrections (typical <0.15m).
    constexpr double kMaxPrimalNorm = 1.0;
    // Build the QP constraint matrix from the free-variable columns only.
    // A small free-variable coefficient is still a real collision constraint.
    // Normalize these rows below instead of discarding them.
    constexpr double kMinFreeVariableNorm = 1e-12;
    std::vector<int> activeRows;
    activeRows.reserve(nC);
    int infeasibleFixedRows = 0;
    for (int row = 0; row < nC; ++row) {
        const double freeNorm = constraints.row(row).segment(3, nV).norm();
        if (freeNorm > kMinFreeVariableNorm) {
            activeRows.push_back(row);
        } else if (constraints.row(row).dot(refControlZ) <
                   lowerBounds[row] - kConstraintViolationTolerance) {
            // The first and last control points are fixed by the foothold
            // schedule, so these rows carry no decision variable. Rejecting the
            // whole QP over them discards every correctable constraint as well:
            // a foot lifting off next to a 60 mm obstacle is "in violation" for
            // the sphere-expanded bound at s=0, which no trajectory could fix.
            // Keep optimizing the free rows; report the residual for diagnosis.
            ++infeasibleFixedRows;
            if (config_.verbose) {
                std::cerr << "Swing QP fixed endpoint out of bound: row=" << row
                          << ", residual=" << constraints.row(row).dot(refControlZ) - lowerBounds[row] << '\n';
            }
        }
    }
    const int nActive = static_cast<int>(activeRows.size());
    QpMatrix A(nActive, nQpVariables);
    QpVector lbA(nActive);
    for (int i = 0; i < nActive; ++i) {
        const int row = activeRows[i];
        const Eigen::VectorXd freeCoefficients = constraints.row(row).segment(3, nV);
        const double norm = freeCoefficients.norm();
        // Row scaling keeps the row magnitudes comparable for qpOASES. Dividing
        // by the raw norm is only safe while the row still moves freely: a hull
        // point that sits on a fixed contact control point has a tiny norm, and
        // dividing there amplifies it back to unit length, so a millimetre
        // shortfall becomes a metres-long correction request. The QP "solves"
        // that by driving the free controls far away, the divergence guard
        // rejects it, and the whole swing loses its optimization. Flooring the
        // divisor at 1 leaves the natural magnitude of nearly-fixed rows
        // intact: the bound stays a small real displacement, the row is simply
        // as hard as its own coefficients allow, and no constraint is dropped.
        const double scale = std::max(norm, 1.0);
        A.row(i) = freeCoefficients.cast<qpOASES::real_t>() / static_cast<qpOASES::real_t>(scale);
        lbA[i] = static_cast<qpOASES::real_t>(
            (lowerBounds[row] - constraints.row(row).dot(refControlZ)) / scale);
    }
    const int nUsable = nActive;
    QpVector g = QpVector::Zero(nQpVariables);
    // Constraint generation: solve a subset, then check EVERY active row.
    // A subset optimum feasible for all active rows is also optimal for the
    // full convex QP. Constant rows were checked against the reference above.
    QpVector primal = QpVector::Zero(nQpVariables);
    std::vector<int> workingRows;
    int budget = config_.qpMaxWorkingSetRecalculations;
    while (primal.allFinite()) {
        // Find the worst violated active constraint.
        int worstActive = -1;
        double worstResidual = 0.0;
        solution = refControlZ;
        solution.segment(3, nV) += primal.head(nV).cast<double>();
        for (int i = 0; i < nUsable; ++i) {
            const double r = constraints.row(activeRows[i]).dot(solution) - lowerBounds[activeRows[i]];
            if (r < worstResidual) {
                worstResidual = r;
                worstActive = i;
            }
        }
        if (worstActive < 0 || worstResidual >= -kConstraintViolationTolerance) {
            // Every free row is satisfied. Fixed-endpoint rows cannot be
            // improved by the QP and were reported when the rows were built, so
            // they must not veto this solution.
            if (config_.verbose) {
                std::cerr << "Swing QP solved: variables=" << nV << ", active_constraints=" << nUsable
                          << "/" << nC << ", generated=" << workingRows.size()
                          << ", iterations=" << config_.qpMaxWorkingSetRecalculations - budget
                          << ", fixed_rows_out_of_bound=" << infeasibleFixedRows << '\n';
            }
            iterations = config_.qpMaxWorkingSetRecalculations - budget;
            return true;
        }
        if (budget <= 0 || std::find(workingRows.begin(), workingRows.end(), worstActive) != workingRows.end()) {
            const int origRow = activeRows[worstActive];
            std::cerr << "Swing QP constraint generation failed: row=" << origRow
                      << ", residual=" << worstResidual << ", remaining=" << budget << '\n';
            return false;
        }
        workingRows.push_back(worstActive);
        QpMatrix selectedA(workingRows.size(), nQpVariables);
        QpVector selectedLower(workingRows.size());
        for (size_t row = 0; row < workingRows.size(); ++row) {
            selectedA.row(row) = A.row(workingRows[row]);
            selectedLower[row] = lbA[workingRows[row]];
        }
        qpOASES::QProblem problem(nQpVariables, static_cast<int>(workingRows.size()));
        qpOASES::Options options;
        options.setToReliable();
        options.boundTolerance = 1e-10;
        options.terminationTolerance = 1e-10;
        options.enableRegularisation = qpOASES::BT_TRUE;
        options.printLevel = qpOASES::PL_NONE;
        problem.setOptions(options);
        qpOASES::int_t nWSR = budget;
        // The correction box is a variable bound: every subset QP stays bounded
        // and the trust region holds even before any row is active.
        QpVector variableLower = QpVector::Constant(nQpVariables, -config_.maxCorrection);
        QpVector variableUpper = QpVector::Constant(nQpVariables, config_.maxCorrection);
        const auto status = problem.init(H.data(), g.data(), selectedA.data(), variableLower.data(),
                                         variableUpper.data(), selectedLower.data(), nullptr, nWSR,
                                         nullptr, primal.data());
        budget -= std::max(1, static_cast<int>(nWSR));
        iterations = config_.qpMaxWorkingSetRecalculations - budget;
        if (status != qpOASES::SUCCESSFUL_RETURN ||
            problem.getPrimalSolution(primal.data()) != qpOASES::SUCCESSFUL_RETURN) {
            const int origRow = activeRows[worstActive];
            std::cerr << "Swing QP failed: status=" << status << ", generated=" << workingRows.size()
                      << ", remaining=" << budget << ", row=" << origRow
                      << ", residual=" << worstResidual << '\n';
            return false;
        }
        // init() takes primal as an initial guess, not an output buffer. Check
        // the newly retrieved solution before it can be accepted as feasible.
        if (!primal.allFinite() || primal.norm() > kMaxPrimalNorm) {
            std::cerr << "Swing QP divergence detected: primal_norm=" << primal.norm()
                      << ", last_row=" << activeRows[worstActive]
                      << ", free_norm=" << A.row(workingRows.back()).norm() << '\n';
            return false;
        }
    }
    return false;
}

// 将参考 B 样条重采样到 QP 使用的控制点数量。
void SwingTrajectoryQpOptimizer::sampleReferenceTrajectoryToControlPoints(
    const BSpline &refSpline, scalar_t swingStartTime, scalar_t swingFinalTime,
    const Eigen::Vector3d &liftOffPos3d, const Eigen::Vector3d &touchDownPos3d,
    std::vector<Eigen::Vector3d> &controlPoints) const {
    controlPoints.clear();
    controlPoints.reserve(config_.numControlPoints);

    const int n = config_.numControlPoints;
    BSpline referenceFitSpline(config_.splineOrder);
    referenceFitSpline.setControlPoints(Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n),
                                        Eigen::VectorXd::Zero(n));
    referenceFitSpline.setKnots(referenceFitSpline.generateKnots(n, 0.0, 1.0, 0));

    const int fitSampleCount = std::max(config_.numCurveSamples, 8 * config_.numControlPoints);
    const Eigen::VectorXd fitParams = Eigen::VectorXd::LinSpaced(fitSampleCount, 0.0, 1.0);
    const Eigen::MatrixXd fitBasis = referenceFitSpline.generateSplineBasis(fitParams);
    Eigen::VectorXd referenceCurveZ(fitSampleCount);
    for (int i = 0; i < fitSampleCount; ++i) {
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
    const Eigen::MatrixXd endpointAcceleration =
        referenceFitSpline.generateSplineDerivativeBasis((Eigen::Vector2d() << 0.0, 1.0).finished(), 2);
    boundaryControlZ[2] = (refSpline.getAcceleration(swingStartTime).z() * duration * duration -
                           endpointAcceleration.row(0).dot(boundaryControlZ)) /
                          endpointAcceleration(0, 2);
    boundaryControlZ[n - 3] = (refSpline.getAcceleration(swingFinalTime).z() * duration * duration -
                               endpointAcceleration.row(1).dot(boundaryControlZ)) /
                              endpointAcceleration(1, n - 3);
    const Eigen::VectorXd fittedControlZ =
        fitReferenceControlPointsToCurve(fitBasis, referenceCurveZ, boundaryControlZ);
    Eigen::MatrixXd horizontal(fitSampleCount, 2);
    for (int i = 0; i < fitSampleCount; ++i) {
        horizontal.row(i) =
            (liftOffPos3d + swingProgress(fitParams[i]) * (touchDownPos3d - liftOffPos3d)).head<2>();
    }
    Eigen::VectorXd xy[2];
    for (int axis = 0; axis < 2; ++axis) {
        Eigen::VectorXd boundary = Eigen::VectorXd::Zero(n);
        boundary.head<3>().setConstant(liftOffPos3d[axis]);
        boundary.tail<3>().setConstant(touchDownPos3d[axis]);
        xy[axis] = fitReferenceControlPointsToCurve(fitBasis, horizontal.col(axis), boundary);
    }
    for (int i = 0; i < n; ++i) {
        controlPoints.emplace_back(xy[0][i], xy[1][i], fittedControlZ[i]);
    }

    if (config_.verbose) {
        const Eigen::VectorXd fittedCurveZ = fitBasis * fittedControlZ;
        const Eigen::VectorXd fitError = fittedCurveZ - referenceCurveZ;
        const double rmsFitError = std::sqrt(fitError.squaredNorm() / std::max(1, fitSampleCount));
        const double maxFitError = fitError.cwiseAbs().maxCoeff();
        if (maxFitError > 5e-4) {
            std::cerr << "Reference B-spline fit error: rms=" << rmsFitError << ", max=" << maxFitError
                      << std::endl;
        }
    }
}

SwingTrajectoryQpOptimizer::OptimizedTrajectory SwingTrajectoryQpOptimizer::optimizeSwingTrajectory(
    const BSpline &reference, scalar_t start, scalar_t end, const Eigen::Vector3d &liftOff,
    const Eigen::Vector3d &touchDown, const HeightBounds &workspace) const {
    if (!std::isfinite(start) || !std::isfinite(end) || end <= start || !liftOff.allFinite() ||
        !touchDown.allFinite()) {
        throw std::invalid_argument("Finite endpoints and positive swing duration are required.");
    }
    OptimizedTrajectory result;
    result.startTime = start;
    result.endTime = end;
    const double duration = end - start;
    std::vector<Eigen::Vector3d> fitted;
    sampleReferenceTrajectoryToControlPoints(reference, start, end, liftOff, touchDown, fitted);
    const int n = fitted.size();
    Eigen::VectorXd x(n), y(n), z(n);
    for (int i = 0; i < n; ++i) {
        x[i] = fitted[i].x();
        y[i] = fitted[i].y();
        z[i] = fitted[i].z();
    }
    BSpline curve(5);
    curve.setControlPoints(x, y, z);
    curve.setKnots(curve.generateKnots(n, 0, 1));
    SwingCollisionCertifier certifier(terrainInterface_, config_.collisionCheck);
    if (config_.checkReference) {
        result.referenceCertificate = certifier.check(curve, config_.footRadius);
        result.referenceCertificate.witnessTime = start + duration * result.referenceCertificate.witnessTime;
    }
    const Eigen::VectorXd phases = Eigen::VectorXd::LinSpaced(config_.numCurveSamples, 0, 1);
    const Eigen::MatrixXd B = curve.generateSplineBasis(phases);
    // Scan for max obstacle height in the swing path for adaptive constraint margins.
    double maxObstacleHeight = 0.0;
    try {
        for (int i = 0; i < phases.size(); ++i) {
            const auto point = curve.getPosition(phases[i]);
            const double h = terrainInterface_->getHeight(point.x(), point.y());
            maxObstacleHeight = std::max(maxObstacleHeight, h);
        }
    } catch (const TerrainUnavailable& e) {
        result.corridor.failureReason = e.what();
        return result;
    }
    // Reference peak height (mid-swing) - for logging only.
    const double refPeakHeight = z.maxCoeff();
    // Keep original position tracking weight - reducing it causes QP divergence.
    // The obstacle handling is done through constraint margins, not objective relaxation.
    Eigen::MatrixXd objective = config_.positionTrackingWeight * B.transpose() * B / phases.size();
    if (config_.verbose && maxObstacleHeight > 0.06) {
        std::cerr << "[SwingQP] Obstacle scan: maxObstacle=" << maxObstacleHeight
                  << ", refPeak=" << refPeakHeight << '\n';
    }
    // derivative bases are already expressed in s in [0,1]; scaling them by
    // 1/(2*pi)^order made the jerk term ~1e-6 of the position term, so the
    // optimizer was effectively unconstrained in smoothness and answered with
    // per-control-point wiggles. Physical-time scaling by the swing period
    // keeps the three derivative weights commensurate with position tracking.
    const double weights[] = {config_.velocityTrackingWeight, config_.accelerationTrackingWeight,
                              config_.jerkSmoothnessWeight};
    for (int order = 1; order <= 3; ++order) {
        const Eigen::MatrixXd D =
            curve.generateSplineDerivativeBasis(phases, order) / std::pow(duration, order);
        objective.noalias() += weights[order - 1] * D.transpose() * D / phases.size();
    }
    // Use original controlPointTrackingWeight - do not scale it down.
    objective.diagonal().array() += config_.controlPointTrackingWeight + config_.hessianRegularization;
    std::vector<Eigen::RowVectorXd> rows;
    std::vector<double> bounds;
    auto append = [&](const Eigen::MatrixXd &map, const Eigen::VectorXd &lower) {
        for (int i = 0; i < map.rows(); ++i) {
            rows.push_back(map.row(i));
            bounds.push_back(lower[i]);
        }
    };
    // Motion direction of the straight XY reference, used to place the calf
    // behind the foot. XY is fixed by the reference, so a single heading holds
    // for the whole swing.
    Eigen::Vector2d heading = (touchDown - liftOff).head<2>();
    heading = heading.norm() > 1e-9 ? heading.normalized() : Eigen::Vector2d::Zero();
    // Build immutable 3D cells in sphere-centre coordinates. Retain all planes;
    // project onto Z only after substituting the fixed reference XY controls.
    CorridorLinearConstraints corridorRows;
    if (config_.safetyCorridorEnabled) {
        result.corridor = buildTerrainSafetyCorridor(*terrainInterface_, curve, config_.corridorConfig,
            config_.subdivisionsPerSpan,
            config_.geometryMode == SwingGeometryMode::Bezier ? HullBasis::Bezier : HullBasis::Minvo);
        if (!result.corridor.valid) {
            if (config_.verbose) std::cerr << "Swing corridor unavailable: " << result.corridor.failureReason << '\n';
            return result;
        }
        corridorRows = corridorControlPointConstraints(result.corridor);
        for (int i = 0; i < corridorRows.A.rows(); ++i) {
            const Eigen::RowVectorXd zRow = corridorRows.A.row(i).tail(n);
            const double upper = corridorRows.b[i] - corridorRows.A.row(i).head(n).dot(x) -
                corridorRows.A.row(i).segment(n, n).dot(y);
            // Reject a violated plane that no free Z control can change. The
            // legacy terrain solver omits fixed contact rows; corridor rows must
            // never inherit that relaxation.
            if (zRow.segment(3, n - 6).norm() <= 1e-12) {
                if (zRow.dot(z) > upper + kConstraintViolationTolerance) return result;
                // Exact substitution leaves a satisfied constant inequality;
                // retain it in corridorRows for the final full-matrix check.
                continue;
            }
            rows.push_back(-zRow);
            bounds.push_back(-upper);
        }
    }
    try {
        if (config_.geometryMode == SwingGeometryMode::Discrete) {
            for (int i = 0; i < phases.size(); ++i) {
                const auto point = curve.getPosition(phases[i]);
                const double s = phases[i];
                rows.push_back(B.row(i));
                const double contactEnvelope = swingProgress(2 * std::min(s, 1 - s));
                const double contactHeight =
                    s <= 0.0 ? liftOff.z() : (s >= 1.0 ? touchDown.z() : -std::numeric_limits<double>::infinity());
                bounds.push_back(terrainLowerBound(point.head<2>().transpose(), heading, contactEnvelope,
                                                   contactHeight,
                                                   terrainInterface_->getHeight(point.x(), point.y())));
            }
        } else {
            const auto controls = curve.getControlPoints();
            for (const auto &span : extractPolynomialSegments(curve)) {
                // Fixed subdivision is the reproducible optimizer baseline. The
                // independent section-3.3 certifier subdivides adaptively, including
                // when optimization is off.
                std::vector<PolynomialSegment> pieces{span};
                if (span.start < 0.5 && span.end > 0.5) {
                    auto halves = span.split(0.5);
                    pieces = {std::move(halves.first), std::move(halves.second)};
                }
                int divisions = 1;
                while (divisions < config_.subdivisionsPerSpan) {
                    std::vector<PolynomialSegment> next;
                    for (const auto &piece : pieces) {
                        auto halves = piece.bisect();
                        next.push_back(std::move(halves.first));
                        next.push_back(std::move(halves.second));
                    }
                    pieces = std::move(next);
                    divisions *= 2;
                }
                for (const auto &piece : pieces) {
                    // Bernstein retains exact contact endpoints; no approximate root
                    // division.
                    const auto basis = (piece.start == 0 || piece.end == 1 ||
                                        config_.geometryMode == SwingGeometryMode::Bezier)
                                           ? HullBasis::Bezier
                                           : HullBasis::Minvo;
                    const Eigen::MatrixXd map = hullMap(piece, basis);
                    // XY is fixed by the reference, so the terrain bound can be
                    // evaluated per hull point instead of once for the whole
                    // piece. Using the piece's XY bounding box makes every hull
                    // point inherit the tallest obstacle anywhere in the piece:
                    // a foot lifting off on the ground next to a 60 mm obstacle
                    // then gets a 73 mm lower bound on its fixed start control
                    // point, which no trajectory can satisfy. The per-point
                    // query keeps each bound at the footprint the foot sphere
                    // actually occupies.
                    const PolynomialSegment identity{piece.start, piece.end, Eigen::MatrixXd::Identity(6, 6)};
                    const Eigen::MatrixXd xyHull = hullMap(piece, basis) * controls;
                    const Eigen::VectorXd clearance =
                        hullMap(identity, basis) *
                        clearanceBezier(piece.start, piece.end, config_.groundClearance);
                    Eigen::VectorXd lower(6);
                    for (int i = 0; i < 6; ++i) {
                        const Eigen::Vector2d xy = xyHull.row(i).head<2>().transpose();
                        // Contact-compatible margin: the C2 envelope above
                        // vanishes at both contacts, so neither the clearance nor
                        // the calf bound can demand height at the fixed lift-off
                        // and touchdown control points where the foot must meet
                        // the surface.
                        const double contactEnvelope =
                            config_.groundClearance > 0.0
                                ? clearance[i] / config_.groundClearance
                                : 0.0;
                        const double contactHeight =
                            piece.start <= 0.0 ? liftOff.z() : (piece.end >= 1.0 ? touchDown.z() : -std::numeric_limits<double>::infinity());
                        lower[i] = terrainLowerBound(xy, heading, contactEnvelope, contactHeight,
                                                     terrainInterface_->getHeight(xy.x(), xy.y()));
                    }
                    append(map, lower);
                }
            }
        }
        // Executability: keep the correction inside a box and bound its rate
        // through the derivative control points. Without them the QP will
        // happily return a 0.5 m foot lift on a 0.3 s swing, which the NMPC/WBC
        // chain cannot track and which shows up downstream as a collision or a
        // body upset. The box itself is a variable bound, not a row: it keeps
        // every subset QP bounded. Only the first derivative is bounded: the
        // second one over-constrains valid corrections (measured median
        // shortfall 13.5 in s-units on the 60 mm obstacle).
        const Eigen::VectorXd knots = curve.getKnots();
        appendCorrectionDerivativeRows(knots, 1, config_.maxVelocity * duration, z, rows, bounds);
        if (workspace) {
            // guardrail: sampled IK sections remain a kinematic approximation, not a
            // continuous certificate.
            Eigen::MatrixXd sections(2, n);
            for (int k = 0; k < n; ++k) {
                const double phase = double(k) / (n - 1);
                sections.col(k) = workspace(start + duration * phase, curve.getPosition(phase));
                if (!sections.col(k).allFinite() || sections(0, k) > sections(1, k)) {
                    throw std::invalid_argument("Invalid IK workspace bounds.");
                }
            }
            for (int i = 0; i < phases.size(); ++i) {
                const double index = phases[i] * (n - 1);
                const int k = std::min(int(index), n - 2);
                Eigen::Vector2d limits =
                    (1 - (index - k)) * sections.col(k) + (index - k) * sections.col(k + 1);
                // Ensure workspace bounds contain the reference trajectory value.
                // If IK-computed workspace excludes the reference, expand it to include
                // the reference plus a small margin. This prevents the QP from making
                // unnecessary corrections when workspace bounds are overly conservative.
                const double referenceZ = B.row(i).dot(z);
                // Small fixed margin - do not scale with obstacle height as that causes
                // QP divergence (primal_norm explodes).
                const double margin = 0.005;  // 5mm
                limits[0] = std::min(limits[0], referenceZ - margin);
                limits[1] = std::max(limits[1], referenceZ + margin);
                // For high obstacles, expand the IK-based upper bound to allow
                // QP to lift the foot higher. Scale gradually with obstacle height.
                if (maxObstacleHeight > 0.06) {
                    // Allow up to 20mm extra above the obstacle height
                    const double ikRelaxation = 0.01 * std::min(1.0, (maxObstacleHeight - 0.06) / 0.04);
                    limits[1] = std::max(limits[1], maxObstacleHeight + 0.01 + ikRelaxation);
                }
                // Additionally relax near endpoints to ensure compatibility with
                // fixed boundary control points (first/last 3 are position/velocity/accel).
                const double endpointBlend = std::min(1.0, std::min(phases[i], 1.0 - phases[i]) * 6.0);
                limits[0] = std::min(limits[0], (1 - endpointBlend) * referenceZ + endpointBlend * limits[0]);
                limits[1] = std::max(limits[1], (1 - endpointBlend) * referenceZ + endpointBlend * limits[1]);
                rows.push_back(B.row(i));
                bounds.push_back(limits[0]);
                rows.push_back(-B.row(i));
                bounds.push_back(-limits[1]);
            }
        }
    } catch (const TerrainUnavailable &e) {
        if (config_.verbose) {
            std::cerr << "Swing QP terrain unavailable: " << e.what() << '\n';
        }
        return result;
    }
    Eigen::MatrixXd A(rows.size(), n);
    Eigen::VectorXd lower(bounds.size());
    for (size_t i = 0; i < rows.size(); ++i) {
        A.row(i) = rows[i];
        lower[i] = bounds[i];
    }
    result.constraintRows = rows.size();
    Eigen::VectorXd solution;
    if (!solveQp(2 * objective, A, lower, z, solution, result.workingSetIterations)) {
        return result;
    }
    result.correctionCost = (solution - z).dot(objective * (solution - z));
    // Log workspace constraint impact on flat terrain
    if (config_.verbose && workspace) {
        double maxWorkspaceViolation = 0;
        double maxCorrection = 0;
        for (int i = 0; i < solution.size(); ++i) {
            maxCorrection = std::max(maxCorrection, std::abs(solution[i] - z[i]));
        }
        if (maxCorrection > 0.001) {  // Only log if correction is significant
            std::cerr << "[SwingQP] Workspace impact: maxCorrection=" << maxCorrection
                      << ", cost=" << result.correctionCost << '\n';
        }
    }
    if (config_.safetyCorridorEnabled) {
        Eigen::VectorXd q(3 * n);
        q << x, y, solution;
        result.corridorMaxViolation = (corridorRows.A * q - corridorRows.b).maxCoeff();
        if (result.corridorMaxViolation > kConstraintViolationTolerance) {
            return result;
        }
        result.corridorCertified = true;
    }
    curve.setControlPoints(x, y, solution);
    // Certify actual output independently of QP rows; certifier uses the original
    // obstacle prisms.
    if (config_.verifyOutput) {
        result.certificate = certifier.check(curve, config_.footRadius);
        result.certificate.witnessTime = start + duration * result.certificate.witnessTime;
        // Only reject on genuine collision detection. Unknown (budget exhausted
        // near contact points) is acceptable — the QP constraints already
        // enforce terrain clearance, and the certifier's contact-endpoint
        // limitation means it cannot fully prove segments that start/end on
        // the ground surface.
        if (result.certificate.status == CollisionStatus::CollisionDetected) {
            if (config_.verbose) {
                std::cerr << "Swing QP output certification failed: status="
                          << static_cast<int>(result.certificate.status)
                          << ", segments=" << result.certificate.checkedSegments
                          << ", subdivisions=" << result.certificate.subdivisions
                          << ", witnessTime=" << result.certificate.witnessTime << '\n';
            }
            return result;
        }
    }
    curve.setKnots(curve.generateKnots(n, start, end));
    result.bspline = std::make_shared<BSpline>(std::move(curve));
    result.solved = true;
    return result;
}
}  // namespace ocs2::legged_robot
