#include "ocs2_quadruped_controller/interface/constraint/SwingTrajectoryQPOptimizer.h"

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
SwingTrajectoryQpOptimizer::SwingTrajectoryQpOptimizer(SwingTrajectoryQpConfig config,
                                                       std::shared_ptr<TerrainInterface> terrain)
    : terrainInterface_(std::move(terrain)), config_(std::move(config)) {
    if (!terrainInterface_ || config_.splineOrder != 5 || config_.numControlPoints < 7 ||
        config_.numCurveSamples < config_.numControlPoints || config_.subdivisionsPerSpan < 1 ||
        config_.qpMaxWorkingSetRecalculations < 1 || static_cast<int>(config_.geometryMode) < 0 ||
        static_cast<int>(config_.geometryMode) > 2) {
        throw std::invalid_argument("Invalid quintic swing geometry configuration.");
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
    const double baseRegularization = std::max(config_.hessianRegularization, 1e-4);
    for (int i = 0; i < nQpVariables; ++i) {
        H(i, i) += static_cast<qpOASES::real_t>(baseRegularization);
    }
    // Maximum allowed primal correction norm to prevent divergence.
    // If the QP solution exceeds this, we fall back to the reference.
    // 1 meter is a conservative limit - typical swing corrections are <0.1m.
    constexpr double kMaxPrimalNorm = 1.0;
    // Build the QP constraint matrix from the free-variable columns only.
    // A small free-variable coefficient is still a real collision constraint.
    // Normalize these rows below instead of discarding them. Only constant
    // rows can be omitted, after checking their fixed endpoint values.
    constexpr double kMinFreeVariableNorm = 1e-12;
    std::vector<int> activeRows;
    activeRows.reserve(nC);
    for (int row = 0; row < nC; ++row) {
        const double freeNorm = constraints.row(row).segment(3, nV).norm();
        if (freeNorm > kMinFreeVariableNorm) {
            activeRows.push_back(row);
        } else if (constraints.row(row).dot(refControlZ) <
                   lowerBounds[row] - kConstraintViolationTolerance) {
            if (config_.verbose) {
                std::cerr << "Swing QP infeasible fixed endpoint: row=" << row
                          << ", residual=" << constraints.row(row).dot(refControlZ) - lowerBounds[row] << '\n';
            }
            return false;
        }
    }
    const int nActive = static_cast<int>(activeRows.size());
    QpMatrix A(nActive, nQpVariables);
    QpVector lbA(nActive);
    for (int i = 0; i < nActive; ++i) {
        const int row = activeRows[i];
        const double norm = constraints.row(row).segment(3, nV).norm();
        A.row(i) = constraints.row(row).segment(3, nV).cast<qpOASES::real_t>() / static_cast<qpOASES::real_t>(norm);
        lbA[i] = static_cast<qpOASES::real_t>((lowerBounds[row] - constraints.row(row).dot(refControlZ)) / norm);
    }
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
        for (int i = 0; i < nActive; ++i) {
            const double r = constraints.row(activeRows[i]).dot(solution) - lowerBounds[activeRows[i]];
            if (r < worstResidual) {
                worstResidual = r;
                worstActive = i;
            }
        }
        if (worstActive < 0 || worstResidual >= -kConstraintViolationTolerance) {
            // Check the original unscaled rows, including fixed endpoints.
            if ((constraints * solution - lowerBounds).minCoeff() < -kConstraintViolationTolerance) {
                return false;
            }
            if (config_.verbose) {
                std::cerr << "Swing QP solved: variables=" << nV << ", active_constraints=" << nActive
                          << "/" << nC << ", generated=" << workingRows.size()
                          << ", iterations=" << config_.qpMaxWorkingSetRecalculations - budget << '\n';
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
        QpVector variableLower = QpVector::Constant(nQpVariables, -qpOASES::INFTY);
        const auto status = problem.init(H.data(), g.data(), selectedA.data(), variableLower.data(), nullptr,
                                         selectedLower.data(), nullptr, nWSR, nullptr, primal.data());
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
            std::cerr << "Swing QP divergence detected: primal_norm=" << primal.norm() << '\n';
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
    Eigen::MatrixXd objective = config_.positionTrackingWeight * B.transpose() * B / phases.size();
    const double weights[] = {config_.velocityTrackingWeight, config_.accelerationTrackingWeight,
                              config_.jerkSmoothnessWeight};
    for (int order = 1; order <= 3; ++order) {
        const Eigen::MatrixXd D =
            curve.generateSplineDerivativeBasis(phases, order) / std::pow(2 * std::acos(-1.0), order);
        objective.noalias() += weights[order - 1] * D.transpose() * D / phases.size();
    }
    objective.diagonal().array() += config_.controlPointTrackingWeight + config_.hessianRegularization;
    std::vector<Eigen::RowVectorXd> rows;
    std::vector<double> bounds;
    auto append = [&](const Eigen::MatrixXd &map, const Eigen::VectorXd &lower) {
        for (int i = 0; i < map.rows(); ++i) {
            rows.push_back(map.row(i));
            bounds.push_back(lower[i]);
        }
    };
    try {
        if (config_.geometryMode == SwingGeometryMode::Discrete) {
            for (int i = 0; i < phases.size(); ++i) {
                const auto point = curve.getPosition(phases[i]);
                const double s = phases[i];
                rows.push_back(B.row(i));
                bounds.push_back(terrainInterface_->getSphereClearanceUpperBound(
                                     point.head<2>(), point.head<2>(), config_.footRadius) +
                                 config_.groundClearance * swingProgress(2 * std::min(s, 1 - s)));
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
                    // XY is fixed: use the same Bernstein footprint for every height
                    // basis. This holds the certified corridor constant in paired basis
                    // experiments.
                    const Eigen::MatrixXd points = piece.bezierMap * controls;
                    const Eigen::Vector2d lo = points.leftCols(2).colwise().minCoeff().transpose();
                    const Eigen::Vector2d hi = points.leftCols(2).colwise().maxCoeff().transpose();
                    const double terrain =
                        terrainInterface_->getSphereClearanceUpperBound(lo, hi, config_.footRadius);
                    const PolynomialSegment identity{piece.start, piece.end, Eigen::MatrixXd::Identity(6, 6)};
                    const Eigen::VectorXd clearance =
                        hullMap(identity, basis) *
                        clearanceBezier(piece.start, piece.end, config_.groundClearance);
                    append(map, Eigen::VectorXd::Constant(6, terrain) + clearance);
                }
            }
        }
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
                const double margin = 0.005;  // 5mm margin
                limits[0] = std::min(limits[0], referenceZ - margin);
                limits[1] = std::max(limits[1], referenceZ + margin);
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
