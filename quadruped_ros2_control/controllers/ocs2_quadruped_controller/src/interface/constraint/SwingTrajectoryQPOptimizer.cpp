
#include "ocs2_quadruped_controller/interface/constraint/SwingTrajectoryQPOptimizer.h"
#include "ocs2_quadruped_controller/interface/constraint/BSpline.h"

#include <Eigen/Cholesky>

namespace {

constexpr double kConstraintViolationTolerance = 1e-8;

Eigen::MatrixXd makeForwardDifferenceMatrix(int numSamples)
{
    if (numSamples < 2)
    {
        return Eigen::MatrixXd(0, std::max(0, numSamples));
    }

    Eigen::MatrixXd diff = Eigen::MatrixXd::Zero(numSamples - 1, numSamples);
    for (int i = 0; i < numSamples - 1; ++i)
    {
        diff(i, i) = -1.0;
        diff(i, i + 1) = 1.0;
    }
    return diff;
}

Eigen::MatrixXd makeSecondDifferenceMatrix(int numSamples)
{
    if (numSamples < 3)
    {
        return Eigen::MatrixXd(0, std::max(0, numSamples));
    }

    Eigen::MatrixXd diff = Eigen::MatrixXd::Zero(numSamples - 2, numSamples);
    for (int i = 0; i < numSamples - 2; ++i)
    {
        diff(i, i) = 1.0;
        diff(i, i + 1) = -2.0;
        diff(i, i + 2) = 1.0;
    }
    return diff;
}

Eigen::MatrixXd makeThirdDifferenceMatrix(int numSamples)
{
    if (numSamples < 4)
    {
        return Eigen::MatrixXd(0, std::max(0, numSamples));
    }

    Eigen::MatrixXd diff = Eigen::MatrixXd::Zero(numSamples - 3, numSamples);
    for (int i = 0; i < numSamples - 3; ++i)
    {
        diff(i, i) = -1.0;
        diff(i, i + 1) = 3.0;
        diff(i, i + 2) = -3.0;
        diff(i, i + 3) = 1.0;
    }
    return diff;
}

Eigen::VectorXd buildPropagatedLiftProfile(const Eigen::VectorXd &requiredLift,
                                          int firstConstrainedSample,
                                          int lastConstrainedSample,
                                          int propagationWindow,
                                          double propagationSigma)
{
    Eigen::VectorXd propagatedLift = requiredLift;
    if (propagationWindow <= 0 || propagationSigma <= 1e-9)
    {
        return propagatedLift;
    }

    const double sigmaSquared = propagationSigma * propagationSigma;
    for (int center = firstConstrainedSample; center < lastConstrainedSample; ++center)
    {
        if (requiredLift[center] <= kConstraintViolationTolerance)
        {
            continue;
        }

        const int begin = std::max(firstConstrainedSample, center - propagationWindow);
        const int end = std::min(lastConstrainedSample, center + propagationWindow + 1);
        for (int sample = begin; sample < end; ++sample)
        {
            const double distance = static_cast<double>(sample - center);
            const double weight = std::exp(-0.5 * distance * distance / sigmaSquared);
            propagatedLift[sample] = std::max(propagatedLift[sample], requiredLift[center] * weight);
        }
    }

    return propagatedLift;
}

double squaredDistanceToSegment2d(const Eigen::Vector2d &point,
                                  const Eigen::Vector2d &segmentStart,
                                  const Eigen::Vector2d &segmentEnd)
{
    const Eigen::Vector2d segment = segmentEnd - segmentStart;
    const double segmentNormSquared = segment.squaredNorm();
    if (segmentNormSquared <= 1e-12)
    {
        return (point - segmentStart).squaredNorm();
    }

    const double rawAlpha = (point - segmentStart).dot(segment) / segmentNormSquared;
    const double alpha = std::min(1.0, std::max(0.0, rawAlpha));
    const Eigen::Vector2d closestPoint = segmentStart + alpha * segment;
    return (point - closestPoint).squaredNorm();
}

double computeAdaptiveControlPointZMax(const Eigen::MatrixXd &basisMat,
                                       const Eigen::VectorXd &referenceControlZ,
                                       const Eigen::VectorXd &requiredCurveZ,
                                       int firstConstrainedSample,
                                       int lastConstrainedSample)
{
    if (basisMat.cols() == 0 || referenceControlZ.size() != basisMat.cols())
    {
        return 0.0;
    }

    const int lastControlPoint = static_cast<int>(basisMat.cols()) - 1;
    double adaptiveControlPointZMax = referenceControlZ.maxCoeff();
    for (int sample = firstConstrainedSample; sample < lastConstrainedSample; ++sample)
    {
        double fixedContribution = 0.0;
        double freeWeight = 0.0;
        for (int controlPoint = 0; controlPoint < basisMat.cols(); ++controlPoint)
        {
            const double weight = basisMat(sample, controlPoint);
            if (controlPoint == 0 || controlPoint == lastControlPoint)
            {
                fixedContribution += weight * referenceControlZ[controlPoint];
            }
            else
            {
                freeWeight += weight;
            }
        }

        if (freeWeight <= 1e-9)
        {
            continue;
        }

        adaptiveControlPointZMax = std::max(
            adaptiveControlPointZMax,
            (requiredCurveZ[sample] - fixedContribution) / freeWeight);
    }

    return adaptiveControlPointZMax;
}

Eigen::VectorXd fitReferenceControlPointsToCurve(const Eigen::MatrixXd &basisMat,
                                                 const Eigen::VectorXd &referenceCurveZ,
                                                 double startHeight,
                                                 double finalHeight)
{
    const int numControlPoints = static_cast<int>(basisMat.cols());
    Eigen::VectorXd fittedControlZ = Eigen::VectorXd::Zero(numControlPoints);
    if (numControlPoints == 0 || referenceCurveZ.size() != basisMat.rows())
    {
        return fittedControlZ;
    }

    fittedControlZ[0] = startHeight;
    if (numControlPoints == 1)
    {
        return fittedControlZ;
    }

    fittedControlZ[numControlPoints - 1] = finalHeight;
    if (numControlPoints == 2)
    {
        return fittedControlZ;
    }

    const int numFreeControlPoints = numControlPoints - 2;
    const Eigen::MatrixXd basisFree = basisMat.middleCols(1, numFreeControlPoints);
    const Eigen::VectorXd fixedContribution =
        basisMat.col(0) * startHeight + basisMat.col(numControlPoints - 1) * finalHeight;
    const Eigen::VectorXd adjustedReference = referenceCurveZ - fixedContribution;

    Eigen::MatrixXd normalMatrix = basisFree.transpose() * basisFree;
    Eigen::VectorXd rightHandSide = basisFree.transpose() * adjustedReference;

    constexpr double kReferenceFitSmoothnessWeight = 1e-4;
    constexpr double kReferenceFitJerkWeight = 1e-6;
    if (numControlPoints >= 3)
    {
        const Eigen::MatrixXd secondDiff = makeSecondDifferenceMatrix(numControlPoints);
        if (secondDiff.rows() > 0)
        {
            const Eigen::MatrixXd secondDiffFree = secondDiff.middleCols(1, numFreeControlPoints);
            const Eigen::VectorXd secondDiffFixed =
                secondDiff.col(0) * startHeight + secondDiff.col(numControlPoints - 1) * finalHeight;
            normalMatrix += kReferenceFitSmoothnessWeight * (secondDiffFree.transpose() * secondDiffFree);
            rightHandSide -= kReferenceFitSmoothnessWeight * (secondDiffFree.transpose() * secondDiffFixed);
        }
    }

    if (numControlPoints >= 4)
    {
        const Eigen::MatrixXd thirdDiff = makeThirdDifferenceMatrix(numControlPoints);
        if (thirdDiff.rows() > 0)
        {
            const Eigen::MatrixXd thirdDiffFree = thirdDiff.middleCols(1, numFreeControlPoints);
            const Eigen::VectorXd thirdDiffFixed =
                thirdDiff.col(0) * startHeight + thirdDiff.col(numControlPoints - 1) * finalHeight;
            normalMatrix += kReferenceFitJerkWeight * (thirdDiffFree.transpose() * thirdDiffFree);
            rightHandSide -= kReferenceFitJerkWeight * (thirdDiffFree.transpose() * thirdDiffFixed);
        }
    }

    constexpr double kReferenceFitRegularization = 1e-9;
    normalMatrix.diagonal().array() += kReferenceFitRegularization;

    const Eigen::LDLT<Eigen::MatrixXd> decomposition(normalMatrix);
    if (decomposition.info() != Eigen::Success)
    {
        for (int i = 1; i < numControlPoints - 1; ++i)
        {
            const double alpha = static_cast<double>(i) / static_cast<double>(numControlPoints - 1);
            fittedControlZ[i] = (1.0 - alpha) * startHeight + alpha * finalHeight;
        }
        return fittedControlZ;
    }

    fittedControlZ.segment(1, numFreeControlPoints) = decomposition.solve(rightHandSide);
    return fittedControlZ;
}

} // namespace



namespace ocs2::legged_robot
{

    // ============================================================================
    // GridMapTerrainInterface 实现
    // ============================================================================

    GridMapTerrainInterface::GridMapTerrainInterface(
        std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
        const std::string &elevationLayer,
        double epsilon)
        : planarTerrainPtr_(std::move(planarTerrainPtr)),
          elevationLayer_(elevationLayer),
          epsilon_(epsilon) {}

    double GridMapTerrainInterface::getHeight(double x, double y) const
    {
        if (!planarTerrainPtr_)
        {
            return 0.0;
        }

        const auto &gridMap = planarTerrainPtr_->gridMap;
        if (!gridMap.exists(elevationLayer_))
        {
            return 0.0;
        }

        const grid_map::Position pos(x, y);
        if (!gridMap.isInside(pos))
        {
            return 0.0;
        }

        const double height = static_cast<double>(gridMap.atPosition(elevationLayer_, pos));
        return std::isfinite(height) ? height : 0.0;
    }

    Eigen::Vector2d GridMapTerrainInterface::getGradient(double x, double y) const
    {
        if (!planarTerrainPtr_)
        {
            return Eigen::Vector2d::Zero();
        }

        const grid_map::GridMap *gridMap = &planarTerrainPtr_->gridMap;
        if (!gridMap->exists(elevationLayer_))
        {
            return Eigen::Vector2d::Zero();
        }

        const grid_map::Position pos(x, y);
        if (!gridMap->isInside(pos))
        {
            return Eigen::Vector2d::Zero();
        }

        const grid_map::Matrix *heightLayer = &gridMap->get(elevationLayer_);
        grid_map::Index index;
        if (!gridMap->getIndex(pos, index))
        {
            return Eigen::Vector2d::Zero();
        }

        grid_map::derivative::GridMapDerivative derivativeFilter;
        Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
        if (derivativeFilter.initialize(gridMap->getResolution()))
        {
            derivativeFilter.estimateGradient(*gridMap, gradient, index, *heightLayer);
            if (gradient.allFinite())
            {
                return gradient;
            }
        }
        return Eigen::Vector2d::Zero();
    }

    Eigen::Matrix2d GridMapTerrainInterface::getHessian(double x, double y) const
    {
        if (!planarTerrainPtr_)
        {
            return Eigen::Matrix2d::Zero();
        }

        const grid_map::GridMap *gridMap = &planarTerrainPtr_->gridMap;
        if (!gridMap->exists(elevationLayer_))
        {
            return Eigen::Matrix2d::Zero();
        }

        const grid_map::Position pos(x, y);
        if (!gridMap->isInside(pos))
        {
            return Eigen::Matrix2d::Zero();
        }

        const grid_map::Matrix *heightLayer = &gridMap->get(elevationLayer_);
        grid_map::Index index;
        if (!gridMap->getIndex(pos, index))
        {
            return Eigen::Matrix2d::Zero();
        }

        grid_map::derivative::GridMapDerivative derivativeFilter;
        Eigen::Vector2d gradient = Eigen::Vector2d::Zero();
        Eigen::Matrix2d hessian = Eigen::Matrix2d::Zero();
        if (derivativeFilter.initialize(gridMap->getResolution()))
        {
            derivativeFilter.estimateGradientAndCurvature(*gridMap, gradient, hessian, index, *heightLayer);
            if (gradient.allFinite() && hessian.allFinite())
            {
                return hessian;
            }
        }
        return Eigen::Matrix2d::Zero();
    }


    SwingTrajectoryQpOptimizer::SwingTrajectoryQpOptimizer(
        SwingTrajectoryQpConfig config,
        std::shared_ptr<TerrainInterface> terrainInterface)
        : terrainInterface_(std::move(terrainInterface)),
          config_(std::move(config)) {}


    Eigen::Vector3d SwingTrajectoryQpOptimizer::interpolateXY(
        const Eigen::Vector3d &liftOff,
        const Eigen::Vector3d &touchDown,
        scalar_t t, scalar_t t0, scalar_t tf) const
    {
        const double alpha = (tf > t0) ? (t - t0) / (tf - t0) : 0.0;
        return liftOff + alpha * (touchDown - liftOff);
    }

    // ============================================================================
    // EdgeSphere 筛选
    // ============================================================================

    std::vector<const EdgeSphere *> SwingTrajectoryQpOptimizer::filterNearbySpheres(
        const Eigen::Vector3d &liftOffPos3d,
        const Eigen::Vector3d &touchDownPos3d) const
    {

        std::vector<const EdgeSphere *> result;
        if (!edgeSpheresPtr_ || edgeSpheresPtr_->empty())
        {
            return result;
        }

        const Eigen::Vector2d liftOffXY = liftOffPos3d.head<2>();
        const Eigen::Vector2d touchDownXY = touchDownPos3d.head<2>();
        const double corridorMargin = std::max(0.0, config_.sphereActivationDist) +
                                      std::max(0.0, config_.sphereSafetyMargin);

        struct NearbySphereCandidate
        {
            const EdgeSphere *sphere = nullptr;
            double distanceSquared = 0.0;
        };

        std::vector<NearbySphereCandidate> candidates;
        candidates.reserve(edgeSpheresPtr_->size());

        for (const auto &sphere : *edgeSpheresPtr_)
        {
            const double inflatedRadius = std::max<double>(sphere.radius + corridorMargin, 1e-6);
            const double distanceSquared = squaredDistanceToSegment2d(sphere.center.head<2>(), liftOffXY, touchDownXY);
            if (distanceSquared <= inflatedRadius * inflatedRadius)
            {
                candidates.push_back({&sphere, distanceSquared});
            }
        }

        if (candidates.empty())
        {
            return result;
        }

        std::sort(candidates.begin(), candidates.end(), [](const NearbySphereCandidate &lhs, const NearbySphereCandidate &rhs) {
            return lhs.distanceSquared < rhs.distanceSquared;
        });

        if (config_.maxSpheresPerQp > 0 && candidates.size() > static_cast<size_t>(config_.maxSpheresPerQp))
        {
            candidates.resize(static_cast<size_t>(config_.maxSpheresPerQp));
        }

        result.reserve(candidates.size());
        for (const auto &candidate : candidates)
        {
            result.push_back(candidate.sphere);
        }

        return result;
    }

    // QP 求解
    bool SwingTrajectoryQpOptimizer::solveQp(const std::vector<int> &activeSet, Eigen::VectorXd &solution,
                                             const Eigen::MatrixXd &H_eigen,
                                             const Eigen::VectorXd &g_eigen,
                                             const Eigen::VectorXd &refZ, const int nV,
                                             const Eigen::VectorXd &lb_eigen, const Eigen::VectorXd &ub_eigen,
                                             const std::vector<CandidateConstraint> &candidates) const
    {
        if (activeSet.empty())
        {
            solution = projectReferenceToBounds(refZ, nV, lb_eigen, ub_eigen);
            return true;
        }

        const int nC = static_cast<int>(activeSet.size());
        Eigen::MatrixXd A_eigen(nC, nV);
        Eigen::VectorXd lbA_eigen(nC);
        Eigen::VectorXd ubA_eigen = Eigen::VectorXd::Constant(nC, 1e20);
        for (int i = 0; i < nC; ++i)
        {
            A_eigen.row(i) = candidates[activeSet[i]].row;
            lbA_eigen[i] = candidates[activeSet[i]].lowerBound;
        }

        std::vector<qpOASES::real_t> H(H_eigen.size());
        std::vector<qpOASES::real_t> g(g_eigen.size());
        std::vector<qpOASES::real_t> lb(lb_eigen.size());
        std::vector<qpOASES::real_t> ub(ub_eigen.size());
        std::vector<qpOASES::real_t> A(A_eigen.size());
        std::vector<qpOASES::real_t> lbA(lbA_eigen.size());
        std::vector<qpOASES::real_t> ubA(ubA_eigen.size());
        int hIdx = 0;
        for (int i = 0; i < nV; ++i)
        {
            for (int j = 0; j < nV; ++j)
            {
                H[hIdx++] = static_cast<qpOASES::real_t>(H_eigen(i, j));
            }
            g[i] = static_cast<qpOASES::real_t>(g_eigen[i]);
            lb[i] = static_cast<qpOASES::real_t>(lb_eigen[i]);
            ub[i] = static_cast<qpOASES::real_t>(ub_eigen[i]);
        }

        int aIdx = 0;
        for (int i = 0; i < nC; ++i)
        {
            for (int j = 0; j < nV; ++j)
            {
                A[aIdx++] = static_cast<qpOASES::real_t>(A_eigen(i, j));
            }
            lbA[i] = static_cast<qpOASES::real_t>(lbA_eigen[i]);
            ubA[i] = static_cast<qpOASES::real_t>(ubA_eigen[i]);
        }

        std::vector<qpOASES::real_t> xOptVec(nV);
        qpOASES::returnValue ret = qpOASES::RET_UNKNOWN_BUG;

        {
            qpOASES::QProblem problem(nV, nC);
            qpOASES::Options options;
            options.setToMPC();
            options.enableRegularisation = qpOASES::BT_TRUE;
            options.printLevel = qpOASES::PL_NONE;
            problem.setOptions(options);

            qpOASES::int_t nWSR = static_cast<qpOASES::int_t>(std::max(1, config_.qpMaxWorkingSetRecalculations));
            ret = problem.init(H.data(), g.data(), A.data(), lb.data(), ub.data(),
                               lbA.data(), ubA.data(), nWSR);

            if (ret == qpOASES::SUCCESSFUL_RETURN)
            {
                problem.getPrimalSolution(xOptVec.data());
            }
        }

        if (ret != qpOASES::SUCCESSFUL_RETURN)
        {
            {
                qpOASES::QProblem reliableProblem(nV, nC);
                qpOASES::Options reliableOptions;
                reliableOptions.setToReliable();
                reliableOptions.enableRegularisation = qpOASES::BT_TRUE;
                reliableOptions.printLevel = qpOASES::PL_NONE;
                reliableProblem.setOptions(reliableOptions);

                qpOASES::int_t nWSR = static_cast<qpOASES::int_t>(std::max(1, config_.qpMaxWorkingSetRecalculations));
                ret = reliableProblem.init(H.data(), g.data(), A.data(), lb.data(), ub.data(),
                                           lbA.data(), ubA.data(), nWSR);
                if (ret == qpOASES::SUCCESSFUL_RETURN)
                {
                    reliableProblem.getPrimalSolution(xOptVec.data());
                }
            }

            if (ret != qpOASES::SUCCESSFUL_RETURN)
            {
                std::cerr << "qpOASES initialization failed with code " << ret;
                if (ret == qpOASES::RET_INIT_FAILED_INFEASIBILITY)
                {
                    std::cerr << " (RET_INIT_FAILED_INFEASIBILITY)";
                }
                else if (ret == qpOASES::RET_MAX_NWSR_REACHED)
                {
                    std::cerr << " (RET_MAX_NWSR_REACHED)";
                }
                else if (ret == qpOASES::RET_INIT_FAILED_CHOLESKY)
                {
                    std::cerr << " (RET_INIT_FAILED_CHOLESKY)";
                }
                std::cerr << ", activeConstraints=" << nC << std::endl;
                return false;
            }
        }

        solution.resize(nV);
        for (int i = 0; i < nV; ++i)
        {
            solution[i] = static_cast<double>(xOptVec[i]);
        }
        return true;
    }

    // 将参考 B 样条重采样到 QP 使用的控制点数量。
    void SwingTrajectoryQpOptimizer::sampleReferenceTrajectoryToControlPoints(const BSpline &refSpline,
                                                                               scalar_t swingStartTime, scalar_t swingFinalTime,
                                                                               const Eigen::Vector3d &liftOffPos3d, const Eigen::Vector3d &touchDownPos3d,
                                                                               std::vector<Eigen::Vector3d> &controlPoints) const
    {
        controlPoints.clear();
        controlPoints.reserve(config_.numControlPoints);

        const Eigen::VectorXd params = Eigen::VectorXd::LinSpaced(config_.numControlPoints, 0.0, 1.0);
        Eigen::VectorXd controlPointX(config_.numControlPoints);
        Eigen::VectorXd controlPointY(config_.numControlPoints);
        for (int i = 0; i < config_.numControlPoints; ++i)
        {
            const double t = swingStartTime + params[i] * (swingFinalTime - swingStartTime);
            const Eigen::Vector3d footPos = interpolateXY(liftOffPos3d, touchDownPos3d, t, swingStartTime, swingFinalTime);
            controlPointX[i] = footPos.x();
            controlPointY[i] = footPos.y();
        }

        BSpline referenceFitSpline(config_.splineOrder);
        referenceFitSpline.setControlPoints(controlPointX, controlPointY, Eigen::VectorXd::Zero(config_.numControlPoints));
        referenceFitSpline.setKnots(referenceFitSpline.generateKnots(config_.numControlPoints, 0.0, 1.0, 0));

        const int fitSampleCount = std::max(config_.numCurveSamples, 8 * config_.numControlPoints);
        const Eigen::VectorXd fitParams = Eigen::VectorXd::LinSpaced(fitSampleCount, 0.0, 1.0);
        const Eigen::MatrixXd fitBasis = referenceFitSpline.generateSplineBasis(fitParams);
        Eigen::VectorXd referenceCurveZ(fitSampleCount);
        for (int i = 0; i < fitSampleCount; ++i)
        {
            referenceCurveZ[i] = refSpline.getPosition(fitParams[i]).z();
        }

        const Eigen::VectorXd fittedControlZ = fitReferenceControlPointsToCurve(
            fitBasis, referenceCurveZ, liftOffPos3d.z(), touchDownPos3d.z());
        for (int i = 0; i < config_.numControlPoints; ++i)
        {
            controlPoints.emplace_back(controlPointX[i], controlPointY[i], fittedControlZ[i]);
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

    Eigen::VectorXd SwingTrajectoryQpOptimizer::computeSphereHeightProfile(const Eigen::VectorXd &sampleX,
                                                                        const Eigen::VectorXd &sampleY,
                                                                        const std::vector<const EdgeSphere *> &nearbySpheres) const
    {
        Eigen::VectorXd sphereHeights = Eigen::VectorXd::Constant(
            sampleX.size(), -std::numeric_limits<double>::infinity());

        if (nearbySpheres.empty())
        {
            return sphereHeights;
        }

        for (auto spherePtr : nearbySpheres)
        {
            const EdgeSphere &sphere = *spherePtr;
            const double safeRadius = std::max<double>(sphere.radius + config_.sphereSafetyMargin, 1e-6);

            for (int i = 0; i < sampleX.size(); ++i)
            {
                const double dx = sampleX[i] - sphere.center.x();
                const double dy = sampleY[i] - sphere.center.y();
                const double radialTerm = safeRadius * safeRadius - dx * dx - dy * dy;
                if (radialTerm > 0.0)
                {
                    double candidateHeight = sphere.center.z() + std::sqrt(radialTerm);
                    if (candidateHeight > sphereHeights[i])
                    {
                        sphereHeights[i] = candidateHeight;
                    }
                }
            }
        }

        return sphereHeights;
    }

    Eigen::VectorXd SwingTrajectoryQpOptimizer::projectReferenceToBounds(const Eigen::VectorXd &refZ, const int nV,
                                                                          const Eigen::VectorXd &lb_eigen, const Eigen::VectorXd &ub_eigen) const
    {
        // 将参考轨迹的控制点高度投影到 [controlPointZMin, controlPointZMax] 范围内，确保初始QP可行
        Eigen::VectorXd projected = refZ;
        for (int i = 0; i < nV; ++i)
        {
            projected[i] = std::min(std::max(projected[i], lb_eigen[i]), ub_eigen[i]);
        }
        return projected;
    }

    bool SwingTrajectoryQpOptimizer::optimizeControlPointsZ(const Eigen::MatrixXd &basisMat,
                                                         const Eigen::VectorXd &refControlZ,
                                                         const Eigen::VectorXd &refCurveZ,
                                                         const Eigen::VectorXd &sampleX,
                                                         const Eigen::VectorXd &terrainHeights,
                                                         const Eigen::VectorXd &sphereHeights,
                                                         Eigen::VectorXd &xOpt) const
    {
        const int nV = basisMat.cols();         // 优化变量数量（控制点数量）
        const int numSamples = basisMat.rows(); // 采样点数量
        if (refControlZ.size() != nV || refCurveZ.size() != numSamples || sampleX.size() != numSamples ||
            terrainHeights.size() != numSamples || sphereHeights.size() != numSamples)
        {
            std::cerr << "Invalid dimensions for optimizeControlPointsZ." << std::endl;
            return false;
        }

        const int ignoredBoundarySamples = std::max(1, config_.ignoreSamples);
        const int firstConstrainedSample = std::min(numSamples - 1, ignoredBoundarySamples);
        const int lastConstrainedSample = std::max(firstConstrainedSample, numSamples - ignoredBoundarySamples);
        const Eigen::VectorXd baselineCurveZ = basisMat * refControlZ;

        Eigen::VectorXd requiredCurveZ = terrainHeights.array() + config_.groundClearance;
        for (int i = 0; i < sphereHeights.size(); ++i)
        {
            if (std::isfinite(sphereHeights[i]))
            {
                requiredCurveZ[i] = std::max(requiredCurveZ[i], sphereHeights[i]);
            }
        }

        const Eigen::VectorXd requiredLiftCurve = (requiredCurveZ - baselineCurveZ).cwiseMax(0.0);
        const Eigen::VectorXd propagatedLiftCurve = buildPropagatedLiftProfile(
            requiredLiftCurve, firstConstrainedSample, lastConstrainedSample,
            config_.liftPropagationWindow, config_.liftPropagationSigma);
        const Eigen::VectorXd propagatedRequiredCurveZ = baselineCurveZ + propagatedLiftCurve;
        const Eigen::VectorXd targetCurveZ = propagatedRequiredCurveZ.cwiseMax(refCurveZ);

        const Eigen::MatrixXd controlPointSecondDiff = makeSecondDifferenceMatrix(nV);
        const Eigen::MatrixXd controlPointThirdDiff = makeThirdDifferenceMatrix(nV);
        const Eigen::MatrixXd forwardDiff = makeForwardDifferenceMatrix(numSamples);
        const Eigen::MatrixXd secondDiff = makeSecondDifferenceMatrix(numSamples);
        const Eigen::MatrixXd thirdDiff = makeThirdDifferenceMatrix(numSamples);

        Eigen::MatrixXd objectiveMatrix = Eigen::MatrixXd::Zero(nV, nV);
        Eigen::VectorXd objectiveVector = Eigen::VectorXd::Zero(nV);

        if (config_.controlPointTrackingWeight > 0.0)
        {
            objectiveMatrix.diagonal().array() += config_.controlPointTrackingWeight;
            objectiveVector += config_.controlPointTrackingWeight * refControlZ;
        }

        if (config_.positionTrackingWeight > 0.0)
        {
            objectiveMatrix += config_.positionTrackingWeight * (basisMat.transpose() * basisMat);
            objectiveVector += config_.positionTrackingWeight * (basisMat.transpose() * targetCurveZ);
        }

        if (controlPointSecondDiff.rows() > 0 && config_.controlPointSmoothnessWeight > 0.0)
        {
            objectiveMatrix += config_.controlPointSmoothnessWeight *
                               (controlPointSecondDiff.transpose() * controlPointSecondDiff);
        }

        if (controlPointThirdDiff.rows() > 0 && config_.controlPointJerkWeight > 0.0)
        {
            objectiveMatrix += config_.controlPointJerkWeight *
                               (controlPointThirdDiff.transpose() * controlPointThirdDiff);
        }

        if (forwardDiff.rows() > 0 && config_.velocityTrackingWeight > 0.0)
        {
            const Eigen::MatrixXd velocityMapping = forwardDiff * basisMat;
            const Eigen::VectorXd referenceVelocity = forwardDiff * targetCurveZ;
            objectiveMatrix += config_.velocityTrackingWeight * (velocityMapping.transpose() * velocityMapping);
            objectiveVector += config_.velocityTrackingWeight * (velocityMapping.transpose() * referenceVelocity);
        }

        if (secondDiff.rows() > 0 && config_.accelerationTrackingWeight > 0.0)
        {
            const Eigen::MatrixXd accelerationMapping = secondDiff * basisMat;
            const Eigen::VectorXd referenceAcceleration = secondDiff * targetCurveZ;
            objectiveMatrix += config_.accelerationTrackingWeight * (accelerationMapping.transpose() * accelerationMapping);
            objectiveVector += config_.accelerationTrackingWeight * (accelerationMapping.transpose() * referenceAcceleration);
        }

        if (thirdDiff.rows() > 0 && config_.jerkSmoothnessWeight > 0.0)
        {
            const Eigen::MatrixXd jerkMapping = thirdDiff * basisMat;
            objectiveMatrix += config_.jerkSmoothnessWeight * (jerkMapping.transpose() * jerkMapping);
        }

        Eigen::MatrixXd H_eigen = 2.0 * objectiveMatrix;
        H_eigen = 0.5 * (H_eigen + H_eigen.transpose()).eval();
        H_eigen.diagonal().array() += 2.0 * std::max(0.0, config_.hessianRegularization);
        const Eigen::VectorXd g_eigen = -2.0 * objectiveVector;

        double requiredUpperBound = terrainHeights.maxCoeff() + config_.groundClearance;
        for (int i = 0; i < sphereHeights.size(); ++i)
        {
            if (std::isfinite(sphereHeights[i]))
            {
                requiredUpperBound = std::max<double>(requiredUpperBound, sphereHeights[i]);
            }
        }
        requiredUpperBound = std::max<double>(requiredUpperBound, refControlZ.maxCoeff());
        constexpr double kControlPointZPadding = 0.03;
        const double adaptiveControlPointZMax = computeAdaptiveControlPointZMax(
            basisMat, refControlZ, requiredCurveZ, firstConstrainedSample, lastConstrainedSample);
        const double dynamicControlPointZMax = std::max(
            std::max<double>(config_.controlPointZMax, requiredUpperBound + kControlPointZPadding),
            adaptiveControlPointZMax + kControlPointZPadding);

        // if (config_.verbose && dynamicControlPointZMax > requiredUpperBound + kControlPointZPadding + 1e-6)
        // {
        //     std::cerr << "Raised control-point z max to " << dynamicControlPointZMax
        //               << " to preserve boundary reachability." << std::endl;
        // }

        Eigen::VectorXd lb_eigen = Eigen::VectorXd::Constant(nV, config_.controlPointZMin);
        Eigen::VectorXd ub_eigen = Eigen::VectorXd::Constant(nV, dynamicControlPointZMax);
        lb_eigen[0] = refControlZ[0];
        ub_eigen[0] = refControlZ[0];
        lb_eigen[nV - 1] = refControlZ[nV - 1];
        ub_eigen[nV - 1] = refControlZ[nV - 1];

        std::vector<CandidateConstraint> candidates;
        candidates.reserve(2 * numSamples);
        for (int j = firstConstrainedSample; j < lastConstrainedSample; ++j)
        {
            const double lowerBound = terrainHeights[j] + config_.groundClearance;
            if (lowerBound > kConstraintViolationTolerance)
            {
                candidates.push_back(CandidateConstraint{
                    basisMat.row(j), lowerBound, j, 0,
                    std::max(0.0, targetCurveZ[j] - refCurveZ[j])});
            }
        }
        for (int j = firstConstrainedSample; j < lastConstrainedSample; ++j)
        {
            if (std::isfinite(sphereHeights[j]))
            {
                const double lowerBound = sphereHeights[j];
                if (lowerBound > kConstraintViolationTolerance)
                {
                    candidates.push_back(CandidateConstraint{
                        basisMat.row(j), lowerBound, j, 1,
                        std::max(0.0, targetCurveZ[j] - refCurveZ[j])});
                }
            }
        }

        if (candidates.empty())
        {
            xOpt = refControlZ;
            return true;
        }

        // 预计算每个候选约束的最大可达高度，以便在求解过程中快速判断约束是否可行
        auto upperAchievableFor = [&](const CandidateConstraint &candidate)
        {
            double upperAchievable = 0.0;
            for (int j = 0; j < nV; ++j)
            {
                const double a = candidate.row[j];
                upperAchievable += a * ((a >= 0.0) ? ub_eigen[j] : lb_eigen[j]);
            }
            return upperAchievable;
        };

        double worstViolation = 0.0;       // 记录最严重的约束违反程度
        int worstConstraint = -1;          // 记录最严重违反的约束索引
        double worstLowerBound = 0.0;      // 记录最严重违反的约束的下界值
        double worstUpperAchievable = 0.0; // 记录最严重违反的约束的最大可达高度
        for (size_t i = 0; i < candidates.size(); ++i)
        {
            const double upperAchievable = upperAchievableFor(candidates[i]);
            const double violation = candidates[i].lowerBound - upperAchievable;
            if (violation > worstViolation)
            {
                worstViolation = violation;
                worstConstraint = static_cast<int>(i);
                worstLowerBound = candidates[i].lowerBound;
                worstUpperAchievable = upperAchievable;
            }
        }

        if (worstViolation > kConstraintViolationTolerance)
        {
            if (config_.verbose)
            {
                const auto &candidate = candidates[static_cast<size_t>(worstConstraint)];
                std::cerr << "QP infeasible before solve. Worst constraint=" << worstConstraint
                          << ", sampleIndex=" << candidate.sampleIndex
                          << ", type=" << (candidate.type == 0 ? "terrain" : "sphere")
                          << ", lowerBound=" << worstLowerBound
                          << ", upperAchievable=" << worstUpperAchievable
                          << ", violation=" << worstViolation << std::endl;
            }
            return false;
        }

        // 将 Eigen 数据转换为 qpOASES 格式
        std::vector<qpOASES::real_t> H;
        std::vector<qpOASES::real_t> g;
        std::vector<qpOASES::real_t> lb;
        std::vector<qpOASES::real_t> ub;
        H.resize(H_eigen.size());
        g.resize(g_eigen.size());
        lb.resize(lb_eigen.size());
        ub.resize(ub_eigen.size());

        int idx = 0;
        for (int i = 0; i < nV; ++i)
        {
            for (int j = 0; j < nV; ++j)
            {
                H[idx++] = static_cast<qpOASES::real_t>(H_eigen(i, j));
            }
            g[i] = static_cast<qpOASES::real_t>(g_eigen[i]);
            lb[i] = static_cast<qpOASES::real_t>(lb_eigen[i]);
            ub[i] = static_cast<qpOASES::real_t>(ub_eigen[i]);
        }

        std::vector<int> activeSet;
        activeSet.reserve(32);
        std::vector<bool> isActive(candidates.size(), false);
        Eigen::VectorXd currentSolution = projectReferenceToBounds(refControlZ, nV, lb_eigen, ub_eigen);

        int hotspotSampleIndex = -1;
        double hotspotScore = -std::numeric_limits<double>::infinity();
        for (const auto &candidate : candidates)
        {
            const double score = candidate.lowerBound;
            if (score > hotspotScore + kConstraintViolationTolerance ||
                (std::abs(score - hotspotScore) <= kConstraintViolationTolerance &&
                 candidate.type == 1))
            {
                hotspotScore = score;
                hotspotSampleIndex = candidate.sampleIndex;
            }
        }

        auto activateConstraintNeighborhood = [&](int centerSampleIndex, int radius)
        {
            std::vector<int> neighborhood;
            neighborhood.reserve(static_cast<size_t>(2 * std::max(0, radius) + 2));
            for (size_t i = 0; i < candidates.size(); ++i)
            {
                if (isActive[i])
                {
                    continue;
                }

                if (std::abs(candidates[i].sampleIndex - centerSampleIndex) > radius)
                {
                    continue;
                }

                neighborhood.push_back(static_cast<int>(i));
            }

            std::sort(neighborhood.begin(), neighborhood.end(), [&](int lhs, int rhs)
            {
                const auto &left = candidates[static_cast<size_t>(lhs)];
                const auto &right = candidates[static_cast<size_t>(rhs)];
                const int leftDistance = std::abs(left.sampleIndex - centerSampleIndex);
                const int rightDistance = std::abs(right.sampleIndex - centerSampleIndex);
                if (leftDistance != rightDistance)
                {
                    return leftDistance < rightDistance;
                }
                if (left.type != right.type)
                {
                    return left.type > right.type;
                }
                return left.referenceDeficit > right.referenceDeficit;
            });

            for (int candidateIndex : neighborhood)
            {
                activeSet.push_back(candidateIndex);
                isActive[static_cast<size_t>(candidateIndex)] = true;
            }
        };

        auto selectMostViolatedCandidate = [&](const Eigen::VectorXd &curveZ, int frontierRadius)
        {
            int bestIndex = -1;
            double bestViolation = 0.0;
            double bestReferenceDeficit = -1.0;
            int bestDistance = std::numeric_limits<int>::max();
            int bestType = -1;

            for (size_t i = 0; i < candidates.size(); ++i)
            {
                if (isActive[i])
                {
                    continue;
                }

                const auto &candidate = candidates[i];
                const double violation = candidate.lowerBound - curveZ[candidate.sampleIndex];
                if (violation <= kConstraintViolationTolerance)
                {
                    continue;
                }

                const int distanceToHotspot = (hotspotSampleIndex >= 0)
                                                  ? std::abs(candidate.sampleIndex - hotspotSampleIndex)
                                                  : 0;
                if (frontierRadius >= 0 && hotspotSampleIndex >= 0 && distanceToHotspot > frontierRadius)
                {
                    continue;
                }

                const bool betterViolation = violation > bestViolation + kConstraintViolationTolerance;
                const bool equalViolation = std::abs(violation - bestViolation) <= kConstraintViolationTolerance;
                const bool betterDeficit = candidate.referenceDeficit > bestReferenceDeficit + kConstraintViolationTolerance;
                const bool equalDeficit = std::abs(candidate.referenceDeficit - bestReferenceDeficit) <= kConstraintViolationTolerance;
                const bool betterDistance = distanceToHotspot < bestDistance;
                const bool betterType = candidate.type > bestType;

                if (betterViolation ||
                    (equalViolation && (betterDeficit ||
                                        (equalDeficit && (betterDistance ||
                                                          (!betterDistance && distanceToHotspot == bestDistance && betterType))))))
                {
                    bestIndex = static_cast<int>(i);
                    bestViolation = violation;
                    bestReferenceDeficit = candidate.referenceDeficit;
                    bestDistance = distanceToHotspot;
                    bestType = candidate.type;
                }
            }

            return bestIndex;
        };

        int currentFrontierRadius = std::max(0, config_.constraintSeedRadius);
        if (hotspotSampleIndex >= 0)
        {
            activateConstraintNeighborhood(hotspotSampleIndex, currentFrontierRadius);
        }

        for (int outerIter = 0; outerIter < config_.maxActiveSetIterations; ++outerIter)
        {
            if (!solveQp(activeSet, currentSolution, H_eigen, g_eigen, refControlZ, nV, lb_eigen, ub_eigen, candidates))
            {
                return false;
            }

            const Eigen::VectorXd curveZ = basisMat * currentSolution;
            int violatingCandidate = selectMostViolatedCandidate(curveZ, currentFrontierRadius);
            while (violatingCandidate < 0 && hotspotSampleIndex >= 0 && currentFrontierRadius < numSamples)
            {
                currentFrontierRadius = std::min(numSamples,
                                                 currentFrontierRadius + std::max(1, config_.constraintExpandStep));
                violatingCandidate = selectMostViolatedCandidate(curveZ, currentFrontierRadius);
            }

            if (violatingCandidate < 0)
            {
                violatingCandidate = selectMostViolatedCandidate(curveZ, -1);
            }

            if (violatingCandidate < 0)
            {
                xOpt = currentSolution;

                if (config_.verbose)
                {
                    const Eigen::VectorXd optimizedCurveZ = basisMat * currentSolution;
                    const Eigen::VectorXd trackingError = optimizedCurveZ - refCurveZ;

                    double minTerrainMargin = std::numeric_limits<double>::infinity();
                    double minSphereMargin = std::numeric_limits<double>::infinity();
                    double maxTerrainViolation = 0.0;
                    double maxSphereViolation = 0.0;
                    int nearTerrainActiveCount = 0;
                    int nearSphereActiveCount = 0;

                    double minTerrainMarginInside = std::numeric_limits<double>::infinity();
                    double minTerrainMarginOutside = std::numeric_limits<double>::infinity();
                    double minSphereMarginInside = std::numeric_limits<double>::infinity();
                    double minSphereMarginOutside = std::numeric_limits<double>::infinity();
                    double maxTerrainViolationInside = 0.0;
                    double maxTerrainViolationOutside = 0.0;
                    double maxSphereViolationInside = 0.0;
                    double maxSphereViolationOutside = 0.0;
                    int worstTerrainInsideIndex = -1;
                    int worstTerrainOutsideIndex = -1;
                    int worstSphereInsideIndex = -1;
                    int worstSphereOutsideIndex = -1;

                    for (int j = 0; j < numSamples; ++j)
                    {
                        const bool insideConstraintWindow = (j >= firstConstrainedSample && j < lastConstrainedSample);
                        const double terrainMargin = optimizedCurveZ[j] - (terrainHeights[j] + config_.groundClearance);
                        minTerrainMargin = std::min(minTerrainMargin, terrainMargin);
                        maxTerrainViolation = std::max(maxTerrainViolation, -terrainMargin);
                        if (std::abs(terrainMargin) < 1e-4)
                        {
                            ++nearTerrainActiveCount;
                        }

                        if (insideConstraintWindow)
                        {
                            if (terrainMargin < minTerrainMarginInside)
                            {
                                minTerrainMarginInside = terrainMargin;
                            }
                            if (-terrainMargin > maxTerrainViolationInside)
                            {
                                maxTerrainViolationInside = -terrainMargin;
                                worstTerrainInsideIndex = j;
                            }
                        }
                        else
                        {
                            if (terrainMargin < minTerrainMarginOutside)
                            {
                                minTerrainMarginOutside = terrainMargin;
                            }
                            if (-terrainMargin > maxTerrainViolationOutside)
                            {
                                maxTerrainViolationOutside = -terrainMargin;
                                worstTerrainOutsideIndex = j;
                            }
                        }

                        if (std::isfinite(sphereHeights[j]))
                        {
                            const double sphereMargin = optimizedCurveZ[j] - sphereHeights[j];
                            minSphereMargin = std::min(minSphereMargin, sphereMargin);
                            maxSphereViolation = std::max(maxSphereViolation, -sphereMargin);
                            if (std::abs(sphereMargin) < 1e-4)
                            {
                                ++nearSphereActiveCount;
                            }

                            if (insideConstraintWindow)
                            {
                                if (sphereMargin < minSphereMarginInside)
                                {
                                    minSphereMarginInside = sphereMargin;
                                }
                                if (-sphereMargin > maxSphereViolationInside)
                                {
                                    maxSphereViolationInside = -sphereMargin;
                                    worstSphereInsideIndex = j;
                                }
                            }
                            else
                            {
                                if (sphereMargin < minSphereMarginOutside)
                                {
                                    minSphereMarginOutside = sphereMargin;
                                }
                                if (-sphereMargin > maxSphereViolationOutside)
                                {
                                    maxSphereViolationOutside = -sphereMargin;
                                    worstSphereOutsideIndex = j;
                                }
                            }
                        }
                    }

                    const double rmsTrackingError = std::sqrt(trackingError.squaredNorm() / std::max(1, numSamples));
                    const double maxTrackingError = trackingError.cwiseAbs().maxCoeff();
                    const double refMaxZ = refCurveZ.maxCoeff();
                    const double refMinZ = refCurveZ.minCoeff();
                    const double optMaxZ = optimizedCurveZ.maxCoeff();
                    const double optMinZ = optimizedCurveZ.minCoeff();

                }

                return true;
            }

            if (isActive[violatingCandidate])
            {
                if (config_.verbose)
                {
                    std::cerr << "Active-set QP stalled at sample " << candidates[violatingCandidate].sampleIndex
                              << " with remaining violation "
                              << (candidates[violatingCandidate].lowerBound - curveZ[candidates[violatingCandidate].sampleIndex])
                              << std::endl;
                }
                return false;
            }

            activateConstraintNeighborhood(candidates[violatingCandidate].sampleIndex,
                                           std::max(0, config_.constraintBandRadius));
        }

        if (config_.verbose)
        {
            std::cerr << "Active-set QP exceeded outer iterations." << std::endl;
        }
        return false;
    }

    // 优化摆动足轨迹，确保满足地形和球障碍约束，同时尽量跟踪参考轨迹。返回优化后的轨迹样本序列。
    SwingTrajectoryQpOptimizer::OptimizedTrajectory SwingTrajectoryQpOptimizer::optimizeSwingTrajectory(
        const BSpline &refSpline,
        scalar_t swingStartTime,
        scalar_t swingFinalTime,
        const Eigen::Vector3d &liftOffPos3d,
        const Eigen::Vector3d &touchDownPos3d,
        const std::vector<const EdgeSphere *> &nearbySpheres) const
    {

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
            refCurveZ[i] = refSpline.getPosition(sampleParams[i]).z();
        }
        const Eigen::VectorXd terrainHeights = computeTerrainHeightProfile(sampleX, sampleY);
        const Eigen::VectorXd sphereHeights = computeSphereHeightProfile(sampleX, sampleY, nearbySpheres);

        Eigen::VectorXd optimizedControlZ; // 优化后的控制点高度
        const bool qpSolved = optimizeControlPointsZ(basisMat, z, refCurveZ, sampleX, terrainHeights, sphereHeights, optimizedControlZ);

        OptimizedTrajectory result;
        result.startTime = swingStartTime;
        result.endTime = swingFinalTime;

        if (!qpSolved)
        {
            if (config_.verbose)
            {
                std::cerr << "Failed to optimize B-spline control points." << std::endl;
            }
            return result;
        }

        bspline.setControlPoints(x, y, optimizedControlZ);

        const Eigen::VectorXd optimizedCurveZ = basisMat * optimizedControlZ;
        const int ignoredBoundarySamples = std::max(1, config_.ignoreSamples);
        const int firstConstrainedSample = std::min(static_cast<int>(optimizedCurveZ.size()) - 1, ignoredBoundarySamples);
        const int lastConstrainedSample = std::max(firstConstrainedSample,
                                                   static_cast<int>(optimizedCurveZ.size()) - ignoredBoundarySamples);

        result.collisionFree = true;
        for (int i = firstConstrainedSample; i < lastConstrainedSample; ++i)
        {
            const double terrainLowerBound = terrainHeights[i] + config_.groundClearance;
            if (optimizedCurveZ[i] + kConstraintViolationTolerance < terrainLowerBound)
            {
                result.collisionFree = false;
                break;
            }

            if (std::isfinite(sphereHeights[i]) && optimizedCurveZ[i] + kConstraintViolationTolerance < sphereHeights[i])
            {
                result.collisionFree = false;
                break;
            }
        }

        if (!result.collisionFree)
        {
            return result;
        }

        result.bspline = std::make_shared<BSpline>(std::move(bspline));
        return result;
    }

} // namespace ocs2::legged_robot
