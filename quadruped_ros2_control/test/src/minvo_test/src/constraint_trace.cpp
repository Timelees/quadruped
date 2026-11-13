// Diagnostic: trace through the QP constraint generation step by step
#include <cmath>
#include <iostream>
#include <memory>
#include <ocs2_quadruped_controller/interface/constraint/SwingTrajectoryQPOptimizer.h>
#include <ocs2_quadruped_controller/interface/constraint/PolynomialHull.h>

using namespace ocs2::legged_robot;

class FlatTerrain final : public TerrainInterface {
public:
    double getHeight(double, double) const override { return 0.0; }
    std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d&, const Eigen::Vector2d&) const override {
        return {{{-5, -5}, {5, 5}, 0}};
    }
};

void traceConstraints() {
    std::cout << "\n=== Constraint Trace ===\n";

    SwingTrajectoryQpConfig config;
    config.numControlPoints = 15;
    config.numCurveSamples = 60;
    config.geometryMode = SwingGeometryMode::Minvo;
    config.subdivisionsPerSpan = 4;
    config.groundClearance = 0.03;
    config.footRadius = 0.019;
    config.hessianRegularization = 1e-8;
    config.verbose = true;
    config.checkReference = false;
    config.verifyOutput = false;

    auto terrain = std::make_shared<FlatTerrain>();

    BSpline refSpline(5);
    const int nRef = 7;
    Eigen::VectorXd z(nRef);
    z << 0.0, 0.002, 0.07, 0.13, 0.07, 0.002, 0.0;
    refSpline.setControlPoints(Eigen::VectorXd::Zero(nRef), Eigen::VectorXd::Zero(nRef), z);
    refSpline.setKnots(refSpline.generateKnots(nRef, 0.0, 0.3));

    const Eigen::Vector3d liftOff(0.1, 0.0, 0.0);
    const Eigen::Vector3d touchDown(0.2, 0.0, 0.0);

    // First, manually construct the fitted curve to see its control points
    std::cout << "Constructing fitted curve...\n";
    const int n = config.numControlPoints;

    // Replicate sampleReferenceTrajectoryToControlPoints logic
    BSpline referenceFitSpline(config.splineOrder);
    referenceFitSpline.setControlPoints(Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n),
                                        Eigen::VectorXd::Zero(n));
    referenceFitSpline.setKnots(referenceFitSpline.generateKnots(n, 0.0, 1.0, 0));

    const int fitSampleCount = std::max(config.numCurveSamples, 8 * config.numControlPoints);
    const Eigen::VectorXd fitParams = Eigen::VectorXd::LinSpaced(fitSampleCount, 0.0, 1.0);
    const Eigen::MatrixXd fitBasis = referenceFitSpline.generateSplineBasis(fitParams);
    Eigen::VectorXd referenceCurveZ(fitSampleCount);
    for (int i = 0; i < fitSampleCount; ++i) {
        const double t = 0.0 + fitParams[i] * (0.3 - 0.0);
        referenceCurveZ[i] = refSpline.getPosition(t).z();
    }
    std::cout << "Reference curve z range: [" << referenceCurveZ.minCoeff() << ", " << referenceCurveZ.maxCoeff() << "]\n";

    // Boundary control points
    const Eigen::VectorXd knots = referenceFitSpline.generateKnots(n, 0.0, 1.0, 0);
    const int degree = config.splineOrder;
    const double duration = 0.3;
    Eigen::VectorXd boundaryControlZ = Eigen::VectorXd::Zero(n);
    boundaryControlZ[0] = liftOff.z();
    boundaryControlZ[1] = liftOff.z() + refSpline.getVelocity(0.0).z() * duration *
                                                 (knots[degree + 1] - knots[degree]) / degree;
    boundaryControlZ[n - 1] = touchDown.z();
    boundaryControlZ[n - 2] = touchDown.z() - refSpline.getVelocity(0.3).z() * duration *
                                                       (knots[n] - knots[n - 1]) / degree;
    std::cout << "Boundary control points:\n";
    std::cout << "  z[0] = " << boundaryControlZ[0] << " (liftOff)\n";
    std::cout << "  z[1] = " << boundaryControlZ[1] << " (velocity)\n";
    std::cout << "  z[n-2] = " << boundaryControlZ[n - 2] << " (velocity)\n";
    std::cout << "  z[n-1] = " << boundaryControlZ[n - 1] << " (touchDown)\n";

    // Now run the optimizer to get the fitted control points
    SwingTrajectoryQpOptimizer optimizer(config, terrain);
    auto wideWorkspace = SwingTrajectoryQpOptimizer::HeightBounds(
        [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
            return {-10.0, 10.0};
        });
    auto result = optimizer.optimizeSwingTrajectory(refSpline, 0.0, 0.3, liftOff, touchDown, wideWorkspace);

    if (result.solved && result.bspline) {
        const auto cp = result.bspline->getControlPoints();
        std::cout << "\nFitted control points z:\n";
        for (int i = 0; i < cp.rows(); ++i) {
            std::cout << "  z[" << i << "] = " << cp(i, 2) << "\n";
        }

        // Check curve z values at control point phases
        // The returned bspline has knots in [start, end] = [0, 0.3], not [0, 1]
        std::cout << "\nCurve z at control point phases:\n";
        for (int k = 0; k < n; ++k) {
            double phase = double(k) / (n - 1);
            double t = 0.3 * phase;  // Map phase to time domain
            double curveZ = result.bspline->getPosition(t).z();
            std::cout << "  phase=" << phase << " (t=" << t << "): z=" << curveZ << "\n";
        }
    }

    // Now test with the failing workspace
    std::cout << "\n=== Testing with failing workspace {0.05, 0.15} ===\n";
    auto failingWorkspace = SwingTrajectoryQpOptimizer::HeightBounds(
        [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
            return {0.05, 0.15};
        });

    // Print what workspace bounds would be at each control point phase
    std::cout << "Workspace bounds at control point phases:\n";
    for (int k = 0; k < n; ++k) {
        double phase = double(k) / (n - 1);
        auto bounds = failingWorkspace(0.0 + 0.3 * phase, Eigen::Vector3d(0.15, 0.0, 0.0));
        std::cout << "  k=" << k << ", phase=" << phase << ": [" << bounds[0] << ", " << bounds[1] << "]\n";
    }

    auto result2 = optimizer.optimizeSwingTrajectory(refSpline, 0.0, 0.3, liftOff, touchDown, failingWorkspace);
    std::cout << "Result: " << (result2.solved ? "OK" : "FAIL") << "\n";
}

int main() {
    try {
        traceConstraints();
        std::cout << "\n=== Done ===\n";
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
