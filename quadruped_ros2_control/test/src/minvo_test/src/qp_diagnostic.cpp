// Diagnose the QP numerical instability at trot startup.
// This reproduces the scenario: useTerrainPlanning=true + MINVO mode,
// first swing phase of a trot gait.
#include <cmath>
#include <iostream>
#include <memory>
#include <Eigen/Eigenvalues>
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

// Create a reference spline mimicking the first swing of a trot gait
BSpline createTrotSwingReference(double swingDuration) {
    BSpline spline(5);
    const int n = 7;
    Eigen::VectorXd z(n);
    // Typical swing heights: start at 0, peak at ~0.085, end at 0
    // With small velocity-induced offsets at control points 1 and 5
    const double endpointOffset = 0.04 * swingDuration / 12.0;
    z << 0.0, endpointOffset, 0.07, 0.13, 0.07, endpointOffset, 0.0;
    spline.setControlPoints(Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n), z);
    spline.setKnots(spline.generateKnots(n, 0.0, 1.0));
    return spline;
}

void diagnoseHessianCondition() {
    std::cout << "\n=== Hessian Condition Analysis ===\n";

    SwingTrajectoryQpConfig config;
    config.numControlPoints = 15;
    config.numCurveSamples = 60;
    config.geometryMode = SwingGeometryMode::Minvo;
    config.subdivisionsPerSpan = 4;
    config.groundClearance = 0.03;
    config.footRadius = 0.019;
    config.hessianRegularization = 1e-8;  // Default value
    config.verbose = true;
    config.checkReference = false;
    config.verifyOutput = false;

    auto terrain = std::make_shared<FlatTerrain>();
    SwingTrajectoryQpOptimizer optimizer(config, terrain);

    // Create a reference spline
    const double swingDuration = 0.3;  // Typical trot swing duration
    BSpline refSpline = createTrotSwingReference(swingDuration);

    const Eigen::Vector3d liftOff(0.1, 0.0, 0.0);
    const Eigen::Vector3d touchDown(0.2, 0.0, 0.0);

    // First: no workspace constraints
    std::cout << "\n--- Without workspace constraints ---\n";
    auto result1 = optimizer.optimizeSwingTrajectory(refSpline, 0.0, swingDuration, liftOff, touchDown, {});
    std::cout << "Solved: " << (result1.solved ? "YES" : "NO") << "\n";
    std::cout << "Working set iterations: " << result1.workingSetIterations << "\n";
    std::cout << "Correction cost: " << result1.correctionCost << "\n";
    std::cout << "Constraint rows: " << result1.constraintRows << "\n";
}

void diagnoseWithWorkspace() {
    std::cout << "\n=== With Workspace Constraints ===\n";

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
    SwingTrajectoryQpOptimizer optimizer(config, terrain);

    const double swingDuration = 0.3;
    BSpline refSpline = createTrotSwingReference(swingDuration);

    const Eigen::Vector3d liftOff(0.1, 0.0, 0.0);
    const Eigen::Vector3d touchDown(0.2, 0.0, 0.0);

    // Wide workspace (what IK failure returns)
    auto wideWorkspace = SwingTrajectoryQpOptimizer::HeightBounds(
        [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
            return {-1.0, 1.0};
        });

    std::cout << "\n--- With wide workspace [-1, 1] ---\n";
    auto result = optimizer.optimizeSwingTrajectory(refSpline, 0.0, swingDuration, liftOff, touchDown, wideWorkspace);
    std::cout << "Solved: " << (result.solved ? "YES" : "NO") << "\n";
    std::cout << "Working set iterations: " << result.workingSetIterations << "\n";
    std::cout << "Correction cost: " << result.correctionCost << "\n";
    std::cout << "Constraint rows: " << result.constraintRows << "\n";
}

void diagnoseDifferentRegularizations() {
    std::cout << "\n=== Hessian Regularization Sweep ===\n";

    const double swingDuration = 0.3;
    BSpline refSpline = createTrotSwingReference(swingDuration);
    const Eigen::Vector3d liftOff(0.1, 0.0, 0.0);
    const Eigen::Vector3d touchDown(0.2, 0.0, 0.0);

    auto wideWorkspace = SwingTrajectoryQpOptimizer::HeightBounds(
        [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
            return {-1.0, 1.0};
        });

    const double regularizations[] = {1e-10, 1e-8, 1e-6, 1e-4, 1e-2, 1e-1};

    for (double reg : regularizations) {
        SwingTrajectoryQpConfig config;
        config.numControlPoints = 15;
        config.numCurveSamples = 60;
        config.geometryMode = SwingGeometryMode::Minvo;
        config.subdivisionsPerSpan = 4;
        config.groundClearance = 0.03;
        config.footRadius = 0.019;
        config.hessianRegularization = reg;
        config.verbose = false;
        config.checkReference = false;
        config.verifyOutput = false;

        auto terrain = std::make_shared<FlatTerrain>();
        SwingTrajectoryQpOptimizer optimizer(config, terrain);

        auto result = optimizer.optimizeSwingTrajectory(refSpline, 0.0, swingDuration, liftOff, touchDown, wideWorkspace);
        std::cout << "reg=" << reg << ": solved=" << (result.solved ? "YES" : "NO")
                  << ", iters=" << result.workingSetIterations
                  << ", cost=" << result.correctionCost << "\n";
    }
}

void diagnoseGeometryModes() {
    std::cout << "\n=== Geometry Mode Comparison ===\n";

    const double swingDuration = 0.3;
    BSpline refSpline = createTrotSwingReference(swingDuration);
    const Eigen::Vector3d liftOff(0.1, 0.0, 0.0);
    const Eigen::Vector3d touchDown(0.2, 0.0, 0.0);

    auto wideWorkspace = SwingTrajectoryQpOptimizer::HeightBounds(
        [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
            return {-1.0, 1.0};
        });

    const SwingGeometryMode modes[] = {
        SwingGeometryMode::Discrete,
        SwingGeometryMode::Bezier,
        SwingGeometryMode::Minvo
    };
    const char* modeNames[] = {"Discrete", "Bezier", "Minvo"};

    for (size_t i = 0; i < 3; ++i) {
        SwingTrajectoryQpConfig config;
        config.numControlPoints = 15;
        config.numCurveSamples = 60;
        config.geometryMode = modes[i];
        config.subdivisionsPerSpan = 4;
        config.groundClearance = 0.03;
        config.footRadius = 0.019;
        config.hessianRegularization = 1e-8;
        config.verbose = false;
        config.checkReference = false;
        config.verifyOutput = false;

        auto terrain = std::make_shared<FlatTerrain>();
        SwingTrajectoryQpOptimizer optimizer(config, terrain);

        auto result = optimizer.optimizeSwingTrajectory(refSpline, 0.0, swingDuration, liftOff, touchDown, wideWorkspace);
        std::cout << modeNames[i] << ": solved=" << (result.solved ? "YES" : "NO")
                  << ", iters=" << result.workingSetIterations
                  << ", cost=" << result.correctionCost
                  << ", rows=" << result.constraintRows << "\n";
    }
}

void diagnoseMinvoBasisNumbers() {
    std::cout << "\n=== MINVO Basis Matrix Analysis ===\n";

    // Check the MINVO basis matrix condition
    const auto& minvo = minvoBasisMatrix();
    std::cout << "MINVO basis matrix:\n" << minvo << "\n\n";

    Eigen::EigenSolver<Eigen::MatrixXd> eig(minvo);
    auto eigenvalues = eig.eigenvalues();
    double minAbs = std::numeric_limits<double>::max();
    double maxAbs = 0;
    for (int i = 0; i < eigenvalues.size(); ++i) {
        double abs = std::abs(eigenvalues[i]);
        minAbs = std::min(minAbs, abs);
        maxAbs = std::max(maxAbs, abs);
    }
    std::cout << "Eigenvalue magnitudes: [";
    for (int i = 0; i < eigenvalues.size(); ++i) {
        std::cout << std::abs(eigenvalues[i]);
        if (i < eigenvalues.size() - 1) std::cout << ", ";
    }
    std::cout << "]\n";
    std::cout << "Condition number (approx): " << maxAbs / minAbs << "\n";

    // Compare with Bezier basis (identity for degree 5)
    std::cout << "\nBezier basis is identity (condition = 1.0)\n";
}

int main() {
    try {
        diagnoseHessianCondition();
        diagnoseWithWorkspace();
        diagnoseDifferentRegularizations();
        diagnoseGeometryModes();
        diagnoseMinvoBasisNumbers();
        std::cout << "\n=== Diagnostics complete ===\n";
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
