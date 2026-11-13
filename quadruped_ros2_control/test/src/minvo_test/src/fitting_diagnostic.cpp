// Diagnostic: check if the reference fitting preserves curve heights
#include <cmath>
#include <iostream>
#include <memory>
#include <ocs2_quadruped_controller/interface/constraint/SwingTrajectoryQPOptimizer.h>

using namespace ocs2::legged_robot;

class FlatTerrain final : public TerrainInterface {
public:
    double getHeight(double, double) const override { return 0.0; }
    std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d&, const Eigen::Vector2d&) const override {
        return {{{-5, -5}, {5, 5}, 0}};
    }
};

void checkReferenceFitting() {
    std::cout << "\n=== Reference Fitting Check ===\n";

    // Create a reference spline mimicking the trot startup scenario
    BSpline refSpline(5);
    const int nRef = 7;
    Eigen::VectorXd z(nRef);
    z << 0.0, 0.002, 0.07, 0.13, 0.07, 0.002, 0.0;
    refSpline.setControlPoints(Eigen::VectorXd::Zero(nRef), Eigen::VectorXd::Zero(nRef), z);
    refSpline.setKnots(refSpline.generateKnots(nRef, 0.0, 0.3));

    std::cout << "Original reference spline:\n";
    std::cout << "  Control points z: " << z.transpose() << "\n";
    // The spline uses time domain [0, 0.3], query at proper times
    std::cout << "  z at t=0.0: " << refSpline.getPosition(0.0).z() << "\n";
    std::cout << "  z at t=0.075: " << refSpline.getPosition(0.075).z() << "\n";
    std::cout << "  z at t=0.15: " << refSpline.getPosition(0.15).z() << "\n";
    std::cout << "  z at t=0.225: " << refSpline.getPosition(0.225).z() << "\n";
    std::cout << "  z at t=0.3: " << refSpline.getPosition(0.3).z() << "\n";
    std::cout << "  Max z (sampled): ";
    double maxZ = 0;
    for (int i = 0; i <= 100; ++i) {
        maxZ = std::max(maxZ, refSpline.getPosition(0.3 * i / 100.0).z());
    }
    std::cout << maxZ << "\n";

    // Now use the QP optimizer's sampleReferenceTrajectoryToControlPoints
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

    const Eigen::Vector3d liftOff(0.1, 0.0, 0.0);
    const Eigen::Vector3d touchDown(0.2, 0.0, 0.0);

    // Call optimizeSwingTrajectory to see the fitted control points
    // We'll extract them by checking what the QP uses as reference
    std::cout << "\nAfter fitting to " << config.numControlPoints << " control points:\n";

    // Use a very wide workspace to ensure QP succeeds and we can see the reference
    auto wideWorkspace = SwingTrajectoryQpOptimizer::HeightBounds(
        [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
            return {-10.0, 10.0};
        });

    auto result = optimizer.optimizeSwingTrajectory(refSpline, 0.0, 0.3, liftOff, touchDown, wideWorkspace);

    if (result.solved && result.bspline) {
        const auto& fittedSpline = *result.bspline;
        const auto controlPoints = fittedSpline.getControlPoints();
        std::cout << "  Fitted control points z:\n    ";
        for (int i = 0; i < controlPoints.rows(); ++i) {
            std::cout << controlPoints(i, 2) << " ";
        }
        std::cout << "\n";

        std::cout << "  Fitted z at t=0.0: " << fittedSpline.getPosition(0.0).z() << "\n";
        std::cout << "  Fitted z at t=0.075: " << fittedSpline.getPosition(0.075).z() << "\n";
        std::cout << "  Fitted z at t=0.15: " << fittedSpline.getPosition(0.15).z() << "\n";
        std::cout << "  Fitted z at t=0.225: " << fittedSpline.getPosition(0.225).z() << "\n";
        std::cout << "  Fitted z at t=0.3: " << fittedSpline.getPosition(0.3).z() << "\n";

        std::cout << "  Max fitted z (sampled): ";
        double maxFittedZ = 0;
        for (int i = 0; i <= 100; ++i) {
            maxFittedZ = std::max(maxFittedZ, fittedSpline.getPosition(0.3 * i / 100.0).z());
        }
        std::cout << maxFittedZ << "\n";
    } else {
        std::cout << "  QP failed to solve!\n";
    }
}

void checkConstraintValues() {
    std::cout << "\n=== Constraint Value Check ===\n";

    SwingTrajectoryQpConfig config;
    config.numControlPoints = 15;
    config.numCurveSamples = 60;
    config.geometryMode = SwingGeometryMode::Minvo;
    config.subdivisionsPerSpan = 4;
    config.groundClearance = 0.03;
    config.footRadius = 0.019;
    config.hessianRegularization = 1e-8;
    config.verbose = false;
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

    // Workspace that should be compatible
    auto workspace = SwingTrajectoryQpOptimizer::HeightBounds(
        [](double, const Eigen::Vector3d& pos) -> Eigen::Vector2d {
            // Return bounds based on position
            std::cout << "  workspace queried at z=" << pos.z() << "\n";
            return {0.0, 0.2};  // Wide enough to contain reference
        });

    std::cout << "Workspace queries during optimization:\n";
    auto result = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
        refSpline, 0.0, 0.3, liftOff, touchDown, workspace);

    std::cout << "Result: " << (result.solved ? "OK" : "FAIL") << "\n";
}

int main() {
    try {
        checkReferenceFitting();
        checkConstraintValues();
        std::cout << "\n=== Diagnostics complete ===\n";
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
