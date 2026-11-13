// Test to verify the reference trajectory protection fix
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

void testReferenceProtection() {
    std::cout << "=== Reference Trajectory Protection Test ===\n\n";

    SwingTrajectoryQpConfig config;
    config.numControlPoints = 15;
    config.numCurveSamples = 60;
    config.geometryMode = SwingGeometryMode::Minvo;
    config.subdivisionsPerSpan = 4;
    config.groundClearance = 0.03;
    config.footRadius = 0.019;
    config.hessianRegularization = 1e-4;
    config.verbose = true;  // Enable verbose to see the protection in action
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

    // Scenario: Workspace that explicitly excludes reference at endpoints
    std::cout << "Test 1: Workspace excludes reference at endpoints\n";
    std::cout << "Workspace: [0.05, 0.5] (excludes z=0 at endpoints)\n\n";

    auto workspace = SwingTrajectoryQpOptimizer::HeightBounds(
        [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
            return {0.05, 0.5};  // Lower bound 0.05 > reference z=0
        });

    auto result = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
        refSpline, 0.0, 0.3, liftOff, touchDown, workspace);

    std::cout << "\nResult: solved=" << result.solved
              << ", cost=" << result.correctionCost << "\n";

    if (result.bspline) {
        std::cout << "\nTrajectory comparison:\n";
        std::cout << "phase | reference | optimized | diff\n";
        for (int i = 0; i <= 10; ++i) {
            double phase = i / 10.0;
            double t = 0.3 * phase;
            double refZ = refSpline.getPosition(t).z();
            double optZ = result.bspline->getPosition(t).z();
            double diff = std::abs(optZ - refZ);
            printf("%.2f  |  %.4f   |  %.4f   | %.6f\n", phase, refZ, optZ, diff);
        }

        double maxDiff = 0;
        for (int i = 0; i <= 100; ++i) {
            double t = 0.3 * i / 100.0;
            double diff = std::abs(result.bspline->getPosition(t).z() - refSpline.getPosition(t).z());
            maxDiff = std::max(maxDiff, diff);
        }
        std::cout << "\nMax deviation from reference: " << maxDiff << "m\n";

        if (maxDiff < 0.01) {
            std::cout << "✓ Reference trajectory preserved (deviation < 10mm)\n";
        } else {
            std::cout << "✗ Reference trajectory significantly modified (deviation >= 10mm)\n";
        }
    }
}

int main() {
    try {
        testReferenceProtection();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
