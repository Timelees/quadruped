// Test MINVO mode stability with realistic workspace constraints
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

// Test MINVO mode with various workspace scenarios
void testMinvoStability() {
    std::cout << "=== MINVO Mode Stability Test ===\n\n";

    SwingTrajectoryQpConfig config;
    config.numControlPoints = 15;
    config.numCurveSamples = 60;
    config.geometryMode = SwingGeometryMode::Minvo;
    config.subdivisionsPerSpan = 4;
    config.groundClearance = 0.03;
    config.footRadius = 0.019;
    config.hessianRegularization = 1e-4;
    config.verbose = false;
    config.checkReference = false;
    config.verifyOutput = false;

    auto terrain = std::make_shared<FlatTerrain>();

    // Reference spline: typical swing trajectory
    BSpline refSpline(5);
    const int nRef = 7;
    Eigen::VectorXd z(nRef);
    z << 0.0, 0.002, 0.07, 0.13, 0.07, 0.002, 0.0;
    refSpline.setControlPoints(Eigen::VectorXd::Zero(nRef), Eigen::VectorXd::Zero(nRef), z);
    refSpline.setKnots(refSpline.generateKnots(nRef, 0.0, 0.3));

    const Eigen::Vector3d liftOff(0.1, 0.0, 0.0);
    const Eigen::Vector3d touchDown(0.2, 0.0, 0.0);

    int passed = 0, failed = 0;

    // Test 1: Wide workspace (IK failure fallback)
    {
        auto workspace = SwingTrajectoryQpOptimizer::HeightBounds(
            [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
                return {-1.0, 1.0};
            });
        auto result = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
            refSpline, 0.0, 0.3, liftOff, touchDown, workspace);
        if (result.solved) {
            std::cout << "[PASS] Wide workspace {-1, 1}\n";
            ++passed;
        } else {
            std::cout << "[FAIL] Wide workspace {-1, 1}\n";
            ++failed;
        }
    }

    // Test 2: Moderate workspace containing reference
    {
        auto workspace = SwingTrajectoryQpOptimizer::HeightBounds(
            [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
                return {-0.05, 0.2};
            });
        auto result = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
            refSpline, 0.0, 0.3, liftOff, touchDown, workspace);
        if (result.solved) {
            std::cout << "[PASS] Moderate workspace {-0.05, 0.2}\n";
            ++passed;
        } else {
            std::cout << "[FAIL] Moderate workspace {-0.05, 0.2}\n";
            ++failed;
        }
    }

    // Test 3: Tight workspace at endpoints (relaxed by endpoint blend)
    {
        auto workspace = SwingTrajectoryQpOptimizer::HeightBounds(
            [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
                return {0.05, 0.15};  // Does not contain z=0 at endpoints
            });
        auto result = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
            refSpline, 0.0, 0.3, liftOff, touchDown, workspace);
        if (result.solved) {
            std::cout << "[PASS] Tight workspace {0.05, 0.15} (endpoint relaxed)\n";
            ++passed;
        } else {
            std::cout << "[FAIL] Tight workspace {0.05, 0.15} (endpoint relaxed)\n";
            ++failed;
        }
    }

    // Test 4: No workspace (baseline)
    {
        auto result = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
            refSpline, 0.0, 0.3, liftOff, touchDown, {});
        if (result.solved) {
            std::cout << "[PASS] No workspace constraints\n";
            ++passed;
        } else {
            std::cout << "[FAIL] No workspace constraints\n";
            ++failed;
        }
    }

    // Test 5: Very short swing (fast trot)
    {
        BSpline shortSpline(5);
        shortSpline.setControlPoints(Eigen::VectorXd::Zero(nRef), Eigen::VectorXd::Zero(nRef), z);
        shortSpline.setKnots(shortSpline.generateKnots(nRef, 0.0, 0.1));  // 100ms swing

        auto workspace = SwingTrajectoryQpOptimizer::HeightBounds(
            [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
                return {-0.5, 0.5};
            });
        auto result = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
            shortSpline, 0.0, 0.1, liftOff, touchDown, workspace);
        if (result.solved) {
            std::cout << "[PASS] Short swing duration (0.1s)\n";
            ++passed;
        } else {
            std::cout << "[FAIL] Short swing duration (0.1s)\n";
            ++failed;
        }
    }

    std::cout << "\n=== Summary: " << passed << " passed, " << failed << " failed ===\n";
}

int main() {
    try {
        testMinvoStability();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }
}
