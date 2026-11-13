// Simulate the real scenario: what does the workspace return during flat terrain walking?
// This reproduces the PerceptiveLeggedReferenceManager's workspace callback behavior
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

// Simulate what the real IK-based workspace returns at different body poses
// During trot startup, the body might be at slightly different heights
void simulateRealWorkspaceBehavior() {
    std::cout << "=== Real Workspace Behavior Simulation ===\n\n";

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

    // Reference spline
    BSpline refSpline(5);
    const int nRef = 7;
    Eigen::VectorXd z(nRef);
    z << 0.0, 0.002, 0.07, 0.13, 0.07, 0.002, 0.0;
    refSpline.setControlPoints(Eigen::VectorXd::Zero(nRef), Eigen::VectorXd::Zero(nRef), z);
    refSpline.setKnots(refSpline.generateKnots(nRef, 0.0, 0.3));

    const Eigen::Vector3d liftOff(0.1, 0.0, 0.0);
    const Eigen::Vector3d touchDown(0.2, 0.0, 0.0);

    // Scenario 1: Workspace returns tight bounds (robot crouched low)
    std::cout << "--- Scenario 1: Tight workspace (body crouched) ---\n";
    std::cout << "Workspace: {0.0, 0.05} at all phases\n";
    {
        auto workspace = SwingTrajectoryQpOptimizer::HeightBounds(
            [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
                return {0.0, 0.05};  // Can only reach 5cm high
            });
        auto result = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
            refSpline, 0.0, 0.3, liftOff, touchDown, workspace);
        std::cout << "Solved: " << result.solved << ", cost: " << result.correctionCost << "\n";
        if (result.bspline) {
            std::cout << "Optimized z at phases 0.0, 0.25, 0.5, 0.75, 1.0:\n  ";
            for (double p : {0.0, 0.25, 0.5, 0.75, 1.0}) {
                printf("%.4f ", result.bspline->getPosition(0.3 * p).z());
            }
            std::cout << "\n\n";
        }
    }

    // Scenario 2: Workspace varies along swing path (asymmetric)
    std::cout << "--- Scenario 2: Varying workspace along path ---\n";
    std::cout << "Workspace: tighter at start, wider at middle\n";
    {
        auto workspace = SwingTrajectoryQpOptimizer::HeightBounds(
            [](double t, const Eigen::Vector3d& pos) -> Eigen::Vector2d {
                // Varies with position x
                double x = pos.x();
                if (x < 0.12) return {0.0, 0.03};  // Tight near liftoff
                if (x < 0.18) return {-0.05, 0.15}; // Wide in middle
                return {-0.02, 0.08};               // Medium near touchdown
            });
        auto result = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
            refSpline, 0.0, 0.3, liftOff, touchDown, workspace);
        std::cout << "Solved: " << result.solved << ", cost: " << result.correctionCost << "\n";
        if (result.bspline) {
            std::cout << "Optimized z at phases 0.0, 0.25, 0.5, 0.75, 1.0:\n  ";
            for (double p : {0.0, 0.25, 0.5, 0.75, 1.0}) {
                printf("%.4f ", result.bspline->getPosition(0.3 * p).z());
            }
            std::cout << "\n\n";
        }
    }

    // Scenario 3: Workspace with discontinuity (IK solution jumps)
    std::cout << "--- Scenario 3: Workspace with sudden change ---\n";
    std::cout << "Workspace: wide for first half, tight for second half\n";
    {
        auto workspace = SwingTrajectoryQpOptimizer::HeightBounds(
            [](double, const Eigen::Vector3d& pos) -> Eigen::Vector2d {
                double x = pos.x();
                if (x < 0.15) return {-0.1, 0.3};
                return {0.01, 0.05};  // Very tight for second half
            });
        auto result = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
            refSpline, 0.0, 0.3, liftOff, touchDown, workspace);
        std::cout << "Solved: " << result.solved << ", cost: " << result.correctionCost << "\n";
        if (result.bspline) {
            std::cout << "Optimized z at 10 points:\n  ";
            for (int i = 0; i <= 10; ++i) {
                printf("%.4f ", result.bspline->getPosition(0.3 * i / 10.0).z());
            }
            std::cout << "\n\n";
        }
    }

    // Scenario 4: The key question - what causes "乱" (chaotic) behavior?
    // Hypothesis: workspace bounds change every MPC cycle as body state changes
    std::cout << "--- Scenario 4: MPC cycle variation simulation ---\n";
    std::cout << "Simulating 5 MPC cycles with slightly different workspace bounds:\n";
    for (int cycle = 0; cycle < 5; ++cycle) {
        double bodyHeight = 0.28 + 0.01 * (cycle - 2);  // Body oscillates ±1cm
        double maxReach = bodyHeight - 0.15;  // Leg can reach body_height - 0.15 below body
        double minReach = -bodyHeight + 0.1;  // Ground limit

        auto workspace = SwingTrajectoryQpOptimizer::HeightBounds(
            [minReach, maxReach](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
                return {minReach, maxReach};
            });
        auto result = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
            refSpline, 0.0, 0.3, liftOff, touchDown, workspace);
        std::cout << "Cycle " << cycle << " (body=" << bodyHeight << ", ws=[" << minReach << "," << maxReach << "]): ";
        if (result.bspline) {
            double maxZ = 0;
            for (int i = 0; i <= 10; ++i) {
                maxZ = std::max(maxZ, result.bspline->getPosition(0.3 * i / 10.0).z());
            }
            printf("peak=%.4f, cost=%.6f\n", maxZ, result.correctionCost);
        } else {
            std::cout << "FAILED\n";
        }
    }

    // Scenario 5: Per-position workspace that depends on XY
    std::cout << "\n--- Scenario 5: XY-dependent workspace (realistic IK) ---\n";
    {
        auto workspace = SwingTrajectoryQpOptimizer::HeightBounds(
            [](double, const Eigen::Vector3d& pos) -> Eigen::Vector2d {
                // Simulating IK limits: as foot moves forward, less Z range
                double x = pos.x();
                double zMax = 0.15 - 0.5 * std::abs(x - 0.15);  // Peak at x=0.15
                double zMin = -0.02 - 0.3 * std::abs(x - 0.15);
                return {zMin, zMax};
            });
        auto result = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
            refSpline, 0.0, 0.3, liftOff, touchDown, workspace);
        std::cout << "Solved: " << result.solved << ", cost: " << result.correctionCost << "\n";
        if (result.bspline) {
            std::cout << "Reference z:    ";
            for (int i = 0; i <= 10; ++i) printf("%.4f ", refSpline.getPosition(0.3 * i / 10.0).z());
            std::cout << "\nOptimized z:    ";
            for (int i = 0; i <= 10; ++i) printf("%.4f ", result.bspline->getPosition(0.3 * i / 10.0).z());
            std::cout << "\n\n";
        }
    }
}

int main() {
    try {
        simulateRealWorkspaceBehavior();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
