// Focused diagnostic: workspace vs terrain constraint conflict
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

void testWorkspaceTerrainConflict() {
    std::cout << "\n=== Workspace vs Terrain Conflict Test ===\n";

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

    // Reference spline with typical swing heights
    BSpline refSpline(5);
    const int n = 7;
    Eigen::VectorXd z(n);
    z << 0.0, 0.002, 0.07, 0.13, 0.07, 0.002, 0.0;
    refSpline.setControlPoints(Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n), z);
    refSpline.setKnots(refSpline.generateKnots(n, 0.0, 0.3));

    const Eigen::Vector3d liftOff(0.1, 0.0, 0.0);
    const Eigen::Vector3d touchDown(0.2, 0.0, 0.0);

    // Case 1: Tight workspace that conflicts with swing height
    std::cout << "\n--- Case 1: Tight workspace {0.0, 0.01} conflicting with swing height ~0.07-0.13 ---\n";
    auto tightConflicting = SwingTrajectoryQpOptimizer::HeightBounds(
        [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
            return {0.0, 0.01};  // Require z in [0, 0.01], but reference has z up to 0.13
        });

    auto result1 = optimizer.optimizeSwingTrajectory(refSpline, 0.0, 0.3, liftOff, touchDown, tightConflicting);
    std::cout << "Solved: " << (result1.solved ? "YES" : "NO") << "\n";
    std::cout << "Iterations: " << result1.workingSetIterations << "\n";

    // Case 2: Tight workspace that is compatible with swing height
    std::cout << "\n--- Case 2: Tight workspace {0.05, 0.15} compatible with swing height ---\n";
    auto tightCompatible = SwingTrajectoryQpOptimizer::HeightBounds(
        [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
            return {0.05, 0.15};  // Reference z=0.07-0.13 is within this
        });

    auto result2 = optimizer.optimizeSwingTrajectory(refSpline, 0.0, 0.3, liftOff, touchDown, tightCompatible);
    std::cout << "Solved: " << (result2.solved ? "YES" : "NO") << "\n";
    std::cout << "Iterations: " << result2.workingSetIterations << "\n";

    // Case 3: Wide workspace (what IK failure returns)
    std::cout << "\n--- Case 3: Wide workspace {-1, 1} from IK failure fallback ---\n";
    auto wideDefault = SwingTrajectoryQpOptimizer::HeightBounds(
        [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
            return {-1.0, 1.0};
        });

    auto result3 = optimizer.optimizeSwingTrajectory(refSpline, 0.0, 0.3, liftOff, touchDown, wideDefault);
    std::cout << "Solved: " << (result3.solved ? "YES" : "NO") << "\n";
    std::cout << "Iterations: " << result3.workingSetIterations << "\n";

    // Case 4: No workspace at all (baseline)
    std::cout << "\n--- Case 4: No workspace constraints (baseline) ---\n";
    auto result4 = optimizer.optimizeSwingTrajectory(refSpline, 0.0, 0.3, liftOff, touchDown, {});
    std::cout << "Solved: " << (result4.solved ? "YES" : "NO") << "\n";
    std::cout << "Iterations: " << result4.workingSetIterations << "\n";
}

void testWorkspaceScaling() {
    std::cout << "\n=== Workspace Bounds Scaling Test ===\n";

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
    const int n = 7;
    Eigen::VectorXd z(n);
    z << 0.0, 0.002, 0.07, 0.13, 0.07, 0.002, 0.0;
    refSpline.setControlPoints(Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n), z);
    refSpline.setKnots(refSpline.generateKnots(n, 0.0, 0.3));

    const Eigen::Vector3d liftOff(0.1, 0.0, 0.0);
    const Eigen::Vector3d touchDown(0.2, 0.0, 0.0);

    // Test different workspace widths
    const double widths[] = {0.01, 0.05, 0.1, 0.2, 0.5, 1.0, 2.0};
    const double centers[] = {0.07, 0.0};  // Center around swing midpoint vs ground

    for (double center : centers) {
        std::cout << "\n--- Workspace centered at z=" << center << " ---\n";
        for (double width : widths) {
            double lower = center - width;
            double upper = center + width;

            auto workspace = SwingTrajectoryQpOptimizer::HeightBounds(
                [lower, upper](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
                    return {lower, upper};
                });

            SwingTrajectoryQpOptimizer optimizer(config, terrain);
            auto result = optimizer.optimizeSwingTrajectory(refSpline, 0.0, 0.3, liftOff, touchDown, workspace);

            std::cout << "  [" << lower << ", " << upper << "]: "
                      << (result.solved ? "OK" : "FAIL")
                      << " (iters=" << result.workingSetIterations << ")\n";
        }
    }
}

int main() {
    try {
        testWorkspaceTerrainConflict();
        testWorkspaceScaling();
        std::cout << "\n=== Diagnostics complete ===\n";
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
