// Simulate the exact conditions at trot startup to reproduce the QP failure.
// The key difference from the simple diagnostic is that we simulate the
// actual reference manager behavior, including IK workspace bounds computation.
#include <cmath>
#include <iostream>
#include <memory>
#include <ocs2_quadruped_controller/interface/constraint/SwingTrajectoryQPOptimizer.h>
#include <ocs2_quadruped_controller/interface/constraint/SwingTrajectoryPlanner.h>

using namespace ocs2::legged_robot;

// Terrain with a small step/bump to simulate real terrain data
class StepTerrain final : public TerrainInterface {
public:
    double stepHeight;
    double stepX;

    StepTerrain(double height = 0.01, double x = 0.15) : stepHeight(height), stepX(x) {}

    double getHeight(double x, double) const override {
        return (x > stepX - 0.05 && x < stepX + 0.05) ? stepHeight : 0.0;
    }

    std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d& lo, const Eigen::Vector2d& hi) const override {
        std::vector<TerrainPrism> prisms;
        // Flat region before step
        if (lo.x() < stepX - 0.05) {
            double maxX = std::min(hi.x(), stepX - 0.05);
            prisms.push_back({{lo.x(), lo.y()}, {maxX, hi.y()}, 0.0});
        }
        // Step region
        if (lo.x() < stepX + 0.05 && hi.x() > stepX - 0.05) {
            double minX = std::max(lo.x(), stepX - 0.05);
            double maxX = std::min(hi.x(), stepX + 0.05);
            prisms.push_back({{minX, lo.y()}, {maxX, hi.y()}, stepHeight});
        }
        // Flat region after step
        if (hi.x() > stepX + 0.05) {
            double minX = std::max(lo.x(), stepX + 0.05);
            prisms.push_back({{minX, lo.y()}, {hi.x(), hi.y()}, 0.0});
        }
        if (prisms.empty()) {
            prisms.push_back({{lo.x(), lo.y()}, {hi.x(), hi.y()}, 0.0});
        }
        return prisms;
    }
};

// Simulate a kinematics model that can fail at certain configurations
class SimplifiedLegKinematics {
public:
    double legLength;
    double maxX, maxY, maxZ;

    SimplifiedLegKinematics(double length = 0.28)
        : legLength(length), maxX(0.2), maxY(0.15), maxZ(0.1) {}

    // Returns false if the position is unreachable
    bool isReachable(const Eigen::Vector3d& footPos) const {
        if (std::abs(footPos.x()) > maxX) return false;
        if (std::abs(footPos.y()) > maxY) return false;
        if (footPos.z() > maxZ || footPos.z() < -legLength) return false;
        double reach = footPos.norm();
        return reach < legLength * 0.95;  // 5% margin
    }

    // Compute workspace bounds at a given XY position
    Eigen::Vector2d computeBounds(const Eigen::Vector2d& xy) const {
        // Simplified: if XY is reachable, compute Z bounds
        double xyDist = xy.norm();
        if (xyDist > legLength * 0.9) {
            // Near singularity - very limited Z range
            return {0.0, 0.01};
        }
        double zMax = std::sqrt(std::max(0.0, legLength * legLength - xyDist * xyDist));
        double zMin = -zMax * 0.5;  // Can reach below body
        return {zMin, zMax};
    }
};

void simulateTrotStartup() {
    std::cout << "\n=== Simulating Trot Startup ===\n";

    SwingTrajectoryPlanner::Config config;
    config.useTerrainPlanning = true;
    config.footRadius = 0.019;
    config.swingHeight = 0.085;
    config.liftOffVelocity = 0.04;
    config.touchDownVelocity = -0.04;
    config.terrainQpConfig.numControlPoints = 15;
    config.terrainQpConfig.numCurveSamples = 60;
    config.terrainQpConfig.geometryMode = SwingGeometryMode::Minvo;
    config.terrainQpConfig.subdivisionsPerSpan = 4;
    config.terrainQpConfig.groundClearance = 0.03;
    config.terrainQpConfig.hessianRegularization = 1e-8;
    config.terrainQpConfig.verbose = true;
    config.terrainQpConfig.checkReference = false;
    config.terrainQpConfig.verifyOutput = false;

    auto terrain = std::make_shared<StepTerrain>(0.01, 0.15);
    SimplifiedLegKinematics kinematics;

    SwingTrajectoryPlanner planner(config, 4);
    planner.setTerrainInterface(terrain);

    // Set up a trot gait schedule
    ocs2::ModeSchedule schedule;
    // Trot: FL+RR stance, then FR+RL stance, repeat
    // Event times for one full cycle
    schedule.eventTimes = {0.0, 0.15, 0.3, 0.45, 0.6};
    // Mode numbers: 0=FL,1=FR,2=RL,3=RR contact
    // FL+RR stance = modes 0,3 -> mode number = 1+8 = 9
    // FR+RL stance = modes 1,2 -> mode number = 2+4 = 6
    // Swing all = 0, Stance all = 15
    schedule.modeSequence = {9, 6, 9, 6, 9};  // Alternating trot

    // Heights: feet on ground (z=0)
    feet_array_t<ocs2::scalar_array_t> liftOffHeights, touchDownHeights;
    for (size_t leg = 0; leg < 4; ++leg) {
        liftOffHeights[leg].resize(schedule.modeSequence.size(), 0.0);
        touchDownHeights[leg].resize(schedule.modeSequence.size(), 0.0);
    }

    // Positions: typical trot foot placements
    feet_array_t<std::vector<Eigen::Vector3d>> liftOffPositions, touchDownPositions;
    for (size_t leg = 0; leg < 4; ++leg) {
        liftOffPositions[leg].resize(schedule.modeSequence.size());
        touchDownPositions[leg].resize(schedule.modeSequence.size());
        for (size_t p = 0; p < schedule.modeSequence.size(); ++p) {
            // Foot positions relative to body
            liftOffPositions[leg][p] = Eigen::Vector3d(0.1, (leg < 2 ? 0.1 : -0.1), 0.0);
            touchDownPositions[leg][p] = Eigen::Vector3d(0.15, (leg < 2 ? 0.1 : -0.1), 0.0);
        }
    }

    // Set up workspace that can fail at extreme configurations
    planner.setWorkspace([&kinematics](size_t /*leg*/, double /*t*/, const Eigen::Vector3d& pos) -> Eigen::Vector2d {
        auto bounds = kinematics.computeBounds(pos.head<2>());
        // Simulate IK failure at extreme XY
        if (!kinematics.isReachable(pos)) {
            // Return wide default (what the real code does on IK failure)
            return {-1.0, 1.0};
        }
        return bounds;
    });

    // Call updateTerrain at the start of the first swing
    const double initTime = 0.0;
    const double finalTime = 0.5;

    std::cout << "Calling updateTerrain at initTime=" << initTime << "\n";
    try {
        planner.updateTerrain(schedule, liftOffHeights, touchDownHeights,
                              liftOffPositions, touchDownPositions, initTime, finalTime);
        std::cout << "updateTerrain completed successfully\n";
    } catch (const std::exception& e) {
        std::cerr << "updateTerrain threw: " << e.what() << "\n";
    }
}

void testEdgeCaseWorkspace() {
    std::cout << "\n=== Edge Case: Workspace at Singularity ===\n";

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

    auto terrain = std::make_shared<StepTerrain>();
    SwingTrajectoryQpOptimizer optimizer(config, terrain);

    // Reference spline with unusual endpoint heights (simulating body tilt)
    BSpline refSpline(5);
    const int n = 7;
    Eigen::VectorXd z(n);
    z << -0.005, 0.0, 0.07, 0.12, 0.07, 0.0, 0.005;  // Asymmetric endpoints
    refSpline.setControlPoints(Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n), z);
    refSpline.setKnots(refSpline.generateKnots(n, 0.0, 0.3));

    const Eigen::Vector3d liftOff(0.1, 0.0, -0.005);
    const Eigen::Vector3d touchDown(0.2, 0.0, 0.005);

    // Workspace with very tight bounds (near singularity)
    auto tightWorkspace = SwingTrajectoryQpOptimizer::HeightBounds(
        [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
            return {-0.001, 0.001};  // Only 2mm of Z range
        });

    std::cout << "\n--- With tight workspace [-0.001, 0.001] ---\n";
    auto result = optimizer.optimizeSwingTrajectory(refSpline, 0.0, 0.3, liftOff, touchDown, tightWorkspace);
    std::cout << "Solved: " << (result.solved ? "YES" : "NO") << "\n";
    std::cout << "Working set iterations: " << result.workingSetIterations << "\n";
    std::cout << "Correction cost: " << result.correctionCost << "\n";
}

void testZeroDurationSwing() {
    std::cout << "\n=== Edge Case: Very Short Swing Duration ===\n";

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

    auto terrain = std::make_shared<StepTerrain>();
    SwingTrajectoryQpOptimizer optimizer(config, terrain);

    // Very short swing (can happen during fast trot)
    const double swingDuration = 0.05;  // 50ms

    BSpline refSpline(5);
    const int n = 7;
    Eigen::VectorXd z(n);
    const double endpointOffset = 0.04 * swingDuration / 12.0;
    z << 0.0, endpointOffset, 0.07, 0.13, 0.07, endpointOffset, 0.0;
    refSpline.setControlPoints(Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n), z);
    refSpline.setKnots(refSpline.generateKnots(n, 0.0, 1.0));

    const Eigen::Vector3d liftOff(0.1, 0.0, 0.0);
    const Eigen::Vector3d touchDown(0.12, 0.0, 0.0);  // Short step

    auto wideWorkspace = SwingTrajectoryQpOptimizer::HeightBounds(
        [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
            return {-1.0, 1.0};
        });

    std::cout << "\n--- Swing duration = " << swingDuration << "s ---\n";
    auto result = optimizer.optimizeSwingTrajectory(refSpline, 0.0, swingDuration, liftOff, touchDown, wideWorkspace);
    std::cout << "Solved: " << (result.solved ? "YES" : "NO") << "\n";
    std::cout << "Working set iterations: " << result.workingSetIterations << "\n";
    std::cout << "Correction cost: " << result.correctionCost << "\n";
}

int main() {
    try {
        simulateTrotStartup();
        testEdgeCaseWorkspace();
        testZeroDurationSwing();
        std::cout << "\n=== Simulation complete ===\n";
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
