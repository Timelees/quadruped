// Verbose diagnostic for a specific failing case
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

int main() {
    SwingTrajectoryQpConfig config;
    config.numControlPoints = 15;
    config.numCurveSamples = 60;
    config.geometryMode = SwingGeometryMode::Minvo;
    config.subdivisionsPerSpan = 4;
    config.groundClearance = 0.03;
    config.footRadius = 0.019;
    config.hessianRegularization = 1e-8;
    config.verbose = true;  // Enable verbose output
    config.checkReference = false;
    config.verifyOutput = false;

    auto terrain = std::make_shared<FlatTerrain>();
    SwingTrajectoryQpOptimizer optimizer(config, terrain);

    BSpline refSpline(5);
    const int nRef = 7;
    Eigen::VectorXd z(nRef);
    z << 0.0, 0.002, 0.07, 0.13, 0.07, 0.002, 0.0;
    refSpline.setControlPoints(Eigen::VectorXd::Zero(nRef), Eigen::VectorXd::Zero(nRef), z);
    refSpline.setKnots(refSpline.generateKnots(nRef, 0.0, 0.3));

    const Eigen::Vector3d liftOff(0.1, 0.0, 0.0);
    const Eigen::Vector3d touchDown(0.2, 0.0, 0.0);

    auto failingWorkspace = SwingTrajectoryQpOptimizer::HeightBounds(
        [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
            return {0.05, 0.15};
        });

    std::cout << "Testing workspace {0.05, 0.15} with verbose output:\n\n";
    auto result = optimizer.optimizeSwingTrajectory(refSpline, 0.0, 0.3, liftOff, touchDown, failingWorkspace);
    std::cout << "\nResult: " << (result.solved ? "OK" : "FAIL") << "\n";
    return 0;
}
