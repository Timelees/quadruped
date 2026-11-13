// Diagnose: why does the QP produce large corrections on flat terrain?
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

void analyzeCorrectionMagnitude() {
    std::cout << "=== Flat Terrain Correction Analysis ===\n\n";

    // Use current config matching task.info
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

    // Typical swing reference from task.info:
    // swingHeight = 0.085, liftOffVelocity = 0.04, touchDownVelocity = -0.04
    BSpline refSpline(5);
    const int nRef = 7;
    Eigen::VectorXd z(nRef);
    z << 0.0, 0.002, 0.07, 0.13, 0.07, 0.002, 0.0;
    refSpline.setControlPoints(Eigen::VectorXd::Zero(nRef), Eigen::VectorXd::Zero(nRef), z);
    refSpline.setKnots(refSpline.generateKnots(nRef, 0.0, 0.3));

    const Eigen::Vector3d liftOff(0.1, 0.0, 0.0);
    const Eigen::Vector3d touchDown(0.2, 0.0, 0.0);

    // Case 1: No workspace (useTerrainPlanning=false equivalent)
    std::cout << "--- Case 1: No workspace (no optimization) ---\n";
    auto resultNoWs = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
        refSpline, 0.0, 0.3, liftOff, touchDown, {});

    if (resultNoWs.solved && resultNoWs.bspline) {
        const auto cp = resultNoWs.bspline->getControlPoints();
        std::cout << "Fitted control points z:\n  ";
        for (int i = 0; i < cp.rows(); ++i) {
            printf("%.4f ", cp(i, 2));
        }
        std::cout << "\nCorrection cost: " << resultNoWs.correctionCost << "\n";
        std::cout << "Max |correction|: ";
        double maxCorr = 0;
        for (int i = 0; i < cp.rows(); ++i) {
            // Compare with reference control points (which are the same as fitted for no workspace)
        }
        std::cout << "\n";

        // Sample curve at 20 points
        std::cout << "Sampled z (20 points):\n  ";
        for (int i = 0; i <= 20; ++i) {
            double t = 0.3 * i / 20.0;
            printf("%.4f ", resultNoWs.bspline->getPosition(t).z());
        }
        std::cout << "\n\n";
    }

    // Case 2: Wide workspace (IK failure fallback) - this is what happens on flat terrain
    std::cout << "--- Case 2: Wide workspace {-1, 1} (IK failure on flat) ---\n";
    auto wideWs = SwingTrajectoryQpOptimizer::HeightBounds(
        [](double, const Eigen::Vector3d&) -> Eigen::Vector2d {
            return {-1.0, 1.0};
        });
    auto resultWideWs = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
        refSpline, 0.0, 0.3, liftOff, touchDown, wideWs);

    if (resultWideWs.solved && resultWideWs.bspline) {
        const auto cp = resultWideWs.bspline->getControlPoints();
        std::cout << "Fitted control points z:\n  ";
        for (int i = 0; i < cp.rows(); ++i) {
            printf("%.4f ", cp(i, 2));
        }
        std::cout << "\nCorrection cost: " << resultWideWs.correctionCost << "\n";
        std::cout << "Sampled z (20 points):\n  ";
        for (int i = 0; i <= 20; ++i) {
            double t = 0.3 * i / 20.0;
            printf("%.4f ", resultWideWs.bspline->getPosition(t).z());
        }
        std::cout << "\n\n";
    }

    // Case 3: Compare reference vs fitted
    std::cout << "--- Case 3: Point-by-point comparison ---\n";
    std::cout << "t      | reference | no-workspace | wide-workspace\n";
    for (int i = 0; i <= 10; ++i) {
        double t = 0.3 * i / 10.0;
        double ref = refSpline.getPosition(t).z();
        double noWs = resultNoWs.bspline ? resultNoWs.bspline->getPosition(t).z() : -1;
        double wideWsVal = resultWideWs.bspline ? resultWideWs.bspline->getPosition(t).z() : -1;
        printf("%.3f  |  %.4f   |   %.4f     |    %.4f\n", t, ref, noWs, wideWsVal);
    }

    // Case 4: Check terrain constraints contribution
    std::cout << "\n--- Case 4: Terrain constraint analysis ---\n";
    // On flat terrain, the terrain clearance is 0 + groundClearance*envelope
    // The envelope at midpoint is groundClearance*1.0 = 0.03
    // Reference peak is ~0.084, so terrain constraints shouldn't bind
    // But MINVO hull constraints add clearance too - let's check what happens

    // Compare geometry modes
    std::cout << "Geometry mode comparison on flat terrain:\n";
    const SwingGeometryMode modes[] = {SwingGeometryMode::Discrete, SwingGeometryMode::Bezier, SwingGeometryMode::Minvo};
    const char* modeNames[] = {"Discrete", "Bezier", "Minvo"};

    for (int m = 0; m < 3; ++m) {
        config.geometryMode = modes[m];
        auto res = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
            refSpline, 0.0, 0.3, liftOff, touchDown, {});
        std::cout << "  " << modeNames[m] << ": solved=" << res.solved
                  << ", cost=" << res.correctionCost
                  << ", iters=" << res.workingSetIterations
                  << ", rows=" << res.constraintRows << "\n";
        if (res.bspline) {
            double maxDev = 0;
            for (int i = 0; i <= 100; ++i) {
                double t = 0.3 * i / 100.0;
                double dev = std::abs(res.bspline->getPosition(t).z() - refSpline.getPosition(t).z());
                maxDev = std::max(maxDev, dev);
            }
            std::cout << "    Max deviation from reference: " << maxDev << "\n";
        }
    }
}

void analyzeTerrainClearanceConstraints() {
    std::cout << "\n=== Terrain Clearance Constraint Analysis ===\n";

    SwingTrajectoryQpConfig config;
    config.numControlPoints = 15;
    config.numCurveSamples = 60;
    config.geometryMode = SwingGeometryMode::Minvo;
    config.subdivisionsPerSpan = 4;
    config.groundClearance = 0.03;
    config.footRadius = 0.019;
    config.hessianRegularization = 1e-4;
    config.verbose = true;
    config.checkReference = false;
    config.verifyOutput = false;

    auto terrain = std::make_shared<FlatTerrain>();

    // Reference with low swing height - might be close to terrain constraint
    BSpline refSpline(5);
    const int nRef = 7;
    Eigen::VectorXd z(nRef);
    z << 0.0, 0.002, 0.07, 0.13, 0.07, 0.002, 0.0;
    refSpline.setControlPoints(Eigen::VectorXd::Zero(nRef), Eigen::VectorXd::Zero(nRef), z);
    refSpline.setKnots(refSpline.generateKnots(nRef, 0.0, 0.3));

    const Eigen::Vector3d liftOff(0.1, 0.0, 0.0);
    const Eigen::Vector3d touchDown(0.2, 0.0, 0.0);

    std::cout << "Reference z at control points phases (in [0,1]):\n";
    // Build fitted curve manually
    SwingTrajectoryQpOptimizer optimizer(config, terrain);
    auto result = optimizer.optimizeSwingTrajectory(refSpline, 0.0, 0.3, liftOff, touchDown, {});

    if (result.bspline) {
        std::cout << "Reference vs Optimized at curve sample points:\n";
        std::cout << "phase | ref z    | opt z    | terrain+clearance | diff\n";
        for (int i = 0; i <= 20; ++i) {
            double phase = double(i) / 20.0;
            double t = 0.3 * phase;
            double refZ = refSpline.getPosition(t).z();
            double optZ = result.bspline->getPosition(t).z();
            // terrain height = 0, clearance envelope at this phase
            double s = phase;
            double swingProgress = s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
            double clearance = config.groundClearance * swingProgress * 2 * std::min(s, 1-s) / std::min(s, 1-s);  // rough
            printf("%.3f  | %.4f  | %.4f  | (varies)           | %.4f\n",
                   phase, refZ, optZ, optZ - refZ);
        }
    }
}

int main() {
    try {
        analyzeCorrectionMagnitude();
        analyzeTerrainClearanceConstraints();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
