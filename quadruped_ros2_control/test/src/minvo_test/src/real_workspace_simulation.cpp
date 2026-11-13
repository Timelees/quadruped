// Simulate real IK-based workspace behavior on flat terrain
// to understand why trajectory correction is happening
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

// Simulate a realistic IK-based workspace that depends on body height and foot position
class RealisticWorkspaceSimulator {
public:
    double bodyHeight;
    double legLength;

    RealisticWorkspaceSimulator(double bodyH = 0.28, double legL = 0.3)
        : bodyHeight(bodyH), legLength(legL) {}

    Eigen::Vector2d computeBounds(const Eigen::Vector3d& footPos) {
        // Simplified IK model: foot must be within leg reach from body
        // Body is at (0, 0, bodyHeight), foot at footPos
        double dx = footPos.x();
        double dy = footPos.y();
        double dz = footPos.z() - bodyHeight;

        double horizontalDist = std::sqrt(dx * dx + dy * dy);
        double maxReach = legLength;

        // Check if foot is reachable at all
        if (horizontalDist > maxReach * 0.95) {
            // Near singularity - very limited Z range
            return {footPos.z() - 0.01, footPos.z() + 0.01};
        }

        // Compute Z bounds based on leg geometry
        double maxVertical = std::sqrt(std::max(0.0, maxReach * maxReach - horizontalDist * horizontalDist));
        double minVertical = -maxVertical * 0.7;  // Can reach below body

        // Transform to world frame
        double zMax = bodyHeight + maxVertical;
        double zMin = bodyHeight + minVertical;

        // Clamp to ground (can't go below ground)
        zMin = std::max(zMin, -0.02);

        return {zMin, zMax};
    }
};

void simulateFlatTerrainWalking() {
    std::cout << "=== Flat Terrain Walking Simulation ===\n\n";

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

    std::cout << "Reference trajectory peak: " << z.maxCoeff() << "m\n";
    std::cout << "Reference trajectory at midpoint: " << refSpline.getPosition(0.15).z() << "m\n\n";

    // Simulate different body heights
    for (double bodyHeight : {0.25, 0.27, 0.28, 0.29, 0.31}) {
        std::cout << "--- Body height = " << bodyHeight << "m ---\n";
        RealisticWorkspaceSimulator ws(bodyHeight, 0.3);

        // Check workspace at key points along the swing
        std::cout << "Workspace bounds at swing positions:\n";
        for (double phase : {0.0, 0.25, 0.5, 0.75, 1.0}) {
            Eigen::Vector3d pos = liftOff + phase * (touchDown - liftOff);
            pos.z() = refSpline.getPosition(0.3 * phase).z();  // Reference height

            auto bounds = ws.computeBounds(pos);
            double refZ = refSpline.getPosition(0.3 * phase).z();
            bool containsRef = (bounds[0] <= refZ && refZ <= bounds[1]);

            printf("  phase=%.2f: pos.z=%.4f, bounds=[%.4f, %.4f], range=%.4f, containsRef=%s\n",
                   phase, refZ, bounds[0], bounds[1], bounds[1] - bounds[0],
                   containsRef ? "YES" : "NO");
        }

        // Run QP optimization with this workspace
        auto workspace = SwingTrajectoryQpOptimizer::HeightBounds(
            [&ws](double, const Eigen::Vector3d& pos) -> Eigen::Vector2d {
                return ws.computeBounds(pos);
            });

        auto result = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
            refSpline, 0.0, 0.3, liftOff, touchDown, workspace);

        std::cout << "QP result: solved=" << result.solved
                  << ", cost=" << result.correctionCost
                  << ", iters=" << result.workingSetIterations << "\n";

        if (result.bspline && result.correctionCost > 1e-6) {
            std::cout << "Optimized trajectory:\n  ";
            for (int i = 0; i <= 10; ++i) {
                printf("%.4f ", result.bspline->getPosition(0.3 * i / 10.0).z());
            }
            std::cout << "\n";
        }
        std::cout << "\n";
    }
}

void analyzeWorkspaceTightness() {
    std::cout << "\n=== Workspace Tightness Analysis ===\n\n";

    // The key question: at what body height does workspace start to constrain the reference?
    // Reference peak is 0.084m at phase=0.5
    // Foot XY at phase=0.5 is approximately (0.15, 0.0)

    RealisticWorkspaceSimulator ws(0.28, 0.3);  // Default body height

    Eigen::Vector3d midSwingPos(0.15, 0.0, 0.084);  // Peak of reference
    auto bounds = ws.computeBounds(midSwingPos);

    std::cout << "At swing midpoint (x=0.15, z=0.084):\n";
    std::cout << "  Workspace bounds: [" << bounds[0] << ", " << bounds[1] << "]\n";
    std::cout << "  Reference z: 0.084\n";
    std::cout << "  Contains reference: " << (bounds[1] >= 0.084 ? "YES" : "NO") << "\n\n";

    // Find the critical body height where workspace starts to constrain
    std::cout << "Critical body height analysis:\n";
    for (double bodyH = 0.20; bodyH <= 0.35; bodyH += 0.01) {
        RealisticWorkspaceSimulator wsTest(bodyH, 0.3);
        auto boundsTest = wsTest.computeBounds(midSwingPos);
        bool constrains = (boundsTest[1] < 0.084);
        if (constrains) {
            printf("  bodyH=%.2f: zMax=%.4f < 0.084 -> CONSTRAINS\n", bodyH, boundsTest[1]);
        }
    }
}

int main() {
    try {
        simulateFlatTerrainWalking();
        analyzeWorkspaceTightness();
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
