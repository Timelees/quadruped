// Offline A/B bench for the swing-height QP against a thin obstacle that the
// nominal reference cannot clear.
//
// Why this exists: the Gazebo loop takes minutes per trial and mixes the swing
// planner with foot placement, state estimation and the WBC, so it cannot say
// "does the QP actually lift the foot over the obstacle". This bench drives the
// production SwingTrajectoryQpOptimizer with an elevation map that matches the
// stair10cm world (60 mm high, 30 mm thick bar) and reports, for the reference
// and for the optimized curve:
//   * minimum foot-centre clearance over the obstacle footprint
//   * peak foot height inside the obstacle's x-band
//   * whether the certifier proves the curve safe
//
// usage: swing_obstacle_bench [--reference-height H] [--clearance C] [...]
#include <ocs2_quadruped_controller/interface/constraint/PolynomialHull.h>
#include <ocs2_quadruped_controller/interface/constraint/SwingCollisionCertifier.h>
#include <ocs2_quadruped_controller/interface/constraint/SwingTrajectoryQPOptimizer.h>
#include <ocs2_quadruped_controller/interface/constraint/SwingTrajectoryPlanner.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

using namespace ocs2::legged_robot;

namespace {
// stair10cm.sdf: <pose>-1.2 0 0.03</pose> with <size>0.03 0.6 0.06</size>.
constexpr double kObstacleXMin = -1.215;
constexpr double kObstacleXMax = -1.185;
constexpr double kObstacleYMin = -0.30;
constexpr double kObstacleYMax = 0.30;
constexpr double kObstacleHeight = 0.06;

// Elevation map as the convex-plane decomposition hands it to the QP: the bar
// survives as a ridge of occupied cells. Resolution matches
// convex_plane_decomposition.yaml (0.02) and its smoothing kernel makes the
// height at the crest ~0.073 m, which is what the previous probe measured.
class ThinObstacleTerrain final : public TerrainInterface {
   public:
    ThinObstacleTerrain(double crest, double resolution) : crest_(crest), resolution_(resolution) {}

    double getHeight(double x, double y) const override {
        if (y < kObstacleYMin || y > kObstacleYMax) return 0.0;
        // Bilinear interpolation over the decomposed patch, matching the
        // production GridMapTerrainInterface: cells whose centre lies inside the
        // bar carry the crest height, everything outside the bar is ground.
        if (x <= kObstacleXMin || x >= kObstacleXMax) return 0.0;
        const double u = (x - kObstacleXMin) / (kObstacleXMax - kObstacleXMin);
        return crest_ * std::min(u, 1.0 - u) * 2.0;
    }

    std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d& lo,
                                        const Eigen::Vector2d& hi) const override {
        std::vector<TerrainPrism> out;
        // Ground everywhere, which is what the decomposition returns outside
        // the bar.
        out.push_back({{-10.0, -10.0}, {10.0, 10.0}, 0.0});
        // Occupied columns across the bar, one per map cell.
        const double y0 = std::max(lo.y(), kObstacleYMin);
        const double y1 = std::min(hi.y(), kObstacleYMax);
        if (y1 <= y0 || hi.x() < kObstacleXMin || lo.x() > kObstacleXMax) return out;
        // Occupied columns across the bar, one per map cell.
        const int first = 0;
        const int last = static_cast<int>(
            std::ceil((kObstacleXMax - kObstacleXMin) / resolution_));
        for (int i = first; i < last; ++i) {
            const double x0 = kObstacleXMin + i * resolution_;
            const double x1 = std::min(x0 + resolution_, kObstacleXMax);
            for (double y = y0; y < y1; y += resolution_) {
                const double yy = std::min(y + resolution_, y1);
                out.push_back({{x0, y}, {x1, yy}, crest_});
            }
        }
        return out;
    }

   private:
    double crest_;
    double resolution_;
};

struct Args {
    double referenceHeight = 0.12;  // task.info swingHeight
    double clearance = 0.02;        // task.info terrainGroundClearance
    double duration = 0.3;          // trot swing phase (swingTimeScale 0.20 -> 0.6)
    double crest = 0.073;           // measured elevation-map height over the bar
    double liftOffVelocity = 0.04;
    double touchDownVelocity = -0.04;
    double footRadius = 0.019;
    double startX = -1.40;          // just before the bar, walking towards -x
    double goalX = -1.00;           // just past it
    bool workspace = true;          // enable the IK workspace rows
    bool verbose = false;
};

double sweepMinClearance(const BSpline& spline, double duration, double radius,
                         double& peakOverBar, double& minOverBar) {
    double minClearance = std::numeric_limits<double>::infinity();
    peakOverBar = -std::numeric_limits<double>::infinity();
    minOverBar = std::numeric_limits<double>::infinity();
    for (int i = 0; i <= 3000; ++i) {
        const double t = duration * i / 3000.0;
        const Eigen::Vector3d p = spline.getPosition(t);
        // The bar only occupies its own footprint; elsewhere the support is the
        // ground plane. Applying the crest height everywhere would report the
        // endpoints (which sit on the ground by construction) as -60 mm.
        const bool overBar = p.x() >= kObstacleXMin && p.x() <= kObstacleXMax &&
                             p.y() >= kObstacleYMin && p.y() <= kObstacleYMax;
        const double surface = overBar ? kObstacleHeight : 0.0;
        const double clr = p.z() - surface;
        minClearance = std::min(minClearance, clr);
        if (p.x() >= kObstacleXMin - radius && p.x() <= kObstacleXMax + radius) {
            peakOverBar = std::max(peakOverBar, p.z());
            minOverBar = std::min(minOverBar, clr);
        }
    }
    return minClearance;
}

Args parse(int argc, char** argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto next = [&]() { return std::stod(argv[++i]); };
        if (key == "--reference-height") a.referenceHeight = next();
        else if (key == "--clearance") a.clearance = next();
        else if (key == "--duration") a.duration = next();
        else if (key == "--crest") a.crest = next();
        else if (key == "--lift-off-velocity") a.liftOffVelocity = next();
        else if (key == "--touch-down-velocity") a.touchDownVelocity = next();
        else if (key == "--start-x") a.startX = next();
        else if (key == "--goal-x") a.goalX = next();
        else if (key == "--no-workspace") a.workspace = false;
        else if (key == "--verbose") a.verbose = true;
        else throw std::invalid_argument("Unknown argument: " + key);
    }
    return a;
}

Eigen::Vector2d ikWorkspace(double, const Eigen::Vector3d& position) {
    // A vertical leg workspace that comfortably contains both the reference and
    // any lifted correction; the terrain rows are the binding constraint here.
    return {-0.10, 0.30 + std::max(0.0, position.z() - 0.30)};
}

// Mirrors the production trigger: the foot lifts off or touches down right next
// to the bar, so the contact-adjacent hull rows carry almost no free-variable
// coefficient while their bound still asks for ground clearance. Dividing such
// a row by its own tiny norm would inflate a millimetre into metres.
int runNearContact(double referenceHeight, double footRadius, bool verbose) {
    const double duration = 0.3;
    const double scale = std::min(1.0, duration / 0.20);
    const double midHeight = scale * referenceHeight;
    BSpline reference = createSwingHeightBSpline(0.0, duration, 0.0, scale * 0.04, midHeight,
                                                0.0, scale * -0.04);
    // Lift-off 20 mm before the bar and touch down 20 mm past it. Both contacts
    // sit on the ground, but the bar's sphere-inflated bound reaches them.
    const double startX = kObstacleXMin - 0.02;
    const double goalX = kObstacleXMax + 0.02;
    {
        Eigen::MatrixXd c = reference.getControlPoints();
        const Eigen::VectorXd progress =
            (Eigen::VectorXd(7) << 0, 0, 0, 0.5, 1, 1, 1).finished();
        c.col(0) = Eigen::VectorXd::Constant(7, startX) + progress * (goalX - startX);
        c.col(1).setZero();
        reference.setControlPoints(c.col(0), c.col(1), c.col(2));
    }
    auto terrain = std::make_shared<ThinObstacleTerrain>(0.073, 0.02);
    SwingTrajectoryQpConfig settings;
    settings.numControlPoints = 15;
    settings.numCurveSamples = 60;
    settings.geometryMode = SwingGeometryMode::Minvo;
    settings.subdivisionsPerSpan = 4;
    settings.groundClearance = 0.02;
    settings.footRadius = footRadius;
    settings.verbose = verbose;
    settings.checkReference = false;
    settings.verifyOutput = false;
    SwingTrajectoryQpOptimizer optimizer(settings, terrain);
    const auto result = optimizer.optimizeSwingTrajectory(
        reference, 0.0, duration, Eigen::Vector3d(startX, 0.0, 0.0),
        Eigen::Vector3d(goalX, 0.0, 0.0));
    if (!result.valid()) {
        std::cout << "  near-contact lift-off/touch-down : INVALID (fallback)\n";
        return 1;
    }
    double peak = 0, minBar = 0;
    sweepMinClearance(*result.bspline, duration, footRadius, peak, minBar);
    std::cout << "  near-contact lift-off/touch-down : SOLVED peak=" << peak
              << " min_bar_clearance=" << minBar << '\n';
    return 0;
}
// Does the production objective prefer one smooth hump over several small
// ripples when both satisfy the same clearance? This isolates the shape
// preference from the constraint geometry.
void probeObjectiveShapePreference() {
    const int n = 15;
    BSpline curve(5);
    const Eigen::VectorXd z = Eigen::VectorXd::Zero(n);
    curve.setControlPoints(z, z, z);
    curve.setKnots(curve.generateKnots(n, 0, 1));
    const Eigen::VectorXd phases = Eigen::VectorXd::LinSpaced(60, 0, 1);
    const Eigen::MatrixXd B = curve.generateSplineBasis(phases);
    Eigen::MatrixXd objective = 0.8 * B.transpose() * B / phases.size();
    const double weights[] = {0.35, 0.2, 0.08};
    for (int order = 1; order <= 3; ++order) {
        const Eigen::MatrixXd D = curve.generateSplineDerivativeBasis(phases, order) /
                                  std::pow(2 * std::acos(-1.0), order);
        objective.noalias() += weights[order - 1] * D.transpose() * D / phases.size();
    }
    objective.diagonal().array() += 0.8 + 1e-8;
    auto score = [&](const Eigen::VectorXd& c) { return c.dot(objective * c); };
    Eigen::VectorXd hump = Eigen::VectorXd::Zero(n), ripple = Eigen::VectorXd::Zero(n);
    for (int i = 0; i < n; ++i) {
        const double s = double(i) / (n - 1);
        hump[i] = 0.05 * std::exp(-std::pow((s - 0.5) / 0.18, 2));
        ripple[i] = 0.05 * (std::exp(-std::pow((s - 0.25) / 0.07, 2)) +
                            std::exp(-std::pow((s - 0.5) / 0.07, 2)) +
                            std::exp(-std::pow((s - 0.75) / 0.07, 2)));
    }
    std::cout << "objective: one_hump J=" << score(hump) << "  three_bumps J=" << score(ripple)
              << (score(ripple) < score(hump) ? "  -> RIPPLE PREFERRED\n" : "  -> single hump preferred\n");
}
}  // namespace

int main(int argc, char** argv) {
    Args args;
    try {
        args = parse(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 2;
    }

    const double scale = std::min(1.0, args.duration / 0.20);
    const double midHeight = scale * args.referenceHeight;
    BSpline reference = createSwingHeightBSpline(
        0.0, args.duration, 0.0, scale * args.liftOffVelocity, midHeight, 0.0,
        scale * args.touchDownVelocity);
    // Embed the height reference along the foot's straight swing path.
    {
        Eigen::MatrixXd c = reference.getControlPoints();
        const Eigen::VectorXd progress =
            (Eigen::VectorXd(7) << 0, 0, 0, 0.5, 1, 1, 1).finished();
        c.col(0) = Eigen::VectorXd::Constant(7, args.startX) +
                   progress * (args.goalX - args.startX);
        c.col(1).setZero();
        reference.setControlPoints(c.col(0), c.col(1), c.col(2));
    }

    auto terrain = std::make_shared<ThinObstacleTerrain>(args.crest, 0.02);

    SwingTrajectoryQpConfig settings;
    settings.numControlPoints = 15;
    settings.numCurveSamples = 60;
    settings.geometryMode = SwingGeometryMode::Minvo;
    settings.subdivisionsPerSpan = 4;
    settings.groundClearance = args.clearance;
    settings.footRadius = args.footRadius;
    settings.verbose = args.verbose;
    settings.checkReference = true;
    settings.verifyOutput = true;
    SwingTrajectoryQpOptimizer optimizer(settings, terrain);

    const Eigen::Vector3d liftOff(args.startX, 0.0, 0.0);
    const Eigen::Vector3d touchDown(args.goalX, 0.0, 0.0);
    SwingCollisionCertifier certifier(terrain, settings.collisionCheck);

    double refPeak = 0, refMinBar = 0, optPeak = 0, optMinBar = 0;
    const double refClearance =
        sweepMinClearance(reference, args.duration, args.footRadius, refPeak, refMinBar);
    const auto refCertificate = certifier.check(reference, args.footRadius);

    const auto workspace = args.workspace ? SwingTrajectoryQpOptimizer::HeightBounds(ikWorkspace)
                                          : SwingTrajectoryQpOptimizer::HeightBounds{};
    const auto result = optimizer.optimizeSwingTrajectory(
        reference, 0.0, args.duration, liftOff, touchDown, workspace);

    // Trackability of the optimized curve. A swing the controller can follow is
    // smooth: correction concentrated in one hump around the obstacle. Reversal
    // count and peak |acceleration| expose the "rippled" curves that the
    // objective's near-zero jerk weight produces.
    struct Shape {
        int reversals = 0;
        double maxAcceleration = 0.0;
        double maxCorrection = 0.0;
        double correctionEnergy = 0.0;
    };
    auto shape = [&](const BSpline& spline) {
        Shape s;
        constexpr int kSamples = 600;
        double previousSlope = 0.0;
        Eigen::Vector3d previous = spline.getPosition(0.0);
        for (int i = 1; i <= kSamples; ++i) {
            const double t = args.duration * i / kSamples;
            const Eigen::Vector3d p = spline.getPosition(t);
            const double slope = p.z() - previous.z();
            if (i > 2 && slope * previousSlope < 0.0 && std::abs(slope) > 1e-6) ++s.reversals;
            if (std::abs(slope) > 1e-9) previousSlope = slope;
            s.maxAcceleration = std::max(s.maxAcceleration, spline.getAcceleration(t).norm());
            const double dz = p.z() - reference.getPosition(t).z();
            s.maxCorrection = std::max(s.maxCorrection, std::abs(dz));
            s.correctionEnergy += dz * dz;
            previous = p;
        }
        s.correctionEnergy = std::sqrt(s.correctionEnergy / kSamples);
        return s;
    };

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "--- scenario ---\n"
              << "reference_height=" << midHeight << " clearance=" << args.clearance
              << " duration=" << args.duration << " crest=" << args.crest << '\n';
    std::cout << "reference : min_clearance=" << refClearance
              << " peak_over_bar=" << refPeak << " min_clearance_over_bar=" << refMinBar
              << " certificate=" << static_cast<int>(refCertificate.status) << '\n';

    if (!result.valid()) {
        std::cout << "optimized : INVALID (QP fallback to reference)\n";
        std::cout << "VERDICT   : FAIL - optimizer produced no usable trajectory\n";
        return 1;
    }
    const double optClearance =
        sweepMinClearance(*result.bspline, args.duration, args.footRadius, optPeak, optMinBar);
    std::cout << "optimized : min_clearance=" << optClearance
              << " peak_over_bar=" << optPeak << " min_clearance_over_bar=" << optMinBar
              << " certificate=" << static_cast<int>(result.certificate.status)
              << " rows=" << result.constraintRows
              << " iterations=" << result.workingSetIterations
              << " correction_cost=" << result.correctionCost << '\n';
    const Shape refShape = shape(reference);
    const Shape optShape = shape(*result.bspline);
    std::cout << "shape     : reference reversals=" << refShape.reversals
              << " max_accel=" << refShape.maxAcceleration << '\n';
    std::cout << "            optimized reversals=" << optShape.reversals
              << " max_accel=" << optShape.maxAcceleration
              << " max_correction=" << optShape.maxCorrection
              << " rms_correction=" << optShape.correctionEnergy << '\n';
    if (args.verbose) {
        std::cerr << "z_profile(s): s,z,z_ref,dz\n";
        for (int i = 0; i <= 60; ++i) {
            const double t = args.duration * i / 60.0;
            const double zo = result.bspline->getPosition(t).z();
            const double zr = reference.getPosition(t).z();
            std::cerr << "PROFILE " << (double(i) / 60.0) << "," << zo << "," << zr << ","
                      << (zo - zr) << "\n";
        }
    }
    const bool clears = optPeak >= kObstacleHeight + args.footRadius;
    const bool proved = result.certificate.status == CollisionStatus::ProvenSafe;
    std::cout << "VERDICT   : " << (clears ? "PASS" : "FAIL")
              << " (foot centre peak " << optPeak << " vs required "
              << kObstacleHeight + args.footRadius << " m)"
              << (proved ? ", certified" : ", NOT certified") << '\n';
    probeObjectiveShapePreference();
    std::cout << "--- contact-adjacent endpoints (production failure mode) ---\n";
    const int nearContact = runNearContact(args.referenceHeight, args.footRadius, args.verbose);
    return clears && nearContact == 0 ? 0 : 1;
}
