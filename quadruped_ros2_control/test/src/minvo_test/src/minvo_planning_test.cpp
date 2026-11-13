// Production geometry/optimizer checks and deterministic paired benchmark.
#include <ocs2_quadruped_controller/interface/constraint/SwingTrajectoryQPOptimizer.h>

#include <Eigen/Eigenvalues>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>

using namespace ocs2::legged_robot;
namespace {
void require(bool value, const char *message) {
    if (!value) {
        throw std::runtime_error(message);
    }
}
class BoxTerrain final : public TerrainInterface {
   public:
    double center = 0.253, width = 0.006, height = 0.12;
    bool unknown = false;
    double getHeight(double x, double y) const override {
        if (unknown) {
            throw TerrainUnavailable("Synthetic unknown map");
        }
        return std::abs(x - center) <= 0.5 * width && std::abs(y) <= 0.10 ? height : 0;
    }
    std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d &lo, const Eigen::Vector2d &hi) const override {
        if (unknown || (lo.array() < -2).any() || (hi.array() > 2).any()) {
            throw TerrainUnavailable("Synthetic unknown map");
        }
        std::vector<TerrainPrism> result{{{-2, -2}, {2, 2}, 0}};
        if (hi.x() >= center - 0.5 * width && lo.x() <= center + 0.5 * width && hi.y() >= -0.1 &&
            lo.y() <= 0.1) {
            result.push_back({{center - 0.5 * width, -0.1}, {center + 0.5 * width, 0.1}, height});
        }
        return result;
    }
};
BSpline reference() {
    BSpline spline(5);
    Eigen::VectorXd z(7), x(7), y = Eigen::VectorXd::Zero(7);
    z << 0, 0.0016, 0.0032, 0.15, 0.0032, 0.0016, 0;
    x << 0, 0, 0, 0.25, 0.5, 0.5, 0.5;
    spline.setControlPoints(x, y, z);
    spline.setKnots(spline.generateKnots(7, 1, 1.4));
    return spline;
}
double bernstein(int degree, int i, double u) {
    double choose = 1;
    for (int j = 1; j <= i; ++j) {
        choose *= double(degree - j + 1) / j;
    }
    return choose * std::pow(u, i) * std::pow(1 - u, degree - i);
}
void checkExtraction() {
    BSpline spline(5);
    Eigen::VectorXd knots(15);
    knots << 1, 1, 1, 1, 1, 1, 1.07, 1.23, 1.31, 1.4, 1.4, 1.4, 1.4, 1.4, 1.4;
    Eigen::MatrixXd controls(9, 3);
    std::mt19937 rng(47);
    std::uniform_real_distribution<double> random(-0.2, 0.2);
    for (int i = 0; i < controls.rows(); ++i) {
        for (int j = 0; j < 3; ++j) {
            controls(i, j) = random(rng);
        }
    }
    spline.setControlPoints(controls.col(0), controls.col(1), controls.col(2));
    spline.setKnots(knots);
    double maxError = 0;
    for (const auto &span : extractPolynomialSegments(spline)) {
        const auto halves = span.bisect();
        for (const auto &piece : {span, halves.first, halves.second}) {
            const Eigen::MatrixXd bezier = piece.bezierMap * controls,
                                  minvo = hullMap(piece, HullBasis::Minvo) * controls;
            for (int i = 0; i <= 100; ++i) {
                const double u = double(i) / 100, s = 2 * u - 1;
                Eigen::Vector3d p = Eigen::Vector3d::Zero();
                for (int j = 0; j <= 5; ++j) {
                    p += bernstein(5, j, u) * bezier.row(j).transpose();
                }
                Eigen::Matrix<double, 6, 1> powers;
                for (int j = 0; j <= 5; ++j) {
                    powers[j] = std::pow(s, 5 - j);
                }
                const auto alpha = minvoBasisMatrix() * powers;
                require(std::abs(alpha.sum() - 1) < 1e-12 && alpha.minCoeff() > -1e-12,
                        "Invalid MINVO basis");
                const Eigen::Vector3d pmv = minvo.transpose() * alpha;
                const auto actual = spline.getPosition(piece.start + u * (piece.end - piece.start));
                maxError = std::max({maxError, (p - actual).norm(), (pmv - actual).norm()});
            }
        }
    }
    require(maxError < 1e-11, "Exact knot extraction/subdivision/basis reconstruction failed");
    BSpline collapsed(5);
    collapsed.setControlPoints(Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6));
    collapsed.setKnots(Eigen::VectorXd::Zero(12));
    bool rejected = false;
    try {
        extractPolynomialSegments(collapsed);
    } catch (const std::invalid_argument &) {
        rejected = true;
    }
    require(rejected, "Zero-length spline domain was accepted");
    // Check basis minima at endpoints and every real stationary point, not just
    // samples.
    for (int row = 0; row < 6; ++row) {
        const auto c = minvoBasisMatrix().row(row);
        Eigen::Matrix4d companion = Eigen::Matrix4d::Zero();
        companion.block<3, 3>(1, 0).setIdentity();
        for (int i = 0; i < 4; ++i) {
            companion(i, 3) = -(i + 1) * c[4 - i] / (5 * c[0]);
        }
        Eigen::EigenSolver<Eigen::Matrix4d> roots(companion, false);
        for (int i = 0; i < 4; ++i) {
            const auto root = roots.eigenvalues()[i];
            if (std::abs(root.imag()) < 1e-7 && root.real() >= -1 && root.real() <= 1) {
                double value = 0;
                for (int j = 0; j < 6; ++j) {
                    value = value * root.real() + c[j];
                }
                require(value > -1e-12, "MINVO polynomial has a negative stationary minimum");
            }
        }
    }
    std::cout << "Maximum reconstruction error: " << maxError << '\n';
}
void checkMap() {
    auto terrain = std::make_shared<convex_plane_decomposition::PlanarTerrain>();
    auto &map = terrain->gridMap;
    map.setGeometry(grid_map::Length(2, 2), 0.02);
    map.add("elevation", 0.0);
    GridMapTerrainInterface query(terrain, "elevation");
    for (int move = 0; move < 2; ++move) {
        if (move) {
            map.move({0.12, -0.08});
        }
        for (grid_map::Index index(0, 0); index[0] < map.getSize()[0]; ++index[0]) {
            for (index[1] = 0; index[1] < map.getSize()[1]; ++index[1]) {
                grid_map::Position xy;
                map.getPosition(index, xy);
                map.at("elevation", index) = 0.1 + 0.2 * xy.x() - 0.15 * xy.y() + 0.05 * xy.x() * xy.y();
            }
        }
        for (double x : {-.4, 0.0, .4}) {
            require(std::abs(query.getHeight(x, .1) - (.1 + .2 * x - .015 + .005 * x)) < 1e-7,
                    "Bilinear map/ring-buffer mismatch");
        }
    }
    map.atPosition("elevation", {0.01, 0.01}) = NAN;
    bool caught = false;
    try {
        query.getPrisms({-.04, -.04}, {.04, .04});
    } catch (const TerrainUnavailable &) {
        caught = true;
    }
    require(caught, "Unknown interpolation stencil was certified");
}
void checkCertificates() {
    auto terrain = std::make_shared<BoxTerrain>();
    terrain->height = 0;
    CollisionCheckConfig cfg;
    cfg.basis = HullBasis::Bezier;
    auto nominal = reference();
    require(SwingCollisionCertifier(terrain, cfg).check(nominal, .019).status == CollisionStatus::ProvenSafe,
            "Flat-ground contact reference not certified");
    // The original hull overlaps the floor, but the curve is positive: refine, do
    // not declare collision.
    BSpline valley(5);
    Eigen::VectorXd valleyZ(6);
    for (int i = 0; i < 6; ++i) {
        valleyZ[i] = .29 - double(i) / 5 + double(i * (i - 1)) / 20;
    }
    valley.setControlPoints(Eigen::VectorXd::Zero(6), Eigen::VectorXd::Zero(6), valleyZ);
    valley.setKnots(valley.generateKnots(6, 0, 1));
    const auto refined = SwingCollisionCertifier(terrain, cfg).check(valley, .019);
    require(refined.status == CollisionStatus::ProvenSafe && refined.subdivisions > 0,
            "Hull overlap was not resolved by exact subdivision");
    auto shallow = cfg;
    shallow.maxDepth = 0;
    require(SwingCollisionCertifier(terrain, shallow).check(valley, .019).status == CollisionStatus::Unknown,
            "Unresolved hull overlap became a collision or SAFE");
    // A diagonal path passes beside a tall box despite overlapping its XY
    // bounding rectangle. This must pass by a non-axis separating projection, not
    // a max-height-only test.
    terrain->height = .18;
    BSpline diagonal(5);
    const Eigen::VectorXd linear = Eigen::VectorXd::LinSpaced(6, 0, .5);
    diagonal.setControlPoints(linear, linear, Eigen::VectorXd::Constant(6, .03));
    diagonal.setKnots(diagonal.generateKnots(6, 0, 1));
    const auto side = SwingCollisionCertifier(terrain, cfg).check(diagonal, .019);
    require(side.status == CollisionStatus::ProvenSafe && side.subdivisions == 0,
            "Geometric certificate did not use separating hull/prism geometry");
    terrain->height = .18;
    require(SwingCollisionCertifier(terrain, cfg).check(nominal, .019).status != CollisionStatus::ProvenSafe,
            "Thin obstacle missed by continuous checker");
    terrain->unknown = true;
    require(SwingCollisionCertifier(terrain, cfg).check(nominal, .019).status == CollisionStatus::Unknown,
            "Unknown map became SAFE");
    terrain->unknown = false;
    terrain->height = .02;
    cfg.maxSegments = 1;
    require(SwingCollisionCertifier(terrain, cfg).check(nominal, .019).status == CollisionStatus::Unknown,
            "Unvisited spans became SAFE");
}
void checkOptimizer() {
    auto terrain = std::make_shared<BoxTerrain>();
    SwingTrajectoryQpConfig config;
    config.numCurveSamples = 30;
    config.groundClearance = .005;
    for (auto mode : {SwingGeometryMode::Bezier, SwingGeometryMode::Minvo}) {
        config.geometryMode = mode;
        SwingTrajectoryQpOptimizer optimizer(config, terrain);
        const auto result = optimizer.optimizeSwingTrajectory(reference(), 1, 1.4, {0, 0, 0}, {.5, 0, 0});
        require(result.valid() && result.certificate.status == CollisionStatus::ProvenSafe,
                "Continuous optimizer failed feasible obstacle");
        for (double t : {1., 1.4}) {
            require((result.bspline->getPosition(t) - Eigen::Vector3d(t == 1 ? 0 : .5, 0, 0)).norm() < 1e-10,
                    "Changed contact endpoints");
            require(std::abs(result.bspline->getVelocity(t).z() - reference().getVelocity(t).z()) < 1e-9,
                    "Changed contact velocity");
            require(
                std::abs(result.bspline->getAcceleration(t).z() - reference().getAcceleration(t).z()) < 1e-8,
                "Changed contact acceleration");
        }
        const auto blocked = optimizer.optimizeSwingTrajectory(
            reference(), 1, 1.4, {0, 0, 0}, {.5, 0, 0},
            [](double, const Eigen::Vector3d &) { return Eigen::Vector2d(0, .06); });
        require(!blocked.valid(), "Obstacle/workspace conflict was accepted");
        terrain->center = 0;
        require(!optimizer.optimizeSwingTrajectory(reference(), 1, 1.4, {0, 0, 0}, {.5, 0, 0}).valid(),
                "Contact registration lowered real obstacle");
        terrain->center = .253;
    }
}
void benchmark(const std::filesystem::path &directory) {
    std::filesystem::create_directories(directory);
    std::ofstream metrics(directory / "metrics.csv"), trajectories(directory / "trajectories.csv");
    std::ofstream hulls(directory / "hull_bounds.csv");
    hulls << "true_min,bezier_lower,minvo_lower\n";
    const PolynomialSegment identity{0, 1, Eigen::MatrixXd::Identity(6, 6)};
    for (int sample = 0; sample <= 100; ++sample) {
        const double minimum = .001 * sample;
        Eigen::VectorXd b(6);
        for (int i = 0; i < 6; ++i) {
            b[i] = .25 + minimum - double(i) / 5 + double(i * (i - 1)) / 20;
        }
        hulls << minimum << ',' << b.minCoeff() << ',' << (hullMap(identity, HullBasis::Minvo) * b).minCoeff()
              << '\n';
    }
    metrics << "case_id,mode,height,width,center,repeat,solved,certified,min_"
               "clearance,peak_z,correction_rms,"
               "objective,planning_ms,rows,iterations,segments\n";
    trajectories << "case_id,mode,t,x,z,terrain,required,nominal_z\n";
    metrics << std::setprecision(12);
    trajectories << std::setprecision(12);
    int caseId = 0;
    for (double height : {.06, .10, .14}) {
        for (double width : {.002, .01, .04}) {
            for (double center : {.217, .253, .289}) {
                auto terrain = std::make_shared<BoxTerrain>();
                terrain->height = height;
                terrain->width = width;
                terrain->center = center;
                for (int mode = 0; mode < 3; ++mode) {
                    SwingTrajectoryQpConfig config;
                    config.geometryMode = static_cast<SwingGeometryMode>(mode);
                    config.numCurveSamples = 15;
                    config.subdivisionsPerSpan = 4;
                    config.groundClearance = .005;
                    config.verifyOutput = mode != 0;  // discrete ablation is explicitly uncertified,
                                                      // measured independently below
                    config.qpMaxWorkingSetRecalculations = 200;
                    SwingTrajectoryQpOptimizer optimizer(config, terrain);
                    for (int repeat = 0; repeat < 5; ++repeat) {
                        const auto nominalCurve = reference();
                        const auto begin = std::chrono::steady_clock::now();
                        const auto result =
                            optimizer.optimizeSwingTrajectory(reference(), 1, 1.4, {0, 0, 0}, {.5, 0, 0});
                        const double ms = std::chrono::duration<double, std::milli>(
                                              std::chrono::steady_clock::now() - begin)
                                              .count();
                        double minClearance = NAN, peak = NAN, rms = NAN;
                        CollisionCertificate certificate;
                        if (result.valid()) {
                            certificate =
                                SwingCollisionCertifier(terrain).check(*result.bspline, config.footRadius);
                            minClearance = 1e9;
                            peak = -1e9;
                            rms = 0;
                            for (int i = 0; i <= 4000; ++i) {
                                const double t = 1 + .4 * i / 4000.;
                                const auto p = result.bspline->getPosition(t);
                                const double required = terrain->getSphereClearanceUpperBound(
                                    p.head<2>(), p.head<2>(), config.footRadius);
                                minClearance = std::min(minClearance, p.z() - required);
                                peak = std::max(peak, p.z());
                                const double nominal = nominalCurve.getPosition(t).z();
                                rms += std::pow(p.z() - nominal, 2);
                                if (repeat == 0 && i % 4 == 0) {
                                    trajectories << caseId << ',' << mode << ',' << t << ',' << p.x() << ','
                                                 << p.z() << ',' << terrain->getHeight(p.x(), 0) << ','
                                                 << required << ',' << nominal << '\n';
                                }
                            }
                            rms = std::sqrt(rms / 4001);
                        }
                        metrics << caseId << ',' << mode << ',' << height << ',' << width << ',' << center
                                << ',' << repeat << ',' << result.valid() << ','
                                << (certificate.status == CollisionStatus::ProvenSafe) << ',' << minClearance
                                << ',' << peak << ',' << rms << ',' << result.correctionCost << ',' << ms
                                << ',' << result.constraintRows << ',' << result.workingSetIterations << ','
                                << certificate.checkedSegments << '\n';
                    }
                }
                ++caseId;
            }
        }
    }
    std::cout << "Paired benchmark: " << caseId << " cases, 3 modes, 5 repeats: " << directory << '\n';
}
}  // namespace
int main(int argc, char **argv) {
    try {
        checkExtraction();
        checkMap();
        checkCertificates();
        checkOptimizer();
        std::cout << "MINVO planning checks passed.\n";
        if (argc == 2) {
            benchmark(argv[1]);
        }
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
