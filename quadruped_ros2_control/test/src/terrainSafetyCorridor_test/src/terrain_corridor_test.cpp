#include <ocs2_quadruped_controller/interface/constraint/SwingTrajectoryQPOptimizer.h>
#include <ocs2_quadruped_controller/perceptive/visualize/TerrainCorridorVisualization.h>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <ocs2_quadruped_controller/interface/constraint/SwingTrajectoryPlanner.h>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

using namespace ocs2::legged_robot;
namespace {
void require(bool value, const std::string& message) {
    if (!value) throw std::runtime_error(message);
}

class AnalyticTerrain final : public TerrainInterface {
 public:
    double floor = 0.0;
    bool unknown = false, invalid = false;
    std::vector<TerrainPrism> obstacles;
    double getHeight(double x, double y) const override {
        if (unknown) throw TerrainUnavailable("Unknown test terrain");
        double h = floor;
        for (const auto& p : obstacles) {
            if (x >= p.minXY.x() && x <= p.maxXY.x() && y >= p.minXY.y() && y <= p.maxXY.y())
                h = std::max(h, p.height);
        }
        return h;
    }
    std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d& lo, const Eigen::Vector2d& hi) const override {
        if (unknown || (lo.array() < -2.0).any() || (hi.array() > 2.0).any())
            throw TerrainUnavailable("Unknown/outside test map");
        std::vector<TerrainPrism> result{{{-2, -2}, {2, 2}, floor}};
        for (const auto& p : obstacles) {
            if ((p.maxXY.array() >= lo.array()).all() && (p.minXY.array() <= hi.array()).all())
                result.push_back(p);
        }
        if (invalid) result.front().height = std::numeric_limits<double>::quiet_NaN();
        return result;
    }
};

BSpline reference(double floor = 0.0, double rise = 0.0, double yaw = 0.0, bool stationary = false) {
    BSpline curve(5);
    Eigen::VectorXd progress(7), z(7);
    progress << 0, 0, 0, 0.25, 0.5, 0.5, 0.5;
    z << 0, 0.004, 0.008, 0.30, 0.008, 0.004, 0;
    z.array() += floor;
    z += 2.0 * rise * progress;
    if (stationary) progress.setZero();
    curve.setControlPoints(std::cos(yaw) * progress, std::sin(yaw) * progress, z);
    curve.setKnots(curve.generateKnots(7, 0, 1));
    return curve;
}

// Independent support-plane test: for EACH inflated occupied prism, some cell
// plane must strictly separate the WHOLE cell. This checks volumes, not sampled
// trajectory positions, and also catches omitted obstacles beside the seed.
void verifyGeometry(const TerrainSafetyCorridor& corridor, const TerrainInterface& terrain,
                    const CorridorConfig& config) {
    require(corridor.valid && !corridor.cells.empty(), "Expected nonempty corridor: " + corridor.failureReason);
    double previous = config.contactPhase;
    for (const auto& cell : corridor.cells) {
        require(std::abs(cell.start - previous) < 1e-12 && cell.end > cell.start, "Gap/overlap in phase coverage");
        previous = cell.end;
        const auto geometric = corridorVertices(cell, false), planning = corridorVertices(cell, true);
        require(geometric.size() >= 4 && planning.size() >= 4, "Unbounded or empty polytope");
        Eigen::Vector2d lo = geometric.front().head<2>(), hi = lo;
        for (const auto& v : geometric) { lo = lo.cwiseMin(v.head<2>()); hi = hi.cwiseMax(v.head<2>()); }
        const double d = config.footRadius + config.mapMargin;
        for (const auto& obstacle : terrain.getPrisms(lo - Eigen::Vector2d::Constant(d), hi + Eigen::Vector2d::Constant(d))) {
            bool separated = false;
            for (const auto& face : cell.halfSpaces) {
                if (face.normal.z() > 1e-12) continue; // occupied prism is unbounded downwards
                double support = face.normal.z() * (obstacle.height + d);
                for (int axis = 0; axis < 2; ++axis) {
                    support += face.normal[axis] * (face.normal[axis] >= 0 ? obstacle.minXY[axis] - d : obstacle.maxXY[axis] + d);
                }
                separated |= face.geometricOffset < support - 1e-10;
            }
            require(separated, "Geometric cell overlaps an inflated occupied prism");
        }
        for (const auto& face : cell.halfSpaces) {
            require(std::abs(face.normal.norm() - 1.0) < 1e-12, "Non-unit plane");
            require(std::abs(face.geometricOffset - face.planningOffset - config.trackingMargin) < 1e-12,
                    "Tracking ball support shrink incorrect");
            for (const auto& v : planning) {
                // Worst error for THIS face. Tests the support-function contract.
                const Eigen::Vector3d actual = v + config.trackingMargin * face.normal;
                require(face.normal.dot(actual) <= face.geometricOffset + 1e-9, "Tracking tube escapes geometry");
            }
        }
    }
    require(std::abs(previous - (1.0 - config.contactPhase)) < 1e-12, "Incomplete non-contact coverage");
}

void verifyConversion(const TerrainSafetyCorridor& corridor, const BSpline& curve) {
    const auto linear = corridorControlPointConstraints(corridor);
    const auto controls = curve.getControlPoints();
    const int n = controls.rows();
    Eigen::VectorXd q(3 * n);
    q << controls.col(0), controls.col(1), controls.col(2);
    int row = 0;
    for (const auto& cell : corridor.cells) {
        Eigen::MatrixXd vertices = cell.hullMapping * controls;
        vertices.col(2).array() += corridor.footRadius;
        for (const auto& face : cell.halfSpaces) {
            for (int j = 0; j < 6; ++j, ++row) {
                const double direct = face.normal.dot(vertices.row(j).transpose()) - face.planningOffset + corridor.numericalMargin;
                require(std::abs((linear.A.row(row).dot(q) - linear.b[row]) - direct) < 1e-12,
                        "MINVO 3N row conversion or sphere-centre translation incorrect");
            }
            const Eigen::Vector3d p = vertices.row(2).transpose(), v(0.12, -0.08, 0.05), u(-0.3, 0.2, 0.7);
            const auto cbf = corridorHocbfConstraint(face, p, v, 5.0, 8.0, 0.04);
            const double h = face.geometricOffset - face.normal.dot(p);
            const double direct = -face.normal.dot(u) - 13.0 * face.normal.dot(v) + 40.0 * h - 0.04;
            require(std::abs(cbf.accelerationNormal.dot(u) - cbf.lowerBound - direct) < 1e-12,
                    "HOCBF signs/gains/disturbance conversion incorrect");
        }
    }
}

void geometryRegressions() {
    CorridorConfig config;
    config.mapMargin = 0.003;
    config.longitudinalExtent = 0.06;
    for (const std::string name : {"flat", "bar", "sides", "stairs", "negative", "diagonal", "curved", "stationary", "thin"}) {
        AnalyticTerrain terrain;
        double yaw = (name == "diagonal" || name == "curved") ? 0.65 : 0.0;
        if (name == "negative") terrain.floor = -0.6;
        if (name == "bar" || name == "negative" || name == "diagonal")
            terrain.obstacles.push_back({{0.22, -0.2}, {0.28, 0.2}, terrain.floor + 0.06});
        if (name == "sides") terrain.obstacles = {{{0.15, 0.07}, {0.35, 0.20}, 0.3}, {{0.12, -0.2}, {0.30, -0.07}, 0.3}};
        if (name == "stairs") terrain.obstacles = {{{0.22, -0.5}, {1.0, 0.5}, 0.04}, {{0.34, -0.5}, {1.0, 0.5}, 0.08}};
        if (name == "thin") terrain.obstacles = {{{0.249, -0.2}, {0.250, 0.2}, 0.06}};
        auto curve = reference(terrain.floor, name == "stairs" ? 0.08 : 0.0, yaw, name == "stationary");
        if (name == "curved") {
            auto points = curve.getControlPoints();
            points(3, 0) -= 0.08; points(3, 1) += 0.12;
            curve.setControlPoints(points.col(0), points.col(1), points.col(2));
        }
        for (auto basis : {HullBasis::Minvo, HullBasis::Bezier}) {
            const auto corridor = buildTerrainSafetyCorridor(terrain, curve, config, 4, basis);
            verifyGeometry(corridor, terrain, config);
            verifyConversion(corridor, curve);
        }
        std::cout << "PASS geometry/exports " << name << '\n';
    }
    AnalyticTerrain terrain;
    terrain.unknown = true;
    require(!buildTerrainSafetyCorridor(terrain, reference(), config).valid, "Unknown terrain accepted");
    terrain.unknown = false; terrain.invalid = true;
    require(!buildTerrainSafetyCorridor(terrain, reference(), config).valid, "NaN terrain accepted");
    terrain.invalid = false;
    {
        auto outside = reference();
        auto controls = outside.getControlPoints();
        controls.col(0).array() += 1.9;
        outside.setControlPoints(controls.col(0), controls.col(1), controls.col(2));
        require(!buildTerrainSafetyCorridor(terrain, outside, config).valid, "Map-edge corridor accepted");
    }
    {
        CorridorHalfSpace floor;
        floor.normal = -Eigen::Vector3d::UnitZ();
        floor.geometricOffset = -0.1;
        const auto cbf = corridorHocbfConstraint(floor, {0, 0, 0.1}, {0, 0, -0.2}, 5, 8, 0.04);
        require((cbf.accelerationNormal - Eigen::Vector3d::UnitZ()).norm() < 1e-12 &&
                std::abs(cbf.lowerBound - 2.64) < 1e-12,
                "Falling at a bottom boundary must require upward acceleration");
    }
    terrain.obstacles = {{{-1, -1}, {1, 1}, 10.0}};
    require(!buildTerrainSafetyCorridor(terrain, reference(), config).valid, "Empty vertical cell accepted");
    bool rejected = false;
    try { auto bad = config; bad.trackingMargin = -1; buildTerrainSafetyCorridor(terrain, reference(), bad); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Negative error budget accepted");
    std::cout << "PASS unknown/NaN/empty/config rejection\n";
}

struct Scene {
    std::shared_ptr<convex_plane_decomposition::PlanarTerrain> map;
    std::shared_ptr<TerrainInterface> terrain;
    BSpline curve;
    double duration;
    SwingTrajectoryPlanner::Config settings;
};
Scene makeScene(const std::string& name, const SwingTrajectoryPlanner::Config& settings, double duration, int swing = 0) {
    auto map = std::make_shared<convex_plane_decomposition::PlanarTerrain>();
    map->gridMap.setFrameId("map");
    // Full pillar/stair sequences follow minvo_test geometry; stair contacts
    // are shifted inward to reserve the production tracking tube.
    // Ten-centimetre bar; each displayed swing is independently optimized.
    map->gridMap.setGeometry(grid_map::Length(1.10, 0.8), 0.005, grid_map::Position(0.40, 0.0));
    map->gridMap.add("elevation");
    for (grid_map::GridMapIterator it(map->gridMap); !it.isPastEnd(); ++it) {
        grid_map::Position p;
        map->gridMap.getPosition(*it, p);
        double z = 0.0;
        if (std::abs(p.y()) <= 0.25) {
            if (name == "bar" && p.x() >= 0.20 && p.x() <= 0.23) z = 0.10;
            if (name == "pillars") {
                for (double left : {0.13, 0.31, 0.49, 0.67})
                    if (p.x() >= left && p.x() <= left + 0.10) z = 0.06;
            }
            if (name == "stairs") {
                if (p.x() >= 0.12 && p.x() <= 0.27) z = 0.10;
                if (p.x() > 0.27 && p.x() <= 0.42) z = 0.20;
                if (p.x() > 0.42 && p.x() <= 0.57) z = 0.30;
            }
        }
        map->gridMap.at("elevation", *it) = z;
    }
    auto terrain = std::make_shared<GridMapTerrainInterface>(map, "elevation");
    const std::vector<double> landings = name == "stairs" ? std::vector<double>{0.085, 0.225, 0.375, 0.525} :
        name == "pillars" ? std::vector<double>{0.105, 0.205, 0.385, 0.565, 0.745} :
        std::vector<double>{0.10, name == "bar" ? 0.40 : 0.34};
    const double startX = landings.at(swing), goalX = landings.at(swing + 1);
    const double startZ = terrain->getHeight(startX, 0.10), goalZ = terrain->getHeight(goalX, 0.10);
    const double scaling = std::min(1.0, duration / settings.swingTimeScale);
    // Production nominal swing BEFORE terrain-adaptive peak/QP correction.
    auto curve = createSwingHeightBSpline(0, duration, startZ, scaling * settings.liftOffVelocity,
        std::max(startZ, goalZ) + scaling * settings.swingHeight, goalZ, scaling * settings.touchDownVelocity);
    const Eigen::VectorXd progress = (Eigen::VectorXd(7) << 0, 0, 0, 0.5, 1, 1, 1).finished();
    const Eigen::VectorXd z = curve.getControlPoints().col(2);
    curve.setControlPoints(Eigen::VectorXd::Constant(7, startX) + progress * (goalX - startX),
                          Eigen::VectorXd::Constant(7, 0.10), z);
    return {map, terrain, curve, duration, settings};
}

double physicalClearance(const Scene& scene, const BSpline& curve, double time, double radius) {
    const Eigen::Vector3d p = curve.getPosition(time);
    return p.z() - scene.terrain->getSphereClearanceUpperBound(p.head<2>(), p.head<2>(), radius);
}

void exportScene(const std::filesystem::path& dir, const std::string& name, const Scene& scene,
                 const TerrainSafetyCorridor& corridor, const BSpline& optimized) {
    std::filesystem::create_directories(dir);
    std::ofstream planes(dir / (name + "_planes.csv")), vertices(dir / (name + "_vertices.csv")),
        trajectory(dir / (name + "_trajectory.csv")), terrain(dir / (name + "_terrain.csv"));
    for (auto* out : {&planes, &vertices, &trajectory, &terrain}) *out << std::setprecision(17);
    planes << "cell,start,end,ax,ay,az,b_geom,b_plan,b_qp\n";
    vertices << "cell,type,x,y,z\n";
    trajectory << "phase,ref_x,ref_y,ref_z,opt_x,opt_y,opt_z,time,ref_clearance,opt_clearance\n";
    terrain << "x,y,z\n";
    for (size_t i = 0; i < corridor.cells.size(); ++i) {
        const auto& cell = corridor.cells[i];
        for (const auto& p : cell.halfSpaces)
            planes << i << ',' << cell.start << ',' << cell.end << ',' << p.normal.x() << ',' << p.normal.y() << ',' << p.normal.z()
                   << ',' << p.geometricOffset << ',' << p.planningOffset << ',' << p.planningOffset - corridor.numericalMargin << '\n';
        for (bool planning : {false, true}) {
            for (const auto& v : corridorVertices(cell, planning))
                vertices << i << ',' << (planning ? "plan" : "geom") << ',' << v.x() << ',' << v.y() << ',' << v.z() << '\n';
        }
    }
    for (int i = 0; i <= 2000; ++i) {
        const double phase = i / 2000.0, t = phase * scene.duration;
        const Eigen::Vector3d ref = scene.curve.getPosition(t) + Eigen::Vector3d(0, 0, corridor.footRadius);
        const Eigen::Vector3d opt = optimized.getPosition(t) + Eigen::Vector3d(0, 0, corridor.footRadius);
        trajectory << phase << ',' << ref.x() << ',' << ref.y() << ',' << ref.z() << ',' << opt.x() << ',' << opt.y() << ',' << opt.z() << ',' << t << ',' << physicalClearance(scene, scene.curve, t, corridor.footRadius) << ',' << physicalClearance(scene, optimized, t, corridor.footRadius) << '\n';
    }
    for (grid_map::GridMapIterator it(scene.map->gridMap); !it.isPastEnd(); ++it) {
        grid_map::Position p; scene.map->gridMap.getPosition(*it, p);
        terrain << p.x() << ',' << p.y() << ',' << scene.map->gridMap.at("elevation", *it) << '\n';
    }
}

visualization_msgs::msg::MarkerArray sceneMarkers(
    const Scene& scene, const SwingTrajectoryQpOptimizer::OptimizedTrajectory& result,
    const std_msgs::msg::Header& header) {
    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray markers;
    Marker clear; clear.header = header; clear.action = Marker::DELETEALL;
    markers.markers.push_back(clear);
    // Palette shared with plot_corridor_sections.py. Colours identify boundary
    // semantics, never time-cell indices. Display uses contact-reference coordinates.
    const std::array<std::array<float, 3>, 7> colors{{
        {{0.82f, 0.54f, 0.16f}}, {{0.45f, 0.47f, 0.49f}}, {{0.0f, 0.55f, 0.20f}},
        {{0.50f, 0.39f, 0.64f}}, {{0.35f, 0.66f, 0.71f}}, {{0.78f, 0.26f, 0.26f}}, {{0.15f, 0.46f, 0.60f}}
    }};
    auto marker = [&](const std::string& ns, int type, int color, double width) {
        Marker m; m.header = header; m.ns = ns; m.id = static_cast<int>(markers.markers.size());
        m.type = type; m.pose.orientation.w = 1; m.scale.x = width;
        m.color.r = colors[color][0]; m.color.g = colors[color][1]; m.color.b = colors[color][2]; m.color.a = 1;
        return m;
    };
    auto point = [&](const Eigen::Vector3d& p) {
        geometry_msgs::msg::Point v; v.x = p.x(); v.y = p.y(); v.z = p.z() - result.corridor.footRadius; return v;
    };
    for (const auto& cell : result.corridor.cells) {
        for (bool planning : {false, true}) {
            const auto vertices = corridorVertices(cell, planning);
            for (const auto& plane : cell.halfSpaces) {
                const double offset = planning ? plane.planningOffset : plane.geometricOffset;
                std::vector<Eigen::Vector3d> face;
                for (const auto& v : vertices)
                    if (std::abs(plane.normal.dot(v) - offset) < 1e-8) face.push_back(v);
                if (face.size() < 3) continue; // redundant plane has no polygon face
                Eigen::Vector3d centre = Eigen::Vector3d::Zero();
                for (const auto& v : face) centre += v;
                centre /= face.size();
                const Eigen::Vector3d u = plane.normal.unitOrthogonal(), v = plane.normal.cross(u);
                std::sort(face.begin(), face.end(), [&](const auto& a, const auto& b) {
                    return std::atan2((a-centre).dot(v), (a-centre).dot(u)) <
                           std::atan2((b-centre).dot(v), (b-centre).dot(u));
                });
                const int color = plane.normal.z() < -0.5 ? (planning ? 2 : 0) :
                                  plane.normal.z() > 0.5 ? (planning ? 3 : 1) : (planning ? 4 : 1);
                const bool lower = plane.normal.z() < -0.5;
                const bool upper = plane.normal.z() > 0.5;
                const std::string boundary = lower ? "lower" : (upper ? "upper" : "sides");
                // Uniform wire width matches the purple planning upper face.
                const double width = 0.0015;
                auto wire = marker((planning ? "planning_" : "geometric_") + boundary,
                                   Marker::LINE_STRIP, color, width);
                wire.color.a = planning ? (lower ? 1.0 : (upper ? 0.75 : 0.20)) :
                                         (lower ? 0.65 : 0.18);
                for (const auto& p : face) wire.points.push_back(point(p));
                wire.points.push_back(point(face.front()));
                markers.markers.push_back(wire);
            }
        }
    }
    auto nominal = marker("nominal_contact_reference", Marker::LINE_LIST, 5, 0.003);
    auto optimized = marker("optimized_contact_reference", Marker::LINE_STRIP, 6, 0.004);
    auto collision = marker("nominal_collision_samples", Marker::SPHERE_LIST, 5, 0.005);
    collision.scale.y = collision.scale.z = collision.scale.x;
    for (int i = 0; i <= 400; ++i) {
        const double t = scene.duration * i / 400.0;
        const Eigen::Vector3d shift(0, 0, result.corridor.footRadius);
        const Eigen::Vector3d ref = scene.curve.getPosition(t) + shift;
        optimized.points.push_back(point(result.bspline->getPosition(t) + shift));
        if (i < 400 && (i / 6) % 2 == 0) {
            nominal.points.push_back(point(ref));
            nominal.points.push_back(point(scene.curve.getPosition(scene.duration * (i+1) / 400.0) + shift));
        }
        if (i % 8 == 0 && physicalClearance(scene, scene.curve, t, result.corridor.footRadius) < -1e-8)
            collision.points.push_back(point(ref));
    }
    markers.markers.push_back(nominal); markers.markers.push_back(optimized); markers.markers.push_back(collision);
    return markers;
}

SwingTrajectoryQpOptimizer::OptimizedTrajectory runPipeline(const Scene& scene, const std::string& name,
                                                          const std::filesystem::path& output) {
    auto config = scene.settings.terrainQpConfig;
    config.safetyCorridorEnabled = true;
    config.geometryMode = SwingGeometryMode::Minvo;
    config.footRadius = scene.settings.footRadius;
    config.corridorConfig.footRadius = config.footRadius;
    // A 10 cm obstacle exceeds the nominal peak: allow enough local vertical
    // search space, while keeping the production foot/tracking safety margins.
    if (name.rfind("bar", 0) == 0) {
        config.corridorConfig.verticalExtent = 0.14;
        config.maxCorrection = 0.20;
    }
    config.checkReference = false;
    config.verifyOutput = true;
    config.verbose = false;
    const auto begin = std::chrono::steady_clock::now();
    SwingTrajectoryQpOptimizer qp(config, scene.terrain);
    const auto result = qp.optimizeSwingTrajectory(scene.curve, 0.0, scene.duration,
        scene.curve.getPosition(0), scene.curve.getPosition(scene.duration));
    const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - begin).count();
    require(result.valid(), name + ": MINVO pipeline failed (" + result.corridor.failureReason + ")");
    require(result.corridorCertified && result.corridorMaxViolation <= 1e-9, name + ": corridor output not certified");
    verifyGeometry(result.corridor, *scene.terrain, config.corridorConfig);
    // Independent dense check includes exact shared boundaries and all directions.
    double worst = -1e9;
    for (const auto& cell : result.corridor.cells) {
        for (int i = 0; i <= 50; ++i) {
            const double t = cell.start + (cell.end - cell.start) * i / 50.0;
            const Eigen::Vector3d p = result.bspline->getPosition(t * scene.duration) + Eigen::Vector3d(0, 0, config.footRadius);
            for (const auto& plane : cell.halfSpaces) worst = std::max(worst, plane.normal.dot(p) - plane.planningOffset);
        }
    }
    require(worst <= 1e-9, name + ": optimized trajectory escapes planning polytope");
    double minReference = 1e9, minOptimized = 1e9;
    for (int i = 0; i <= 2000; ++i) {
        const double t = scene.duration * i / 2000.0;
        minReference = std::min(minReference, physicalClearance(scene, scene.curve, t, config.footRadius));
        minOptimized = std::min(minOptimized, physicalClearance(scene, *result.bspline, t, config.footRadius));
    }
    require(minOptimized >= -1e-8, name + ": optimized sphere intersects terrain");
    if (name.rfind("flat", 0) != 0 && name.find("_swing0") != std::string::npos) {
        require(minReference < -1e-4, name + ": nominal swing never collided with terrain");
        require(result.correctionCost > 1e-10, name + ": never exercised a QP correction");
    }
    std::cout << "  reference_min_clearance=" << minReference << " optimized_min_clearance=" << minOptimized
              << " duration=" << scene.duration << '\n';
    for (int derivative = 1; derivative <= 2; ++derivative) {
        for (double t : {0.0, scene.duration}) {
            const Eigen::VectorXd phase = Eigen::VectorXd::Constant(1, t);
            const Eigen::MatrixXd a = result.bspline->generateSplineDerivativeBasis(phase, derivative) * result.bspline->getControlPoints();
            const Eigen::MatrixXd b = scene.curve.generateSplineDerivativeBasis(phase, derivative) * scene.curve.getControlPoints();
            require((a - b).norm() < 1e-8, "Changed contact velocity/acceleration");
        }
    }
    const auto start = scene.curve.getPosition(0), end = scene.curve.getPosition(scene.duration);
    require((result.bspline->getPosition(0) - start).norm() < 1e-10 &&
            (result.bspline->getPosition(scene.duration) - end).norm() < 1e-10, "Changed contact positions");
    std::cout << "PASS pipeline " << name << " cells=" << result.corridor.cells.size()
              << " rows=" << result.constraintRows << " hull_residual=" << result.corridorMaxViolation
              << " dense_residual=" << worst << " elapsed_ms=" << ms << '\n';
    std_msgs::msg::Header header; header.frame_id = "map";
    const auto markerStart = std::chrono::steady_clock::now();
    const auto markers = terrainCorridorMarkers({result.corridor, result.corridor, result.corridor, result.corridor}, header);
    const double markerMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - markerStart).count();
    require(markers.markers.size() > 1, "3D corridor visualization is empty");
    std::cout << "  four_leg_markers=" << markers.markers.size() << " marker_ms=" << markerMs << '\n';
    if (!output.empty()) {
        std::filesystem::create_directories(output);
        std::ofstream metrics(output / (name + "_metrics.csv"));
        metrics << std::setprecision(17)
                << "schema,duration,foot_radius,swing_height,lift_off_velocity,touch_down_velocity,contact_phase,reference_min_clearance,optimized_min_clearance,corridor_max_violation,correction_cost,output_certificate,vertical_extent,max_correction\n"
                << "2," << scene.duration << ',' << config.footRadius << ',' << scene.settings.swingHeight
                << ',' << scene.settings.liftOffVelocity << ',' << scene.settings.touchDownVelocity
                << ',' << config.corridorConfig.contactPhase << ',' << minReference << ',' << minOptimized
                << ',' << result.corridorMaxViolation << ',' << result.correctionCost << ',' << static_cast<int>(result.certificate.status) << ',' << config.corridorConfig.verticalExtent << ',' << config.maxCorrection << '\n';
    }
    if (!output.empty()) exportScene(output, name, scene, result.corridor, *result.bspline);
    return result;
}
}  // namespace

int main(int argc, char** argv) {
    try {
        std::string selected = "all";
        std::filesystem::path output;
        bool publish = false;
        const auto robotShare = ament_index_cpp::get_package_share_directory("cyberdog_description");
        std::string taskFile = robotShare + "/config/ocs2/task.info";
        std::string gaitFile = robotShare + "/config/ocs2/gait.info";
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--scene" && i + 1 < argc) selected = argv[++i];
            else if (arg == "--output" && i + 1 < argc) output = argv[++i];
            else if (arg == "--task" && i + 1 < argc) taskFile = argv[++i];
            else if (arg == "--gait" && i + 1 < argc) gaitFile = argv[++i];
            else if (arg == "--publish") publish = true;
            else if (arg == "--verify-only") {}
            else if (arg == "--ros-args") break;
            else throw std::invalid_argument("Unknown argument: " + arg);
        }
        require(selected == "all" || selected == "flat" || selected == "bar" || selected == "pillars" || selected == "stairs", "Unknown scene");
        const auto settings = loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", false);
        boost::property_tree::ptree gait;
        boost::property_tree::read_info(gaitFile, gait);
        const auto& times = gait.get_child("trot.switchingTimes");
        require(times.size() >= 2, "Trot switching times require two entries");
        auto time = times.begin();
        const double first = (time++)->second.get_value<double>();
        const double duration = time->second.get_value<double>() - first;
        require(duration > 0 && settings.swingTimeScale > 0, "Invalid swing duration/time scale");
        std::cout << "Robot task=" << taskFile << " gait=" << gaitFile << " trot swing=" << duration << "s\n";
        geometryRegressions();
        {
            auto terrain = std::make_shared<AnalyticTerrain>();
            terrain->unknown = true;
            SwingTrajectoryQpConfig config; config.safetyCorridorEnabled = true; config.checkReference = false;
            const auto curve = reference();
            const auto result = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
                curve, 0, 1, curve.getPosition(0), curve.getPosition(1));
            require(!result.valid() && !result.corridorCertified, "Unknown terrain produced an accepted trajectory");
            terrain->unknown = false;
            config.corridorConfig.contactPhase = 0.0;
            const auto fixed = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
                curve, 0, 1, curve.getPosition(0), curve.getPosition(1));
            require(!fixed.valid() && !fixed.corridorCertified,
                    "Infeasible fixed contact corridor row was silently dropped");
            config.corridorConfig.contactPhase = 0.1;
            terrain->obstacles = {{{-1, -1}, {1, 1}, 10.0}};
            const auto blocked = SwingTrajectoryQpOptimizer(config, terrain).optimizeSwingTrajectory(
                curve, 0, 1, curve.getPosition(0), curve.getPosition(1));
            require(!blocked.valid() && !blocked.corridorCertified, "Blocked corridor produced an accepted trajectory");
        }
        for (const std::string name : {"flat", "bar", "pillars", "stairs"}) {
            if (selected != "all" && selected != name) continue;
            const int count = name == "pillars" ? 4 : (name == "stairs" ? 3 : 1);
            std::vector<Scene> scenes;
            std::vector<SwingTrajectoryQpOptimizer::OptimizedTrajectory> results;
            for (int swing = 0; swing < count; ++swing) {
                scenes.push_back(makeScene(name, settings, duration, swing));
                if (swing) require((scenes[swing - 1].curve.getPosition(duration) -
                                    scenes[swing].curve.getPosition(0)).norm() < 1e-10,
                                   name + ": discontinuous support positions between swings");
                results.push_back(runPipeline(scenes.back(), name + "_swing" + std::to_string(swing), output));
            }
            if (!output.empty()) {
                std::ofstream manifest(output / (name + "_swings.csv"));
                manifest << "swing,stem\n";
                for (int swing = 0; swing < count; ++swing)
                    manifest << swing << ',' << name << "_swing" << swing << '\n';
            }
            if (publish) {
                require(selected != "all", "--publish requires one --scene");
                rclcpp::init(argc, argv);
                auto node = std::make_shared<rclcpp::Node>("terrain_corridor_scene");
                const auto qos = rclcpp::QoS(1).transient_local().reliable();
                auto mapPub = node->create_publisher<grid_map_msgs::msg::GridMap>("/corridor_test/elevation_map", qos);
                auto markerPub = node->create_publisher<visualization_msgs::msg::MarkerArray>("/corridor_test/markers", qos);
                auto timer = node->create_wall_timer(std::chrono::milliseconds(500), [&] {
                    scenes.front().map->gridMap.setTimestamp(node->now().nanoseconds());
                    mapPub->publish(*grid_map::GridMapRosConverter::toMessage(scenes.front().map->gridMap));
                    std_msgs::msg::Header header; header.frame_id = "map"; header.stamp = node->now();
                    visualization_msgs::msg::MarkerArray markers;
                    for (size_t swing = 0; swing < scenes.size(); ++swing) {
                        auto part = sceneMarkers(scenes[swing], results[swing], header);
                        for (auto& marker : part.markers) {
                            if (swing && (marker.action == visualization_msgs::msg::Marker::DELETEALL)) continue;
                            if (marker.action != visualization_msgs::msg::Marker::DELETEALL)
                                marker.ns += "_swing" + std::to_string(swing);
                            markers.markers.push_back(std::move(marker));
                        }
                    }
                    markerPub->publish(markers);
                });
                RCLCPP_INFO(node->get_logger(), "Publishing semantic corridor boundaries and complete nominal/MINVO contact-reference sequences (%s)", name.c_str());
                rclcpp::spin(node);
                rclcpp::shutdown();
            }
        }
        std::cout << "All terrain corridor checks passed.\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
