#include <ocs2_quadruped_controller/interface/constraint/SwingTrajectoryPlanner.h>
#include <ocs2_quadruped_controller/interface/constraint/SwingTrajectoryQPOptimizer.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <grid_map_core/GridMapMath.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <iomanip>
#include <iostream>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace ocs2::legged_robot;
namespace {
Eigen::Vector3d vector3(const YAML::Node& value) {
    if (!value.IsSequence() || value.size() != 3) throw std::invalid_argument("Expected XYZ array");
    return {value[0].as<double>(), value[1].as<double>(), value[2].as<double>()};
}
struct Box {
    double x0, x1, y0, y1, z;
};
Box box(const YAML::Node& row) {
    if (row.size() != 5) throw std::invalid_argument("Box needs [xmin,xmax,ymin,ymax,height]");
    Box b{row[0].as<double>(), row[1].as<double>(), row[2].as<double>(), row[3].as<double>(),
          row[4].as<double>()};
    if (b.x1 <= b.x0 || b.y1 <= b.y0 || b.z < 0) throw std::invalid_argument("Invalid box dimensions");
    return b;
}
// Test-only policy: virtual occupied columns encode the explicitly requested crossing floor.
// Physical output is independently certified against the unmodified elevation map.
class CrossingTerrain final : public TerrainInterface {
   public:
    CrossingTerrain(std::shared_ptr<TerrainInterface> terrain, std::vector<Box> bands)
        : terrain_(std::move(terrain)), bands_(std::move(bands)) {}
    double getHeight(double x, double y) const override {
        double h = terrain_->getHeight(x, y);
        for (const auto& b : bands_)
            if (x >= b.x0 && x <= b.x1 && y >= b.y0 && y <= b.y1) h = std::max(h, b.z);
        return h;
    }
    std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d& lo, const Eigen::Vector2d& hi) const override {
        auto result = terrain_->getPrisms(lo, hi);
        for (const auto& b : bands_)
            if (hi.x() >= b.x0 && lo.x() <= b.x1 && hi.y() >= b.y0 && lo.y() <= b.y1)
                result.push_back({{b.x0, b.y0}, {b.x1, b.y1}, b.z});
        return result;
    }

   private:
    std::shared_ptr<TerrainInterface> terrain_;
    std::vector<Box> bands_;
};
visualization_msgs::msg::Marker marker(const std::string& ns, int id, int type, const std::string& frame) {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = m.ADD;
    m.pose.orientation.w = 1;
    m.color.a = 1;
    m.scale.x = .004;
    return m;
}
geometry_msgs::msg::Point point(const Eigen::Vector3d& p) {
    geometry_msgs::msg::Point value;
    value.x = p.x();
    value.y = p.y();
    value.z = p.z();
    return value;
}
}  // namespace
int main(int argc, char** argv) {
    try {
        if (argc < 3)
            throw std::invalid_argument("Usage: minvo_scene CONFIG.yaml DATA_DIRECTORY [--export-only]");
        const auto config = YAML::LoadFile(argv[1]);
        const bool exportOnly = argc == 4 && std::string(argv[3]) == "--export-only";
        const std::filesystem::path out(argv[2]);
        std::filesystem::create_directories(out);
        std::filesystem::copy_file(argv[1], out / "config.yaml",
                                   std::filesystem::copy_options::overwrite_existing);
        const std::string frame = config["frame"].as<std::string>();
        const auto mapConfig = config["map"];
        auto mapData = std::make_shared<convex_plane_decomposition::PlanarTerrain>();
        auto& map = mapData->gridMap;
        map.setFrameId(frame);
        map.setGeometry({mapConfig["length"][0].as<double>(), mapConfig["length"][1].as<double>()},
                        mapConfig["resolution"].as<double>(),
                        {mapConfig["center"][0].as<double>(), mapConfig["center"][1].as<double>()});
        map.add("elevation", 0.0);
        map.add("color", 0.0);
        const double colorMin = config["visualization"]["height_range"][0].as<double>();
        const double colorMax = config["visualization"]["height_range"][1].as<double>();
        std::vector<Eigen::Vector3f> colors;
        for (const auto& rgb : config["visualization"]["colors"])
            colors.push_back(vector3(rgb).cast<float>() / 255.0f);
        if (colorMax <= colorMin || colors.size() < 2)
            throw std::invalid_argument("Invalid height color scale");
        std::vector<Box> boxes;
        for (const auto& row : config["obstacles"]) boxes.push_back(box(row));
        std::ofstream mapCsv(out / "map.csv");
        mapCsv << "x,y,z\n" << std::setprecision(12);
        for (grid_map::Index index(0, 0); index[0] < map.getSize()[0]; ++index[0]) {
            for (index[1] = 0; index[1] < map.getSize()[1]; ++index[1]) {
                grid_map::Position xy;
                map.getPosition(index, xy);
                double h = 0;
                for (const auto& b : boxes)
                    if (xy.x() >= b.x0 && xy.x() <= b.x1 && xy.y() >= b.y0 && xy.y() <= b.y1)
                        h = std::max(h, b.z);
                map.at("elevation", index) = h;
                const double scaled =
                    std::clamp((h - colorMin) / (colorMax - colorMin), 0.0, 1.0) * (colors.size() - 1);
                const auto stop = std::min(static_cast<size_t>(scaled), colors.size() - 2);
                const float fraction = scaled - stop;
                const Eigen::Vector3f rgb = (1 - fraction) * colors[stop] + fraction * colors[stop + 1];
                grid_map::colorVectorToValue(rgb, map.at("color", index));
                mapCsv << xy.x() << ',' << xy.y() << ',' << map.at("elevation", index) << '\n';
            }
        }
        auto physical = std::make_shared<GridMapTerrainInterface>(mapData, "elevation");
        std::vector<Box> bands;
        for (const auto& row : config["crossing_bands"]) bands.push_back(box(row));
        auto crossing = std::make_shared<CrossingTerrain>(physical, bands);
        const double radius = config["foot_radius"].as<double>();
        std::ofstream curves(out / "trajectories.csv"), hulls(out / "hulls.csv"),
            metrics(out / "metrics.csv"), splineControls(out / "spline_controls.csv"),
            knots(out / "knots.csv");
        curves << "swing,method,t,x,y,z,vz,az,surface_z,required_z\n" << std::setprecision(12);
        hulls << "swing,method,basis,span,start,end,vertex,x,y,z\n" << std::setprecision(12);
        metrics << "swing,method,valid,certificate,min_clearance,reference_clearance,endpoint_error,"
                   "objective,planning_ms,gap_min_bottom,crossing_violation,reference_certificate\n"
                << std::setprecision(12);
        splineControls << "swing,method,index,x,y,z_minus_radius\n" << std::setprecision(12);
        knots << "swing,method,index,knot\n" << std::setprecision(12);
        std::ofstream smoothness(out / "smoothness.csv");
        smoothness << "swing,method,min_internal_continuity,max_speed,max_acceleration,"
                      "max_position_gap,max_velocity_gap,max_acceleration_gap,"
                      "lift_off_vz,touch_down_vz,lift_off_acceleration,touch_down_acceleration\n"
                   << std::setprecision(12);
        visualization_msgs::msg::MarkerArray markers;
        auto reset = marker("reset", 0, visualization_msgs::msg::Marker::LINE_STRIP, frame);
        reset.action = reset.DELETEALL;
        markers.markers.push_back(reset);
        int swingId = 0;
        bool allValid = true;
        for (const auto& swing : config["swings"]) {
            auto start = vector3(swing["start"]), goal = vector3(swing["goal"]);
            // Endpoints are contacts, so snap height to the actual support surface.
            start.z() = physical->getHeight(start.x(), start.y());
            goal.z() = physical->getHeight(goal.x(), goal.y());
            // Contact must fit on the represented support surface, including the sphere footprint.
            for (const auto& contact : {start, goal}) {
                if (contact.z() + 1e-7 <
                    physical->getSphereClearanceUpperBound(contact.head<2>(), contact.head<2>(), radius))
                    throw std::invalid_argument("Contact does not fit its support plane");
            }
            const double duration = swing["duration"].as<double>();
            if (duration <= 0 || !start.allFinite() || !goal.allFinite())
                throw std::invalid_argument("Invalid swing");
            const auto refConfig = config["reference"];
            if (refConfig["swing_time_scale"].as<double>() <= 0 ||
                refConfig["swing_height"].as<double>() < 0)
                throw std::invalid_argument("Invalid swing reference height/time scale");
            const double scale = std::min(1.0, duration / refConfig["swing_time_scale"].as<double>());
            const double midHeight = std::max(start.z(), goal.z()) +
                                     scale * refConfig["swing_height"].as<double>();
            BSpline reference = createSwingHeightBSpline(
                0, duration, start.z(), scale * refConfig["lift_off_velocity"].as<double>(),
                midHeight, goal.z(), scale * refConfig["touch_down_velocity"].as<double>());
            // The online height reference has zero XY; embed it along this foot's swing.
            auto controls = reference.getControlPoints();
            for (int j = 0; j < controls.rows(); ++j) {
                const double u = j < 3 ? 0.0 : j > 3 ? 1.0 : 0.5;
                controls.row(j).head<2>() = ((1 - u) * start + u * goal).head<2>().transpose();
            }
            reference.setControlPoints(controls.col(0), controls.col(1), controls.col(2));
            if (std::abs(reference.getPosition(duration / 2).z() - midHeight) > 1e-9 ||
                (reference.getPosition(0) - start).norm() > 1e-9 ||
                (reference.getPosition(duration) - goal).norm() > 1e-9)
                throw std::runtime_error("Swing reference midpoint/contact mismatch");
            double referenceClearance = std::numeric_limits<double>::infinity();
            for (int i = 0; i <= 1000; ++i) {
                const auto p = reference.getPosition(duration * i / 1000.0);
                referenceClearance = std::min(referenceClearance, p.z() - physical->getSphereClearanceUpperBound(
                    p.head<2>(), p.head<2>(), radius));
            }
            const auto referenceCertificate = SwingCollisionCertifier(physical).check(reference, radius);
            auto exportCurve = [&](const BSpline& spline, const std::string& method) {
                double maxSpeed = 0, maxAcceleration = 0;
                for (int i = 0; i <= 4000; ++i) {
                    const double t = duration * i / 4000.0;
                    maxSpeed = std::max(maxSpeed, spline.getVelocity(t).norm());
                    maxAcceleration = std::max(maxAcceleration, spline.getAcceleration(t).norm());
                }
                int continuity = 5;
                double positionGap = 0, velocityGap = 0, accelerationGap = 0;
                const auto& ks = spline.getKnots();
                for (int i = 0; i < ks.size();) {
                    int j = i + 1;
                    while (j < ks.size() && ks[j] == ks[i]) ++j;
                    if (ks[i] > 0 && ks[i] < duration) {
                        continuity = std::min(continuity, 5 - (j - i));
                        const double eps = duration * 1e-9;
                        positionGap = std::max(positionGap,
                            (spline.getPosition(ks[i] + eps) - spline.getPosition(ks[i] - eps)).norm());
                        velocityGap = std::max(velocityGap,
                            (spline.getVelocity(ks[i] + eps) - spline.getVelocity(ks[i] - eps)).norm());
                        accelerationGap = std::max(accelerationGap,
                            (spline.getAcceleration(ks[i] + eps) - spline.getAcceleration(ks[i] - eps)).norm());
                    }
                    i = j;
                }
                smoothness << swingId << ',' << method << ',' << continuity << ',' << maxSpeed << ','
                           << maxAcceleration << ',' << positionGap << ',' << velocityGap << ','
                           << accelerationGap << ',' << spline.getVelocity(0).z() << ','
                           << spline.getVelocity(duration).z() << ',' << spline.getAcceleration(0).norm()
                           << ',' << spline.getAcceleration(duration).norm() << '\n';
                if (continuity < 2 || positionGap > 1e-6 || velocityGap > 1e-5 || accelerationGap > 1e-3)
                    throw std::runtime_error("Swing spline failed C2 continuity check");
                auto line = marker(method, swingId, visualization_msgs::msg::Marker::LINE_STRIP, frame);
                line.color.r = method == "reference" ? .95 : method == "Bezier" ? 1.0 : .05;
                line.color.g = method == "reference" ? .08 : method == "Bezier" ? .55 : .75;
                line.color.b = method == "MINVO" ? .12 : method == "MINVO_geometry" ? .9 : .05;
                if (method == "MINVO_geometry") {
                    line.color.r = .1;
                    line.color.g = .45;
                }
                for (int i = 0; i <= 600; ++i) {
                    double t = duration * i / 600.0;
                    auto p = spline.getPosition(t);
                    curves << swingId << ',' << method << ',' << t << ',' << p.x() << ',' << p.y() << ','
                           << p.z() << ',' << spline.getVelocity(t).z() << ','
                           << spline.getAcceleration(t).z() << ',' << physical->getHeight(p.x(), p.y()) << ','
                           << physical->getSphereClearanceUpperBound(p.head<2>(), p.head<2>(), radius)
                           << '\n';
                    if (method != "reference" || (i / 8) % 2 == 0) {
                        line.points.push_back(point(p));
                        if (method == "reference" && i < 600 && (i + 1) / 8 % 2 != 0) {
                            markers.markers.push_back(line);
                            line.id += 100;
                            line.points.clear();
                        }
                    }
                }
                markers.markers.push_back(line);
                auto vertices = spline.getControlPoints();
                for (int i = 0; i < vertices.rows(); ++i)
                    splineControls << swingId << ',' << method << ',' << i << ',' << vertices(i, 0) << ','
                                   << vertices(i, 1) << ',' << vertices(i, 2) << '\n';
                for (int i = 0; i < spline.getKnots().size(); ++i)
                    knots << swingId << ',' << method << ',' << i << ',' << spline.getKnots()[i] << '\n';
                int spanId = 0;
                for (const auto& segment : extractPolynomialSegments(spline)) {
                    for (auto basis : {HullBasis::Bezier, HullBasis::Minvo}) {
                        Eigen::MatrixXd points = hullMap(segment, basis) * vertices;
                        for (int j = 0; j < points.rows(); ++j)
                            hulls << swingId << ',' << method << ','
                                  << (basis == HullBasis::Bezier ? "Bezier" : "MINVO") << ',' << spanId << ','
                                  << segment.start << ',' << segment.end << ',' << j << ',' << points(j, 0)
                                  << ',' << points(j, 1) << ',' << points(j, 2) << '\n';
                    }
                    ++spanId;
                }
            };
            exportCurve(reference, "reference");
            for (const auto& method : bands.empty()
                                          ? std::vector<std::string>{"Bezier", "MINVO"}
                                          : std::vector<std::string>{"Bezier", "MINVO_geometry", "MINVO"}) {
                SwingTrajectoryQpConfig settings;
                settings.geometryMode =
                    method == "Bezier" ? SwingGeometryMode::Bezier : SwingGeometryMode::Minvo;
                settings.numCurveSamples = config["optimizer"]["samples"].as<int>();
                settings.numControlPoints = config["optimizer"]["control_points"].as<int>();
                settings.groundClearance = config["optimizer"]["clearance"].as<double>();
                settings.subdivisionsPerSpan = config["optimizer"]["subdivisions"].as<int>();
                settings.footRadius = radius;
                auto model = method == "MINVO_geometry" ? std::shared_ptr<TerrainInterface>(physical)
                                                        : std::shared_ptr<TerrainInterface>(crossing);
                SwingTrajectoryQpOptimizer optimizer(settings, model);
                auto before = std::chrono::steady_clock::now();
                auto result = optimizer.optimizeSwingTrajectory(reference, 0, duration, start, goal);
                const double ms =
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - before)
                        .count();
                double clearance = NAN, error = NAN, gapBottom = std::numeric_limits<double>::infinity(),
                       crossingViolation = 0;
                CollisionCertificate cert;
                if (result.valid()) {
                    cert = SwingCollisionCertifier(physical).check(*result.bspline, radius);
                    clearance = std::numeric_limits<double>::infinity();
                    for (int i = 0; i <= 2000; ++i) {
                        auto p = result.bspline->getPosition(duration * i / 2000.0);
                        clearance = std::min(clearance, p.z() - physical->getSphereClearanceUpperBound(
                                                                    p.head<2>(), p.head<2>(), radius));
                        for (const auto& band : bands) {
                            if (p.x() >= band.x0 && p.x() <= band.x1 && p.y() >= band.y0 &&
                                p.y() <= band.y1) {
                                gapBottom = std::min(gapBottom, p.z());
                                crossingViolation = std::max(crossingViolation, band.z - p.z());
                            }
                        }
                    }
                    error = std::max((result.bspline->getPosition(0) - start).norm(),
                                     (result.bspline->getPosition(duration) - goal).norm());
                    exportCurve(*result.bspline, method);
                }
                bool valid = result.valid() && cert.status == CollisionStatus::ProvenSafe &&
                             (method == "MINVO_geometry" || crossingViolation <= 1e-7);
                allValid &= valid;
                metrics << swingId << ',' << method << ',' << valid << ',' << static_cast<int>(cert.status)
                        << ',' << clearance << ',' << referenceClearance << ',' << error << ','
                        << result.correctionCost << ',' << ms << ','
                        << (std::isfinite(gapBottom) ? gapBottom : NAN) << ',' << crossingViolation << ','
                        << static_cast<int>(referenceCertificate.status) << '\n';
                std::cout << "swing " << swingId << ' ' << method << " valid=" << valid
                          << " reference clearance=" << referenceClearance << '\n';
            }
            for (int endpoint = 0; endpoint < 2; ++endpoint) {
                auto m = marker("contacts", 2 * swingId + endpoint, visualization_msgs::msg::Marker::SPHERE,
                                frame);
                auto p = endpoint ? goal : start;
                m.pose.position = point(p);
                m.scale.x = m.scale.y = m.scale.z = .014;
                m.color.r = m.color.g = m.color.b = .08;
                markers.markers.push_back(m);
            }
            ++swingId;
        }
        mapCsv.close();
        curves.close();
        hulls.close();
        metrics.close();
        splineControls.close();
        knots.close();
        if (!allValid) throw std::runtime_error("One or more plans failed; inspect metrics.csv");
        if (exportOnly) return 0;
        rclcpp::init(0, nullptr);
        auto node = std::make_shared<rclcpp::Node>("minvo_scene");
        auto qos = rclcpp::QoS(1).transient_local().reliable();
        auto mapPub = node->create_publisher<grid_map_msgs::msg::GridMap>("/minvo_scene/elevation_map", qos);
        auto markerPub =
            node->create_publisher<visualization_msgs::msg::MarkerArray>("/minvo_scene/trajectories", qos);
        auto timer = node->create_wall_timer(std::chrono::milliseconds(500), [&] {
            map.setTimestamp(node->now().nanoseconds());
            mapPub->publish(*grid_map::GridMapRosConverter::toMessage(map));
            for (auto& m : markers.markers) m.header.stamp = node->now();
            markerPub->publish(markers);
        });
        rclcpp::spin(node);
        rclcpp::shutdown();
    } catch (const std::exception& e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
    return 0;
}
