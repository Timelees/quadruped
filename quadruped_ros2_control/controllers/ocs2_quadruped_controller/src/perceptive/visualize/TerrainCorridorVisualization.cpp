#include "ocs2_quadruped_controller/perceptive/visualize/TerrainCorridorVisualization.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>

namespace ocs2::legged_robot {
namespace {
geometry_msgs::msg::Point toPoint(const Eigen::Vector3d& p) {
    geometry_msgs::msg::Point point;
    point.x = p.x(); point.y = p.y(); point.z = p.z();
    return point;
}
void appendFaces(const CorridorPolytope& cell, bool planning,
                 visualization_msgs::msg::Marker& lines, visualization_msgs::msg::Marker& fill) {
    const auto vertices = corridorVertices(cell, planning);
    for (const auto& plane : cell.halfSpaces) {
        const double bound = planning ? plane.planningOffset : plane.geometricOffset;
        std::vector<Eigen::Vector3d> face;
        Eigen::Vector3d centre = Eigen::Vector3d::Zero();
        for (const auto& p : vertices) {
            if (std::abs(plane.normal.dot(p) - bound) < 1e-8) {
                face.push_back(p);
                centre += p;
            }
        }
        if (face.size() < 3) continue;
        centre /= face.size();
        const Eigen::Vector3d u = plane.normal.unitOrthogonal();
        const Eigen::Vector3d v = plane.normal.cross(u);
        std::sort(face.begin(), face.end(), [&](const auto& a, const auto& b) {
            const Eigen::Vector3d da = a - centre, db = b - centre;
            return std::atan2(v.dot(da), u.dot(da)) < std::atan2(v.dot(db), u.dot(db));
        });
        for (size_t i = 0; i < face.size(); ++i) {
            lines.points.push_back(toPoint(face[i]));
            lines.points.push_back(toPoint(face[(i + 1) % face.size()]));
        }
        if (planning) {
            for (size_t i = 1; i + 1 < face.size(); ++i) {
                for (const auto& p : {face[0], face[i], face[i + 1]}) fill.points.push_back(toPoint(p));
            }
        }
    }
}
}  // namespace

visualization_msgs::msg::MarkerArray terrainCorridorMarkers(
    const std::vector<TerrainSafetyCorridor>& corridors, const std_msgs::msg::Header& header) {
    visualization_msgs::msg::MarkerArray result;
    visualization_msgs::msg::Marker clear;
    clear.header = header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    result.markers.push_back(clear);
    int id = 0;
    for (size_t leg = 0; leg < corridors.size(); ++leg) {
        const auto& corridor = corridors[leg];
        if (!corridor.valid) continue;
        for (size_t i = 0; i < corridor.cells.size(); ++i) {
            const auto& cell = corridor.cells[i];
            visualization_msgs::msg::Marker geom, plan, fill;
            for (auto* marker : {&geom, &plan, &fill}) {
                marker->header = header;
                marker->ns = "leg_" + std::to_string(leg);
                marker->id = id++;
                marker->pose.orientation.w = 1.0;
                marker->scale.x = marker->scale.y = marker->scale.z = 1.0;
            }
            geom.type = plan.type = visualization_msgs::msg::Marker::LINE_LIST;
            fill.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            geom.scale.x = 0.0015;
            plan.scale.x = 0.003;
            geom.color.r = 0.7; geom.color.g = 0.7; geom.color.b = 0.7; geom.color.a = 0.35;
            plan.color.r = (i % 3 == 0) ? 0.95 : 0.15;
            plan.color.g = (i % 3 == 1) ? 0.85 : 0.35;
            plan.color.b = (i % 3 == 2) ? 0.95 : 0.3;
            plan.color.a = 0.9;
            fill.color = plan.color;
            fill.color.a = 0.06;
            appendFaces(cell, false, geom, fill);
            appendFaces(cell, true, plan, fill);
            result.markers.push_back(std::move(geom));
            result.markers.push_back(std::move(plan));
            result.markers.push_back(std::move(fill));
        }
    }
    return result;
}

TerrainCorridorVisualization::TerrainCorridorVisualization(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, size_t numFeet, scalar_t maxUpdateFrequency) {
    (void)numFeet;
    (void)maxUpdateFrequency;
    marker_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(kTopic, 1);
}

void TerrainCorridorVisualization::update(const std::vector<TerrainSafetyCorridor>& corridors,
                                          const std_msgs::msg::Header& header) {
    marker_publisher_->publish(terrainCorridorMarkers(corridors, header));
}
}  // namespace ocs2::legged_robot
