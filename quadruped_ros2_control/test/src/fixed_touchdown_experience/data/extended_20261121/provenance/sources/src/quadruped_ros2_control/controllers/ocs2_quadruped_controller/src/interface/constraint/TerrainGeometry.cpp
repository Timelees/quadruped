#include "ocs2_quadruped_controller/interface/constraint/TerrainGeometry.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ocs2::legged_robot {
GridMapTerrainInterface::GridMapTerrainInterface(
    std::shared_ptr<convex_plane_decomposition::PlanarTerrain> terrain, const std::string& layer)
    : terrain_(std::move(terrain)), layer_(layer) {
    if (!terrain_) {
        throw std::invalid_argument("Terrain map is required.");
    }
}

std::shared_ptr<TerrainInterface> GridMapTerrainInterface::snapshot() const {
    return std::make_shared<GridMapTerrainInterface>(
        std::make_shared<convex_plane_decomposition::PlanarTerrain>(*terrain_), layer_);
}

std::vector<TerrainPrism> GridMapTerrainInterface::getPrisms(const Eigen::Vector2d& minXY,
                                                             const Eigen::Vector2d& maxXY) const {
    const auto& map = terrain_->gridMap;
    const auto layer = map.exists(certifiedElevationLayer(layer_)) ? certifiedElevationLayer(layer_) : layer_;
    if (!minXY.allFinite() || !maxXY.allFinite() || (minXY.array() > maxXY.array()).any()) {
        throw std::invalid_argument("Invalid terrain query rectangle.");
    }
    if (!map.exists(layer)) {
        throw TerrainUnavailable("Missing terrain layer: " + layer_);
    }
    const double r = map.getResolution();
    const Eigen::Vector2d origin =
        map.getPosition() - 0.5 * map.getLength().matrix() + Eigen::Vector2d::Constant(0.5 * r);
    const Eigen::Vector2i first = ((minXY - origin) / r).array().floor().cast<int>();
    const Eigen::Vector2i last = ((maxXY - origin) / r).array().floor().cast<int>();
    if ((first.array() < 0).any() || (last.array() >= map.getSize().array() - 1).any()) {
        throw TerrainUnavailable("Terrain rectangle exceeds known interpolation support.");
    }
    std::vector<TerrainPrism> result;
    result.reserve((last.x() - first.x() + 1) * (last.y() - first.y() + 1));
    for (int i = first.x(); i <= last.x(); ++i) {
        for (int j = first.y(); j <= last.y(); ++j) {
            const Eigen::Vector2d lower = origin + r * Eigen::Vector2d(i, j);
            double upper = -std::numeric_limits<double>::infinity();
            for (int dx = 0; dx < 2; ++dx) {
                for (int dy = 0; dy < 2; ++dy) {
                    const double z = map.atPosition(layer, lower + r * Eigen::Vector2d(dx, dy));
                    if (!std::isfinite(z)) {
                        throw TerrainUnavailable("Unknown terrain interpolation patch.");
                    }
                    upper = std::max(upper, z);
                }
            }
            // A bilinear patch is a convex combination of all four corner heights.
            result.push_back({lower, lower + Eigen::Vector2d::Constant(r), upper});
        }
    }
    return result;
}

double GridMapTerrainInterface::getHeight(double x, double y) const {
    const Eigen::Vector2d xy(x, y);
    const auto patch = getPrisms(xy, xy).front();
    const double r = terrain_->gridMap.getResolution();
    const auto layer = terrain_->gridMap.exists(certifiedElevationLayer(layer_)) ? certifiedElevationLayer(layer_) : layer_;
    const Eigen::Vector2d uv = (xy - patch.minXY) / r;
    double height = 0.0;
    for (int i = 0; i < 2; ++i) {
        for (int j = 0; j < 2; ++j) {
            height += (i ? uv.x() : 1 - uv.x()) * (j ? uv.y() : 1 - uv.y()) *
                      terrain_->gridMap.atPosition(layer, patch.minXY + r * Eigen::Vector2d(i, j));
        }
    }
    return height;
}

double TerrainInterface::getSphereClearanceUpperBound(const Eigen::Vector2d& minXY,
                                                      const Eigen::Vector2d& maxXY, double radius) const {
    double bound = -std::numeric_limits<double>::infinity();
    for (const auto& prism :
         getPrisms(minXY - Eigen::Vector2d::Constant(radius), maxXY + Eigen::Vector2d::Constant(radius))) {
        const Eigen::Vector2d distance = (prism.minXY - maxXY).cwiseMax(minXY - prism.maxXY).cwiseMax(0.0);
        const double squared = distance.squaredNorm();
        if (squared <= radius * radius) {
            bound =
                std::max(bound, prism.height + std::sqrt(std::max(0.0, radius * radius - squared)) - radius);
        }
    }
    return bound;
}
}  // namespace ocs2::legged_robot
