#pragma once

#include <convex_plane_decomposition/PlanarRegion.h>

#include <Eigen/Core>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace ocs2::legged_robot {

struct TerrainPrism {
    Eigen::Vector2d minXY, maxXY;
    double height;  // occupied from below to height, world frame
};
class TerrainUnavailable : public std::runtime_error {
   public:
    using std::runtime_error::runtime_error;
};

class TerrainInterface {
   public:
    virtual ~TerrainInterface() = default;
    virtual double getHeight(double x, double y) const = 0;
    // Must cover the entire requested rectangle, or throw TerrainUnavailable.
    virtual std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d& minXY,
                                                const Eigen::Vector2d& maxXY) const = 0;
    double getSphereClearanceUpperBound(const Eigen::Vector2d& minXY, const Eigen::Vector2d& maxXY,
                                        double radius) const;
};

class GridMapTerrainInterface final : public TerrainInterface {
   public:
    GridMapTerrainInterface(std::shared_ptr<convex_plane_decomposition::PlanarTerrain> terrain,
                            const std::string& layer);
    double getHeight(double x, double y) const override;
    std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d& minXY,
                                        const Eigen::Vector2d& maxXY) const override;

   private:
    std::shared_ptr<convex_plane_decomposition::PlanarTerrain> terrain_;
    std::string layer_;
};
}  // namespace ocs2::legged_robot
