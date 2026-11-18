#include <iostream>
#include "ocs2_quadruped_controller/interface/constraint/TerrainSafetyCorridor.h"
using namespace ocs2::legged_robot;
namespace {
class Terrain final : public TerrainInterface {
 public:
    double getHeight(double, double) const override { return 0.0; }
    std::vector<TerrainPrism> getPrisms(const Eigen::Vector2d&, const Eigen::Vector2d&) const override {
        return {{{-2, -2}, {2, 2}, 0.0}, {{0.24, -0.1}, {0.26, 0.1}, 0.08}};
    }
};
}
int main() {
    BSpline spline(5);
    Eigen::VectorXd x(6);
    x << 0, 0.1, 0.2, 0.3, 0.4, 0.5;
    spline.setControlPoints(x, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Constant(6, 0.2));
    spline.setKnots(spline.generateKnots(6, 0, 1));
    const auto corridor = buildTerrainSafetyCorridor(Terrain{}, spline, CorridorConfig{});
    if (!corridor.valid || corridor.cells.size() < 2) return 1;
    const auto rows = corridorControlPointConstraints(corridor);
    Eigen::VectorXd q(18);
    q << x, Eigen::VectorXd::Zero(6), Eigen::VectorXd::Constant(6, 0.2);
    if ((rows.A * q - rows.b).maxCoeff() > 1e-9) return 1;
    std::cout << "3D corridor / MINVO conversion self-check passed\n";
}
