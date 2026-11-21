#pragma once
#include "ocs2_quadruped_controller/interface/constraint/PolynomialHull.h"
#include "ocs2_quadruped_controller/interface/constraint/TerrainGeometry.h"

namespace ocs2::legged_robot {

enum class CollisionStatus { ProvenSafe, CollisionDetected, Unknown };
struct CollisionCertificate {
    CollisionStatus status = CollisionStatus::Unknown;
    int checkedSegments = 0;
    int subdivisions = 0;
    double witnessTime = 0.0;
};
struct CollisionCheckConfig {
    HullBasis basis = HullBasis::Minvo;
    int maxDepth = 10;
    int maxSegments = 4096;
    double numericalTolerance = 1e-9;  // metre contact/roundoff tolerance, not tracking error
};

// Section 3.3: hull + sphere vs conservative occupied prisms; overlap -> subdivision.
class SwingCollisionCertifier {
   public:
    SwingCollisionCertifier(std::shared_ptr<TerrainInterface> terrain, CollisionCheckConfig config = {});
    CollisionCertificate check(const BSpline& contactHeightSpline, double radius) const;

   private:
    std::shared_ptr<TerrainInterface> terrain_;
    CollisionCheckConfig config_;
};
}  // namespace ocs2::legged_robot
