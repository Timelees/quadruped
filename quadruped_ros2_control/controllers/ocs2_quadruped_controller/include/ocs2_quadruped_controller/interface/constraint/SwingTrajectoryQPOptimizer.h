#pragma once

#include <ocs2_core/Types.h>

#include <functional>
#include <limits>

#include "ocs2_quadruped_controller/interface/constraint/SwingCollisionCertifier.h"
#include "ocs2_quadruped_controller/interface/constraint/TerrainSafetyCorridor.h"

namespace ocs2::legged_robot {
enum class SwingGeometryMode { Discrete = 0, Bezier = 1, Minvo = 2 };

struct SwingTrajectoryQpConfig {
    int numControlPoints = 15;
    int numCurveSamples = 60;
    int splineOrder = 5;
    double controlPointTrackingWeight = 0.8;
    double positionTrackingWeight = 0.8;
    double velocityTrackingWeight = 0.35;
    double accelerationTrackingWeight = 0.2;
    double jerkSmoothnessWeight = 0.08;
    double groundClearance = 0.03;
    double footRadius = 0.019;
    // Section 3.1 corridor shrink: reference-to-executed tracking error the
    // terrain rows must reserve (b_bar = b - sigma_E). Applied with the same
    // C2 envelope as groundClearance so contacts stay feasible.
    double trackingMargin = 0.02;
    // Body envelope F behind the foot: a trailing calf bound sampled at these
    // horizontal distances, tilted by tan(theta) to follow the shank. Scaled by
    // the same C2 envelope so a touchdown next to a riser stays feasible.
    double calfLength = 0.09;
    double calfSlope = 0.6;
    int calfSamples = 3;
    // Executability rows on the correction curve. Without them the QP happily
    // returns corrections the NMPC/WBC chain cannot track.
    double maxCorrection = 0.10;
    double maxVelocity = 2.0;
    double maxAcceleration = 80.0;
    double hessianRegularization = 1e-8;
    int qpMaxWorkingSetRecalculations = 200;
    std::string elevationLayer = "elevation";
    bool verbose = false;
    SwingGeometryMode geometryMode = SwingGeometryMode::Minvo;
    int subdivisionsPerSpan = 4;
    bool checkReference = true;
    bool verifyOutput = true;  // disable only for uncertified ablation output
    CollisionCheckConfig collisionCheck;
    // Optional immutable 3D corridor rows, additional to existing terrain/calf
    // rows. Applies to the explicit non-contact interval in corridorConfig.
    bool safetyCorridorEnabled = false;
    CorridorConfig corridorConfig;
};

class SwingTrajectoryQpOptimizer {
   public:
    struct OptimizedTrajectory {
        std::shared_ptr<BSpline> bspline;
        scalar_t startTime = 0.0, endTime = 0.0;
        bool solved = false;
        double correctionCost = 0.0;
        CollisionCertificate referenceCertificate, certificate;
        int constraintRows = 0, workingSetIterations = 0;
        TerrainSafetyCorridor corridor;
        bool corridorCertified = false;  // all six hull vertices, non-contact interval only
        double corridorMaxViolation = std::numeric_limits<double>::infinity();
        bool valid() const { return solved && static_cast<bool>(bspline); }
    };
    using HeightBounds = std::function<Eigen::Vector2d(double, const Eigen::Vector3d&)>;
    SwingTrajectoryQpOptimizer(SwingTrajectoryQpConfig config, std::shared_ptr<TerrainInterface> terrain);
    OptimizedTrajectory optimizeSwingTrajectory(const BSpline& reference, scalar_t start, scalar_t end,
                                                const Eigen::Vector3d& liftOff,
                                                const Eigen::Vector3d& touchDown,
                                                const HeightBounds& workspace = {}) const;
    const SwingTrajectoryQpConfig& getConfig() const { return config_; }

   private:
    // Lower bound on foot-centre z at one surface point: terrain envelope behind
    // the foot (calf), terrain under it, plus corridor margins, blended toward
    // referenceZ by the contact envelope so fixed contact rows stay feasible.
    // footTerrain is the height queried at the fixed horizontal hull vertex.
    double terrainLowerBound(const Eigen::Vector2d& xy, const Eigen::Vector2d& heading, double envelope,
                             double contactHeight, double footTerrain) const;
    bool solveQp(const Eigen::MatrixXd& hessian, const Eigen::MatrixXd& constraints,
                 const Eigen::VectorXd& lowerBounds, const Eigen::VectorXd& reference,
                 Eigen::VectorXd& solution, int& iterations) const;
    void sampleReferenceTrajectoryToControlPoints(const BSpline& reference, scalar_t start, scalar_t end,
                                                  const Eigen::Vector3d& liftOff,
                                                  const Eigen::Vector3d& touchDown,
                                                  std::vector<Eigen::Vector3d>& points) const;
    std::shared_ptr<TerrainInterface> terrainInterface_;
    SwingTrajectoryQpConfig config_;
};
}  // namespace ocs2::legged_robot
