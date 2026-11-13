#pragma once

#include <ocs2_core/Types.h>

#include <functional>

#include "ocs2_quadruped_controller/interface/constraint/SwingCollisionCertifier.h"

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
    double hessianRegularization = 1e-8;
    int qpMaxWorkingSetRecalculations = 200;
    std::string elevationLayer = "elevation";
    bool verbose = false;
    SwingGeometryMode geometryMode = SwingGeometryMode::Minvo;
    int subdivisionsPerSpan = 4;
    bool checkReference = true;
    bool verifyOutput = true;  // disable only for uncertified ablation output
    CollisionCheckConfig collisionCheck;
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
