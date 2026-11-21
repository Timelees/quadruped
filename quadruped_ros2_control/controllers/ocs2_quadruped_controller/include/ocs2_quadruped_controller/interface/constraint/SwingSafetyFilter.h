#pragma once

#include <Eigen/Core>
#include <limits>
#include <optional>
#include "ocs2_quadruped_controller/interface/constraint/BSpline.h"

namespace ocs2::legged_robot {
struct TerrainSafetyCorridor;
// First execution-layer implementation, independently disabled by default.
struct SafetyFilterConfig {
    bool enabled = false;
    bool tubeEnabled = true;
    double hocbfGain1 = 20.0;
    double hocbfGain2 = 20.0;
    double disturbanceBound = 0.5;  // isotropic acceleration residual [m/s^2]
    double positionMeasurementError = 0.001; // Euclidean initial-state uncertainty [m]
    double velocityMeasurementError = 0.01;   // [m/s]
    double positionBound = 0.01;    // per-axis error box [m]
    double velocityBound = 0.30;    // [m/s]
    double couplingGain = 10.0;
    double couplingBound = 0.35;
    double tubeRate = 20.0;
    double maxCorrection = 20.0;    // [m/s^2], per axis
    double maxAcceleration = 80.0;
    // Sample/hold compensation zeta = rate * actual control period. Rates
    // require calibration against the actuator/model residual before deployment.
    double hocbfSampleRate = 20.0;  // [m/s^3]
    double tubeSampleRate = 10.0;
    double landingPositionWindow = 0.025; // half width [m]
    double landingMaxDownSpeed = 0.30;    // [m/s]
    double landingMaxUpSpeed = 0.10;      // [m/s]
    double maxPeriod = 0.006;       // certified sampling budget [s]
};

enum class SafetyFilterStatus { Disabled, Solved, NoCorridor, NoActiveSwing, Infeasible, InvalidState, InvalidPeriod, BarrierEntry, TubeEntry, TubeRate, MarginMismatch, JointInfeasible, Contact, LandingSolved, NoLandingCorridor, GeometryOnly, LandingGeometryOnly };
struct SafetyFilterResult {
    SafetyFilterStatus status = SafetyFilterStatus::Disabled;
    SafetyFilterStatus tubeStatus = SafetyFilterStatus::Disabled;
    Eigen::Vector3d correction = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    // Same acceleration rows are imposed in the final WBC QP, including boxes.
    // A soft acceleration tracking task alone cannot enforce a safety filter.
    Eigen::MatrixXd accelerationA;
    Eigen::VectorXd accelerationB;
    // Experimental, same-plane diagnostics: normal(3), offset, raw h,
    // robust h, robust hdot, robust psi1, zero-hold input bound.
    Eigen::MatrixXd hocbfDiagnostics;
    int activeRows = 0, iterations = 0;
    double clearance = std::numeric_limits<double>::quiet_NaN();
    double psi1 = std::numeric_limits<double>::quiet_NaN();
    double tubeSlack = std::numeric_limits<double>::quiet_NaN();
    double worstHocbfResidual = 0.0, worstTubeResidual = 0.0;
    bool active() const { return status == SafetyFilterStatus::Solved || status == SafetyFilterStatus::LandingSolved ||
        status == SafetyFilterStatus::GeometryOnly || status == SafetyFilterStatus::LandingGeometryOnly; }
    bool failed() const { return !active() && status != SafetyFilterStatus::Disabled &&
        status != SafetyFilterStatus::NoActiveSwing && status != SafetyFilterStatus::Contact; }
};

class SwingSafetyFilter {
 public:
    struct FootState {
        Eigen::Vector3d position = Eigen::Vector3d::Zero(); // sphere centre
        Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
        Eigen::Vector3d positionError = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocityError = Eigen::Vector3d::Zero();
        Eigen::Vector3d referenceAcceleration = Eigen::Vector3d::Zero();
        // Whole-body execution bounds interventions around its feasible output.
        // Standalone filtering defaults to the nominal tracking input.
        std::optional<Eigen::Vector3d> correctionCentre;
        // Optional exact reference over the zero-order hold. Without it the
        // caller explicitly supplies a constant-acceleration reference model.
        const BSpline* reference = nullptr;
        double referenceTime = 0.0;
    };
    explicit SwingSafetyFilter(SafetyFilterConfig config = {});
    const SafetyFilterConfig& config() const { return config_; }
    void setEnabled(bool enabled) { config_.enabled = enabled; }
    // phase is normalized to the ORIGINAL swing window, period is actual seconds.
    // Only the cell(s) containing phase are active. No blanket floor release:
    // contact transitions outside the corridor certificate retain existing control.
    SafetyFilterResult filter(const TerrainSafetyCorridor&, const FootState&,
        const Eigen::Vector3d& nominalAcceleration, double phase, double period, double timeToTouchdown = 0.0) const;
    // Execution keeps feasible hard geometry when the stricter tube is
    // unavailable. Admission continues to call filter(), which never relaxes it.
    SafetyFilterResult filterExecution(const TerrainSafetyCorridor&, const FootState&,
        const Eigen::Vector3d& nominalAcceleration, double phase, double period, double timeToTouchdown = 0.0) const;
 private:
    SafetyFilterConfig config_;
};
}  // namespace ocs2::legged_robot
