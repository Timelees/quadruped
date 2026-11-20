#pragma once

#include <Eigen/Core>

#include <string>
#include <functional>

#include "ocs2_quadruped_controller/interface/constraint/BSpline.h"
#include "ocs2_quadruped_controller/interface/constraint/TerrainSafetyCorridor.h"

namespace ocs2::legged_robot {

// Section 4.4 / Algorithm 1 (steps 2-4): decide whether a mid-swing update may
// replace the executing reference while keeping the original touchdown time.
// The candidate is the old reference plus the quintic state correction of
// eq. (36)-(38); it is admitted only if the correction stays inside the old
// reference's remaining geometric margin over the correction window (eq. 39).
struct SwingReplanConfig {
    // Off by default: enabling it changes the executing reference during a swing.
    bool enabled = false;
    // Correction window delta of eq. (36). The candidate rejoins the old
    // reference with C2 continuity after this many seconds.
    double correctionDuration = 0.08;
    // Predicted latency between the update request and the actual activation.
    double activationDelay = 0.0;
    double maxActivationDelay = 0.10; // actual reference age accepted at MRT [s]
    // State deviations that are rejected before building anything.
    double maxPositionDeviation = 0.06;
    double maxVelocityDeviation = 0.40;
    // Z control correction in the remaining-trajectory QP. This is distinct
    // from the measured-state gate above; the loader shares the terrain QP's
    // existing correction budget instead of relaxing state admission.
    double maxTrajectoryCorrection = 0.06;
    // Eq. (39) consumes the margin available over the correction window. The
    // fraction reserves part of it for the executor.
    double maxMarginFraction = 0.5;
    double minRemainingTime = 0.10;
    double updateInterval = 0.06;
    // Physical derivative bounds on the complete candidate, per axis.
    double nominalTrackingWeight = 0.15;
    double previousTrackingWeight = 1.0;
    double landingPositionWindow = 0.025;
    double landingMaxDownSpeed = 0.30;
    double landingMaxUpSpeed = 0.10;
    double maxVelocity = 3.0;
    double maxAcceleration = 80.0;
    double maxJerk = 12000.0;
    // Eq. (34), isotropic state-estimation/model budgets.
    double positionUncertainty = 0.001;
    double velocityUncertainty = 0.01;
    double disturbanceBound = 0.5;
};

enum class SwingReplanOutcome {
    Disabled,
    Admitted,
    NoExecutingReference,
    RejectedDeviation,
    RejectedMargin,
    RejectedCandidate,
    RejectedCorridor,
    RejectedTerminal,
    RejectedActivation,
    RejectedControl,
};

// Counters for the admission-rate statistics of section 5.3. One attempt is one
// update of an executing swing, not one MPC cycle of a future swing.
struct SwingReplanStats {
    int attempts = 0;
    int admitted = 0;
    int noReference = 0;
    int rejectedDeviation = 0;
    int rejectedMargin = 0;
    int rejectedCandidate = 0;
    int rejectedCorridor = 0;
    int rejectedTerminal = 0;
    int rejectedActivation = 0;
    int rejectedControl = 0;
    std::string json() const;
};

struct SwingReplanDecision {
    SwingReplanOutcome outcome = SwingReplanOutcome::Disabled;
    BSpline candidate;  // defined on [activation, end] only when admitted
    double activation = 0.0;
    double end = 0.0;
    double correctionBound = 0.0;  // worst max_j a^T D_j over the corridor planes
    double availableMargin = 0.0;  // worst eq. (11) margin over the same planes
    std::string reason;
    bool admitted() const { return outcome == SwingReplanOutcome::Admitted; }
};

class SwingOnlineReplanner {
   public:
    explicit SwingOnlineReplanner(SwingReplanConfig config = {});

    // oldReference must cover [activation, end] and is the reference currently
    // being executed. swingStart is that reference's lift-off time: the corridor's
    // cell phases are normalized to [swingStart, end], not to the activation
    // instant. position/velocity/acceleration are the measured foot centre state at
    // the activation instant. corridor is the executing swing's corridor, so its
    // hull mappings match oldReference's control points.
    SwingReplanDecision admit(const BSpline& oldReference, const TerrainSafetyCorridor& corridor,
                              double swingStart, double activation, double end,
                              const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
                              const Eigen::Vector3d& acceleration);
    SwingReplanDecision admit(const BSpline& oldReference, const TerrainSafetyCorridor& corridor,
                              double swingStart, double activation, double end,
                              const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
                              const Eigen::Vector3d& acceleration,
                              const std::function<Eigen::Vector2d(double,const Eigen::Vector3d&)>& workspace);

    // Rebuild mappings in the candidate's own knot space, using exact segment
    // restriction at every corridor boundary. No sampling enters the certificate.
    static TerrainSafetyCorridor remapCorridor(const BSpline& reference,
                                               const TerrainSafetyCorridor& corridor,
                                               double swingStart, double swingEnd);
    bool validate(const BSpline& candidate, const TerrainSafetyCorridor& corridor,
                  double swingStart, double swingEnd) const;
    void reject(SwingReplanOutcome outcome);
    void recordRejection(SwingReplanOutcome outcome) { ++stats_.attempts; reject(outcome); }
    void revokeAdmission(SwingReplanOutcome outcome) { --stats_.admitted; reject(outcome); }

    const SwingReplanConfig& config() const { return config_; }
    void setEnabled(bool enabled) { config_.enabled = enabled; }
    const SwingReplanStats& stats() const { return stats_; }

   private:
    bool validate(const BSpline& candidate, const TerrainSafetyCorridor& corridor,
                  double swingStart, double swingEnd, std::string* failure) const;
    // Bernstein control points D_0..D_5 of eq. (38), in metres.
    static void correctionControlPoints(double delta, const Eigen::Vector3d& dPosition,
                                        const Eigen::Vector3d& dVelocity, const Eigen::Vector3d& dAcceleration,
                                        std::vector<Eigen::Vector3d>& controls);

    SwingReplanConfig config_;
    SwingReplanStats stats_;
};
}  // namespace ocs2::legged_robot
