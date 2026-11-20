/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <array>
#include <atomic>
#include <limits>
#include <memory>

#include <ocs2_core/reference/ModeSchedule.h>

#include <ocs2_legged_robot/common/Types.h>

#include "ocs2_quadruped_controller/interface/constraint/BSpline.h"
#include "ocs2_quadruped_controller/interface/constraint/SwingOnlineReplanner.h"
#include "ocs2_quadruped_controller/interface/constraint/SwingSafetyFilter.h"
#include "ocs2_quadruped_controller/interface/constraint/SwingTrajectoryQPOptimizer.h"
#include "ocs2_quadruped_controller/interface/constraint/TerrainSafetyCorridor.h"

namespace ocs2::legged_robot {
    // Shared by the online planner and scene experiments; midHeight is the curve height at mid-time.
    BSpline createSwingHeightBSpline(scalar_t swingStartTime, scalar_t swingFinalTime,
                                    scalar_t liftOffHeight, scalar_t liftOffVelocity,
                                    scalar_t midHeight, scalar_t touchDownHeight,
                                    scalar_t touchDownVelocity);

    class SwingTrajectoryPlanner {
    public:
        struct Config {
            scalar_t liftOffVelocity = 0.0;
            scalar_t touchDownVelocity = 0.0;
            scalar_t swingHeight = 0.1;
            scalar_t swingTimeScale = 0.15;
            scalar_t footRadius = 0.019; // preserve the original perceptive liftoff offset [m]
            scalar_t trackingPositionErrorGain = 20.0; // Terrain 3D tracking only; stance keeps model_settings gain
            scalar_t trackingVerticalErrorGain = 20.0;
            bool trackingVerticalHardConstraint = false;
            // swing phases shorter than this time will be scaled down in height and velocity
            bool useTerrainPlanning = false;
            bool directReplacementTestOnly = false; // causal ablation, bypasses state admission
            SwingTrajectoryQpConfig terrainQpConfig;
            SwingReplanConfig replanConfig;
            SafetyFilterConfig safetyFilterConfig;
        };

        SwingTrajectoryPlanner(Config config, size_t numFeet);

        void update(const ModeSchedule &modeSchedule, scalar_t terrainHeight);

        void update(const ModeSchedule &modeSchedule, const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                    const feet_array_t<scalar_array_t> &touchDownHeightSequence,
                    scalar_t initTime = -std::numeric_limits<scalar_t>::infinity());

        void update(const ModeSchedule &modeSchedule, const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                    const feet_array_t<scalar_array_t> &touchDownHeightSequence,
                    const feet_array_t<scalar_array_t> &maxHeightSequence,
                    scalar_t initTime = -std::numeric_limits<scalar_t>::infinity());

        void updateTerrain(const ModeSchedule &modeSchedule,
                       const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                       const feet_array_t<scalar_array_t> &touchDownHeightSequence,
                       const feet_array_t<scalar_array_t> &maxHeightSequence,
                       const feet_array_t<std::vector<Eigen::Vector3d>> &liftOffPositionSequence,
                       const feet_array_t<std::vector<Eigen::Vector3d>> &touchDownPositionSequence,
                       scalar_t initTime = 0.0,
                       scalar_t finalTime = std::numeric_limits<scalar_t>::infinity());

        void updateTerrain(const ModeSchedule &modeSchedule,
                       const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                       const feet_array_t<scalar_array_t> &touchDownHeightSequence,
                       const feet_array_t<std::vector<Eigen::Vector3d>> &liftOffPositionSequence,
                       const feet_array_t<std::vector<Eigen::Vector3d>> &touchDownPositionSequence,
                       scalar_t initTime = 0.0,
                       scalar_t finalTime = std::numeric_limits<scalar_t>::infinity());

        // Contact-reference state (sphere centre minus footRadius*ez) at initTime. Needed by the
        // section 4.4 online replanning branch; an empty array skips it.
        using SwingFootState = feet_array_t<Eigen::Vector3d>;

        void updateTerrain(const ModeSchedule &modeSchedule,
                       const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                       const feet_array_t<scalar_array_t> &touchDownHeightSequence,
                       const feet_array_t<std::vector<Eigen::Vector3d>> &liftOffPositionSequence,
                       const feet_array_t<std::vector<Eigen::Vector3d>> &touchDownPositionSequence,
                       scalar_t initTime, scalar_t finalTime,
                       const SwingFootState &footPosition, const SwingFootState &footVelocity,
                       const SwingFootState &footAcceleration);

        void updateTerrain(const ModeSchedule &modeSchedule,
                       const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                       const feet_array_t<scalar_array_t> &touchDownHeightSequence,
                       const feet_array_t<scalar_array_t> &maxHeightSequence,
                       const feet_array_t<std::vector<Eigen::Vector3d>> &liftOffPositionSequence,
                       const feet_array_t<std::vector<Eigen::Vector3d>> &touchDownPositionSequence,
                       scalar_t initTime, scalar_t finalTime,
                       const SwingFootState &footPosition, const SwingFootState &footVelocity,
                       const SwingFootState &footAcceleration);

        void setMapTiming(TerrainMapTiming timing);
        void setTerrainInterface(std::shared_ptr<TerrainInterface> terrainInterface);
        using Workspace = std::function<Eigen::Vector2d(size_t, double, const Eigen::Vector3d&)>;
        void setWorkspace(Workspace workspace) { workspace_ = std::move(workspace); }
        bool isTerrainPlanningEnabled() const { return config_.useTerrainPlanning; }
        const Config& getConfig() const { return config_; }
        bool isOnlineReplanningEnabled() const { return replanner_.config().enabled; }
        void setOnlineReplanningEnabled(bool enabled) { replanner_.setEnabled(enabled); }
        // Controlled map-reveal experiments release all methods at the same
        // intervention. A release cannot enable a configured frozen baseline.
        void setOnlineReplanningReleased(bool released) {
            replanner_.setEnabled(config_.replanConfig.enabled && released);
        }
        const SwingReplanStats& getReplanStats() const { return replanner_.stats(); }
        // Section 4.4 admission statistics; only counted when replanning is on.
        void recordReplanStats(int& admitted, int& rejected) const {
            const auto& stats = replanner_.stats();
            admitted = stats.admitted;
            rejected = stats.rejectedDeviation + stats.rejectedMargin + stats.rejectedCandidate;
        }
        // Most recent corridor built per leg, published for RViz. Empty entries
        // mean corridor mode is off or construction failed (e.g. unknown map).
        feet_array_t<TerrainSafetyCorridor> getTerrainCorridors() const;
        // Corridor of the swing that covers time, for the execution-layer safety
        // filter. The RViz corridor above is the freshest future plan, which is
        // not the geometry the foot is currently flying through.
        const TerrainSafetyCorridor& getTerrainCorridor(size_t leg, scalar_t time) const;

        struct ExecutionSwing {
            BSpline reference;
            TerrainSafetyCorridor corridor;
            double start = 0.0, end = 0.0;
        };
        using ExecutionSwings = feet_array_t<std::vector<ExecutionSwing>>;
        struct TerrainSnapshot {
            std::shared_ptr<TerrainInterface> terrain;
            TerrainMapTiming timing;
        };
        std::shared_ptr<const TerrainSnapshot> certificationTerrain() const {
            return std::atomic_load(&certificationTerrain_);
        }
        // Whole-interval check using an immutable synchronized map, also used
        // for nominal fallback and actual activation. No trajectory QP runs here.
        TerrainSafetyCorridor certifyReference(const BSpline&, double start, double end,
            double after, const TerrainSnapshot&) const;
        struct ActivationCheck {
            bool accepted = false;
            std::string reason;
            double delay = 0.0, positionError = 0.0, velocityError = 0.0;
            TerrainSafetyCorridor corridor;
        };
        ActivationCheck checkActivation(const ExecutionSwing& candidate, const ExecutionSwing* previous,
            double now, double period, const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
            const TerrainSnapshot& map) const;
        void setActiveExecutionSwings(std::shared_ptr<const ExecutionSwings> swings) {
            std::atomic_store(&activeExecutionSwings_, std::move(swings));
        }
        std::shared_ptr<const ExecutionSwings> activeExecutionSwings() const {
            return std::atomic_load(&activeExecutionSwings_);
        }
        // After an actual activation veto, recompute the next MPC policy using
        // the accepted reference for one planning cycle before bounded retries;
        // newer terrain evidence must remain actionable before touchdown.
        void rejectExecutionUpdate(size_t leg, double swingEnd) {
            rejectedSwingEnd_.at(leg).store(swingEnd, std::memory_order_relaxed);
        }
        std::shared_ptr<const ExecutionSwings> executionSwings() const {
            return std::atomic_load(&executionSwings_);
        }
        struct FootFeedback {
            double time = -std::numeric_limits<double>::infinity();
            SwingFootState velocity;
        };
        void setFootFeedback(std::shared_ptr<const FootFeedback> feedback) {
            std::atomic_store(&footFeedback_, std::move(feedback));
        }
        std::shared_ptr<const FootFeedback> footFeedback() const { return std::atomic_load(&footFeedback_); }

        vector3_t getVelocityConstraint(size_t leg, scalar_t time) const;
        vector3_t getPositionConstraint(size_t leg, scalar_t time) const;

        scalar_t getZvelocityConstraint(size_t leg, scalar_t time) const;

        scalar_t getZpositionConstraint(size_t leg, scalar_t time) const;

    private:
        BSpline replanExecutingSwing(size_t leg, const BSpline &executing, scalar_t swingStartTime,
                                     scalar_t swingFinalTime, scalar_t initTime,
                                     const SwingFootState &footPosition, const SwingFootState &footVelocity,
                                     const SwingFootState &footAcceleration);
        feet_array_t<std::vector<bool> > extractContactFlags(const std::vector<size_t> &phaseIDsStock) const;
        static std::pair<int, int> findIndex(size_t index, const std::vector<bool> &contactFlagStock);
        static std::pair<std::vector<int>, std::vector<int> > updateFootSchedule(
            const std::vector<bool> &contactFlagStock);
        static void checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex,
                                             const std::vector<size_t> &phaseIDsStock);
        static scalar_t swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale);
        Workspace workspace_;
        const Config config_;
        const size_t numFeet_;
        std::array<std::atomic<double>, 4> rejectedSwingEnd_{};

        std::shared_ptr<SwingTrajectoryQpOptimizer> swingTrajectoryQpOptimizer_;
        TerrainMapTiming mapTiming_;
        std::shared_ptr<TerrainInterface> terrainInterface_;  // For adaptive swing height
        SwingOnlineReplanner replanner_;
        std::shared_ptr<const ExecutionSwings> executionSwings_;
        std::shared_ptr<const ExecutionSwings> activeExecutionSwings_;
        std::shared_ptr<const TerrainSnapshot> certificationTerrain_;
        std::shared_ptr<const FootFeedback> footFeedback_;
        feet_array_t<double> lastReplanTime_{{-INFINITY, -INFINITY, -INFINITY, -INFINITY}};
        feet_array_t<double> pendingActivation_{{INFINITY, INFINITY, INFINITY, INFINITY}};
        feet_array_t<double> pendingEnd_{};


        feet_array_t<std::vector<BSpline> > feetHeightTrajectories_;
        feet_array_t<std::vector<scalar_t> > feetHeightTrajectoriesEvents_;
        feet_array_t<std::vector<TerrainSafetyCorridor> > terrainCorridors_;
        feet_array_t<std::vector<scalar_t> > terrainCorridorStarts_, terrainCorridorEnds_;
    };

    SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string &fileName,
                                                               const std::string &fieldName = "swing_trajectory_config",
                                                               bool verbose = true);
}
