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

#include <memory>

#include <ocs2_core/reference/ModeSchedule.h>

#include <ocs2_legged_robot/common/Types.h>

#include "ocs2_quadruped_controller/interface/constraint/BSpline.h"
#include "ocs2_quadruped_controller/interface/constraint/SwingTrajectoryQPOptimizer.h"

namespace ocs2::legged_robot {
    class SwingTrajectoryPlanner {
    public:
        struct Config {
            scalar_t liftOffVelocity = 0.0;
            scalar_t touchDownVelocity = 0.0;
            scalar_t swingHeight = 0.1;
            scalar_t swingTimeScale = 0.15;
            // swing phases shorter than this time will be scaled down in height and velocity
            bool useCbfUpdate = false;
            SwingTrajectoryQpConfig cbfQpConfig;
        };

        SwingTrajectoryPlanner(Config config, size_t numFeet);

        void update(const ModeSchedule &modeSchedule, scalar_t terrainHeight);

        void update(const ModeSchedule &modeSchedule, const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                    const feet_array_t<scalar_array_t> &touchDownHeightSequence);

        void update(const ModeSchedule &modeSchedule, const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                    const feet_array_t<scalar_array_t> &touchDownHeightSequence,
                    const feet_array_t<scalar_array_t> &maxHeightSequence);

        void updateCBF(const ModeSchedule &modeSchedule,
                       const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                       const feet_array_t<scalar_array_t> &touchDownHeightSequence,
                       const feet_array_t<std::vector<Eigen::Vector3d>> &liftOffPositionSequence,
                       const feet_array_t<std::vector<Eigen::Vector3d>> &touchDownPositionSequence);

        void updateCBF(const ModeSchedule &modeSchedule,
                       const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                       const feet_array_t<scalar_array_t> &touchDownHeightSequence,
                       const feet_array_t<scalar_array_t> &maxHeightSequence,
                       const feet_array_t<std::vector<Eigen::Vector3d>> &liftOffPositionSequence,
                       const feet_array_t<std::vector<Eigen::Vector3d>> &touchDownPositionSequence);

        void setTerrainInterface(std::shared_ptr<TerrainInterface> terrainInterface);
        bool isCbfUpdateEnabled() const { return config_.useCbfUpdate && static_cast<bool>(swingTrajectoryQpOptimizer_); }
        const Config& getConfig() const { return config_; }

        scalar_t getZvelocityConstraint(size_t leg, scalar_t time) const;

        scalar_t getZpositionConstraint(size_t leg, scalar_t time) const;

    private:
        struct CachedSwingTrajectory {
            scalar_t startTime = 0.0;
            scalar_t finalTime = 0.0;
            Eigen::Vector3d liftOff = Eigen::Vector3d::Zero();
            Eigen::Vector3d touchDown = Eigen::Vector3d::Zero();
            BSpline spline;
        };

        feet_array_t<std::vector<bool> > extractContactFlags(const std::vector<size_t> &phaseIDsStock) const;
        static std::pair<int, int> findIndex(size_t index, const std::vector<bool> &contactFlagStock);
        static std::pair<std::vector<int>, std::vector<int> > updateFootSchedule(
            const std::vector<bool> &contactFlagStock);
        static void checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex,
                                             const std::vector<size_t> &phaseIDsStock);
        static scalar_t swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale);
        bool getCachedSwingTrajectory(size_t leg, scalar_t startTime, scalar_t finalTime,
                                      const Eigen::Vector3d& liftOff, const Eigen::Vector3d& touchDown,
                                      BSpline& spline) const;
        void cacheSwingTrajectory(size_t leg, scalar_t startTime, scalar_t finalTime,
                                  const Eigen::Vector3d& liftOff, const Eigen::Vector3d& touchDown,
                                  const BSpline& spline);

        const Config config_;
        const size_t numFeet_;

        std::shared_ptr<SwingTrajectoryQpOptimizer> swingTrajectoryQpOptimizer_;

        feet_array_t<std::vector<BSpline> > feetHeightTrajectories_;
        feet_array_t<std::vector<scalar_t> > feetHeightTrajectoriesEvents_;
        feet_array_t<std::vector<CachedSwingTrajectory> > cachedSwingTrajectories_;
    };

    SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string &fileName,
                                                               const std::string &fieldName = "swing_trajectory_config",
                                                               bool verbose = true);
}
