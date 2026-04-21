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

#include <ocs2_core/reference/ModeSchedule.h>

#include <mutex>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_legged_robot/foot_planner/SplineCpg.h>
#include "ocs2_quadruped_controller/interface/constraint/QuinticSplineCpg.h"
#include "ocs2_quadruped_controller/interface/constraint/SwingTrajectoryQPOptimizer.h"
#include "ocs2_quadruped_controller/perceptive/synchronize/EdgeSphere.h"
#include "ocs2_quadruped_controller/interface/constraint/BSpline.h"

namespace ocs2::legged_robot {
    class SwingTrajectoryPlanner {
    public:
        struct Config {
            scalar_t liftOffVelocity = 0.0;     // 起跳时的速度（通常为正值，表示向上）
            scalar_t touchDownVelocity = 0.0;   // 着陆时的速度（通常为负值，表示向下）
            scalar_t swingHeight = 0.1;         // 摆动轨迹的标称高度（相对于地面）
            scalar_t swingTimeScale = 0.15;     // 用于缩放摆动时间的参数，实际摆动时间越长，轨迹越平缓。
            scalar_t safetyMargin = 0.02;       // 安全裕度
            // swing phases shorter than this time will be scaled down in height and velocity
        };

        struct TrajectoryCollisionInfo{
            bool collidesWithTerrain = false;  // 轨迹是否与地形发生碰撞
            bool collidesWithEdgeSphere = false; // 轨迹是否与边界球发生碰撞
            double minTerrainClearance = std::numeric_limits<double>::infinity(); // 与地形的最小距离
            double minEdgeSphereClearance = std::numeric_limits<double>::infinity(); // 与边界球的最小距离
            double maxHeightDeficit = 0.0; // 距离满足安全包络还差多少高度

            int nSamples = 200; // 轨迹采样点数
        };

        SwingTrajectoryPlanner(Config config, size_t numFeet);

        const Config& getConfig() const { return config_; }

        void update(const ModeSchedule &modeSchedule, scalar_t terrainHeight);

        void update(const ModeSchedule &modeSchedule, const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                    const feet_array_t<scalar_array_t> &touchDownHeightSequence);

        void update(const ModeSchedule &modeSchedule, const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                    const feet_array_t<scalar_array_t> &touchDownHeightSequence,
                    const feet_array_t<scalar_array_t> &maxHeightSequence);

        scalar_t getZvelocityConstraint(size_t leg, scalar_t time) const;

        scalar_t getZpositionConstraint(size_t leg, scalar_t time) const;

        contact_flag_t getContactFlags(scalar_t time) const;

        /**
         * @brief 设置摆动相 QP 优化器。
         */
        void setQpOptimizer(std::shared_ptr<TerrainInterface> terrainInterface,
                    const SwingTrajectoryQpConfig& qpConfig = SwingTrajectoryQpConfig{});

        /**
         * @brief 传入 EdgeSphere 数据供摆动相 QP 优化器使用。
         */
        void setEdgeSpheres(std::shared_ptr<EdgeSphereArray> edgeSpheresPtr);

        /**
         * @brief 带 QP 优化的摆动轨迹更新。
         *
         * 先生成参考轨迹，再在需要时调用 Z 方向 B 样条 QP 优化器。
         * 需要提供各腿的 3D 起飞/着地位置用于地形查询。
         */
        void updateWithQpOptimization(const ModeSchedule &modeSchedule,
                                      scalar_t currentTime,
                                      const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                                      const feet_array_t<scalar_array_t> &touchDownHeightSequence,
                                      const feet_array_t<std::vector<Eigen::Vector3d>> &liftOffPositions3d,
                                      const feet_array_t<std::vector<Eigen::Vector3d>> &touchDownPositions3d);

        bool isQpOptimizerEnabled() const { return qpOptimizer_ != nullptr; }


    private:
        struct LockedSwingTrajectory {
            bool active = false;
            bool useBspline = false;
            scalar_t startTime = 0.0;
            scalar_t endTime = 0.0;
            std::shared_ptr<QuinticSplineCpg> referenceSpline;
            std::shared_ptr<BSpline> bspline;

            void reset() {
                active = false;
                useBspline = false;
                startTime = 0.0;
                endTime = 0.0;
                referenceSpline.reset();
                bspline.reset();
            }

            bool contains(scalar_t time) const {
                return active && time >= startTime && time <= endTime;
            }
        };

        /**
         * Extracts for each leg the contact sequence over the motion phase sequence.
         * @param phaseIDsStock
         * @return contactFlagStock
         */
        feet_array_t<std::vector<bool> > extractContactFlags(const std::vector<size_t> &phaseIDsStock) const;

        /**
         * Finds the take-off and touch-down times indices for a specific leg.
         *
         * @param index
         * @param contactFlagStock
         * @return {The take-off time index for swing legs, touch-down time index for swing legs}
         */
        static std::pair<int, int> findIndex(size_t index, const std::vector<bool> &contactFlagStock);

        /**
         * based on the input phaseIDsStock finds the start subsystem and final subsystem of the swing
         * phases of the a foot in each subsystem.
         *
         * startTimeIndexStock: eventTimes[startTimesIndex] will be the take-off time for the requested leg.
         * finalTimeIndexStock: eventTimes[finalTimesIndex] will be the touch-down time for the requested leg.
         *
         * @param [in] footIndex: Foot index
         * @param [in] phaseIDsStock: The sequence of the motion phase IDs.
         * @param [in] contactFlagStock: The sequence of the contact status for the requested leg.
         * @return { startTimeIndexStock, finalTimeIndexStock}
         */
        static std::pair<std::vector<int>, std::vector<int> > updateFootSchedule(
            const std::vector<bool> &contactFlagStock);

        static std::pair<scalar_t, scalar_t> resolveSwingTimeInterval(const scalar_array_t& eventTimes,
                                                                      int swingStartIndex,
                                                                      int swingFinalIndex);

        static int clampPhaseIndex(int index, size_t size);

        /**
         * Check if event time indices are valid
         * @param leg
         * @param index : phase index
         * @param startIndex : liftoff event time index
         * @param finalIndex : touchdown event time index
         * @param phaseIDsStock : mode sequence
         */

        static scalar_t swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale);

        
        scalar_t getRequiredSafeHeightAtPoint(const Eigen::Vector2d& xy,
                             const std::vector<const EdgeSphere*>& nearbySpheres) const;

        TrajectoryCollisionInfo evaluateReferenceTrajectoryCollision(const QuinticSplineCpg& spline,
                                        scalar_t swingStartTime,
                                        scalar_t swingFinalTime,
                                        const Eigen::Vector3d& liftOffPos3d,
                                        const Eigen::Vector3d& touchDownPos3d,
                                        const std::vector<const EdgeSphere*>& nearbySpheres) const;

        void clearExpiredSwingTrajectoryLocks(scalar_t currentTime);

        bool applyLockedSwingTrajectory(size_t leg, size_t phaseIndex);

        void lockReferenceSwingTrajectory(size_t leg, scalar_t startTime, scalar_t endTime,
                         const QuinticSplineCpg& referenceSpline);

        void lockOptimizedSwingTrajectory(size_t leg, scalar_t startTime, scalar_t endTime,
                         const BSpline& bspline);

        const Config config_;
        const TrajectoryCollisionInfo collisionInfo_;
        const size_t numFeet_;


        // feet_array_t<std::vector<SplineCpg> > feetHeightTrajectories_;
        feet_array_t<std::vector<QuinticSplineCpg> > feetHeightTrajectories_;
        feet_array_t<std::vector<BSpline>> feetHeightBsplines_;                             // 优化后的 B 样条轨迹
        feet_array_t<std::vector<bool>> feetUseBSpline_;                                    // 是否使用 B 样条
        feet_array_t<std::vector<std::pair<scalar_t, scalar_t>>> feetBSplineTimeRanges_;    // BSpline 对应的时间区间
        feet_array_t<std::vector<scalar_t> > feetHeightTrajectoriesEvents_;
        feet_array_t<std::vector<bool> > plannedContactFlags_;
        feet_array_t<std::vector<int> > startTimesIndices_;
        feet_array_t<std::vector<int> > finalTimesIndices_;

        // QP 优化器
        std::unique_ptr<SwingTrajectoryQpOptimizer> qpOptimizer_;
        feet_array_t<std::vector<SwingTrajectoryQpOptimizer::OptimizedTrajectory>> optimizedTrajectories_;
        feet_array_t<LockedSwingTrajectory> lockedSwingTrajectories_;

    };

    SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string &fileName,
                                                               const std::string &fieldName = "swing_trajectory_config",
                                                               bool verbose = true);
}
