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

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "ocs2_quadruped_controller/interface/constraint/SwingTrajectoryPlanner.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <utility>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/Lookup.h>

#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

/* 规划摆动相足端轨迹 */
namespace ocs2::legged_robot
{

    SwingTrajectoryPlanner::SwingTrajectoryPlanner(Config config, size_t numFeet) : config_(config),
                                                                                    numFeet_(numFeet)
    {
    }

    scalar_t SwingTrajectoryPlanner::getZvelocityConstraint(size_t leg, scalar_t time) const
    {
        const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
        return feetHeightTrajectories_[leg][index].velocity(time);
    }

    scalar_t SwingTrajectoryPlanner::getZpositionConstraint(size_t leg, scalar_t time) const
    {
        const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
        return feetHeightTrajectories_[leg][index].position(time);
    }

    contact_flag_t SwingTrajectoryPlanner::getContactFlags(scalar_t time) const
    {
        contact_flag_t contactFlags{};
        for (size_t leg = 0; leg < numFeet_; ++leg)
        {
            if (plannedContactFlags_[leg].empty())
            {
                contactFlags[leg] = false;
                continue;
            }
            const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
            const int clampedIndex = clampPhaseIndex(index, plannedContactFlags_[leg].size());
            contactFlags[leg] = plannedContactFlags_[leg][static_cast<size_t>(clampedIndex)];
        }
        return contactFlags;
    }

    // 解析摆动相时间区间，确保起止时间有效且满足最小摆动时间要求
    scalar_t SwingTrajectoryPlanner::getSwingPhaseProgress(size_t leg, scalar_t time) const
    {
        if (leg >= numFeet_ || plannedContactFlags_[leg].empty() || feetHeightTrajectoriesEvents_[leg].empty())
        {
            return scalar_t(-1.0);
        }

        const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
        const int clampedIndex = clampPhaseIndex(index, plannedContactFlags_[leg].size());
        if (clampedIndex < 0 || plannedContactFlags_[leg][static_cast<size_t>(clampedIndex)])
        {
            return scalar_t(-1.0);
        }

        const auto [swingStartTime, swingFinalTime] = resolveSwingTimeInterval(
            feetHeightTrajectoriesEvents_[leg], startTimesIndices_[leg][static_cast<size_t>(clampedIndex)],
            finalTimesIndices_[leg][static_cast<size_t>(clampedIndex)]);

        if (swingFinalTime <= swingStartTime)
        {
            return scalar_t(-1.0);
        }

        const scalar_t alpha = (time - swingStartTime) / (swingFinalTime - swingStartTime);
        return std::min<scalar_t>(1.0, std::max<scalar_t>(0.0, alpha));
    }

    void SwingTrajectoryPlanner::update(const ModeSchedule &modeSchedule, scalar_t terrainHeight)
    {
        const scalar_array_t terrainHeightSequence(modeSchedule.modeSequence.size(), terrainHeight);
        feet_array_t<scalar_array_t> liftOffHeightSequence;
        liftOffHeightSequence.fill(terrainHeightSequence);
        feet_array_t<scalar_array_t> touchDownHeightSequence;
        touchDownHeightSequence.fill(terrainHeightSequence);
        update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
    }

    void SwingTrajectoryPlanner::update(const ModeSchedule &modeSchedule,
                                        const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                                        const feet_array_t<scalar_array_t> &touchDownHeightSequence)
    {
        scalar_array_t heightSequence(modeSchedule.modeSequence.size());
        feet_array_t<scalar_array_t> maxHeightSequence;
        for (size_t j = 0; j < numFeet_; j++)
        {
            for (size_t p = 0; p < modeSchedule.modeSequence.size(); ++p)
            {
                heightSequence[p] = std::max(liftOffHeightSequence[j][p], touchDownHeightSequence[j][p]);
            }
            maxHeightSequence[j] = heightSequence;
        }
        update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence);
    }

    void SwingTrajectoryPlanner::update(const ModeSchedule &modeSchedule,
                                        const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                                        const feet_array_t<scalar_array_t> &touchDownHeightSequence,
                                        const feet_array_t<scalar_array_t> &maxHeightSequence)
    {
        const auto &modeSequence = modeSchedule.modeSequence;
        const auto &eventTimes = modeSchedule.eventTimes;
        const auto eesContactFlagStocks = extractContactFlags(modeSequence);
        plannedContactFlags_ = eesContactFlagStocks;

        
        feet_array_t<std::vector<int>> startTimesIndices;
        feet_array_t<std::vector<int>> finalTimesIndices;
        for (size_t leg = 0; leg < numFeet_; leg++)
        {
            std::tie(startTimesIndices[leg], finalTimesIndices[leg]) =
                updateFootSchedule(eesContactFlagStocks[leg]);
        }
        startTimesIndices_ = startTimesIndices;
        finalTimesIndices_ = finalTimesIndices;

        for (size_t j = 0; j < numFeet_; j++)
        {
            feetHeightTrajectories_[j].clear();
            feetHeightTrajectories_[j].reserve(modeSequence.size());
            for (size_t p = 0; p < modeSequence.size(); ++p)
            {
                if (!eesContactFlagStocks[j][p])
                {
                    const int swingStartIndex = startTimesIndices[j][p];
                    const int swingFinalIndex = finalTimesIndices[j][p];
                    const scalar_t swingStartTime = eventTimes[swingStartIndex];
                    const scalar_t swingFinalTime = eventTimes[swingFinalIndex];

                    const scalar_t scaling = swingTrajectoryScaling(swingStartTime, swingFinalTime, config_.swingTimeScale);

                    const CubicSpline::Node liftOff{ swingStartTime, liftOffHeightSequence[j][p], scaling * config_.liftOffVelocity };
                    const CubicSpline::Node touchDown{ swingFinalTime, touchDownHeightSequence[j][p], scaling * config_.touchDownVelocity };
                    const scalar_t midHeight = maxHeightSequence[j][p] + scaling * config_.swingHeight;
                    feetHeightTrajectories_[j].emplace_back(liftOff, midHeight, touchDown);
                }
                else
                {
                    const CubicSpline::Node liftOff{0.0, liftOffHeightSequence[j][p], 0.0};
                    const CubicSpline::Node touchDown{1.0, liftOffHeightSequence[j][p], 0.0};
                    feetHeightTrajectories_[j].emplace_back(liftOff, liftOffHeightSequence[j][p], touchDown);
                }
            }
            feetHeightTrajectoriesEvents_[j] = eventTimes;
        }
    }

    // 摆动相起始时间和结束时间
    std::pair<std::vector<int>, std::vector<int>> SwingTrajectoryPlanner::updateFootSchedule(
        const std::vector<bool> &contactFlagStock)
    {
        const size_t numPhases = contactFlagStock.size();

        std::vector<int> startTimeIndexStock(numPhases, 0);
        std::vector<int> finalTimeIndexStock(numPhases, 0);

        // find the startTime and finalTime indices for swing feet
        for (size_t i = 0; i < numPhases; i++)
        {
            if (!contactFlagStock[i])
            {
                std::tie(startTimeIndexStock[i], finalTimeIndexStock[i]) = findIndex(i, contactFlagStock);
            }
        }
        return {startTimeIndexStock, finalTimeIndexStock};
    }

    feet_array_t<std::vector<bool>> SwingTrajectoryPlanner::extractContactFlags(
        const std::vector<size_t> &phaseIDsStock) const
    {
        const size_t numPhases = phaseIDsStock.size();

        feet_array_t<std::vector<bool>> contactFlagStock;
        std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

        for (size_t i = 0; i < numPhases; i++)
        {
            const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
            for (size_t j = 0; j < numFeet_; j++)
            {
                contactFlagStock[j][i] = contactFlag[j];
            }
        }
        return contactFlagStock;
    }

    std::pair<int, int> SwingTrajectoryPlanner::findIndex(size_t index, const std::vector<bool> &contactFlagStock)
    {
        const size_t numPhases = contactFlagStock.size();

        // skip if it is a stance leg
        if (contactFlagStock[index])
        {
            return {0, 0};
        }

        // find the starting time
        const int currentIndex = static_cast<int>(index);
        int startTimesIndex = -1;
        for (int ip = currentIndex - 1; ip >= 0; ip--)
        {
            if (contactFlagStock[ip])
            {
                startTimesIndex = ip;
                break;
            }
        }

        // find the final time
        int finalTimesIndex = static_cast<int>(numPhases) - 1;
        for (int ip = currentIndex + 1; ip < static_cast<int>(numPhases); ip++)
        {
            if (contactFlagStock[static_cast<size_t>(ip)])
            {
                finalTimesIndex = ip - 1;
                break;
            }
        }

        return {startTimesIndex, finalTimesIndex};
    }

    std::pair<scalar_t, scalar_t> SwingTrajectoryPlanner::resolveSwingTimeInterval(const scalar_array_t &eventTimes,
                                                                                    int swingStartIndex,
                                                                                    int swingFinalIndex)
    {
        if (eventTimes.empty())
        {
            return {scalar_t(0.0), scalar_t(1e-3)};
        }

        const int firstValidIndex = 0;
        const int lastValidIndex = static_cast<int>(eventTimes.size()) - 1;
        const int clampedStartIndex = std::min(std::max(swingStartIndex, firstValidIndex), lastValidIndex);
        const int clampedFinalIndex = std::min(std::max(swingFinalIndex, firstValidIndex), lastValidIndex);

        const scalar_t swingStartTime = eventTimes[clampedStartIndex];
        const scalar_t rawFinalTime = eventTimes[clampedFinalIndex];
        const scalar_t swingFinalTime = (rawFinalTime > swingStartTime)
                                            ? rawFinalTime
                                            : swingStartTime + scalar_t(1e-3);
        return {swingStartTime, swingFinalTime};
    }

    int SwingTrajectoryPlanner::clampPhaseIndex(int index, size_t size)
    {
        if (size == 0)
        {
            return -1;
        }
        return std::min(std::max(index, 0), static_cast<int>(size) - 1);
    }



    scalar_t SwingTrajectoryPlanner::swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime,
                                                            scalar_t swingTimeScale)
    {
        return std::min(1.0, (finalTime - startTime) / swingTimeScale);
    }

    BSpline SwingTrajectoryPlanner::createSwingHeightBSpline(scalar_t swingStartTime,
                                                             scalar_t swingFinalTime,
                                                             scalar_t liftOffHeight,
                                                             scalar_t touchDownHeight,
                                                             scalar_t liftOffVelocity,
                                                             scalar_t touchDownVelocity,
                                                             scalar_t midHeight)
    {
        constexpr int degree = 3;
        constexpr int numControlPoints = 7;
        const scalar_t duration = swingFinalTime - swingStartTime;

        Eigen::VectorXd x = Eigen::VectorXd::LinSpaced(numControlPoints, 0.0, 1.0);
        Eigen::VectorXd y = Eigen::VectorXd::Zero(numControlPoints);
        Eigen::VectorXd z(numControlPoints);

        const scalar_t liftOffSlopeU = liftOffVelocity * duration;
        const scalar_t touchDownSlopeU = touchDownVelocity * duration;
        const scalar_t startHandle = liftOffHeight + liftOffSlopeU / 12.0;
        const scalar_t finalHandle = touchDownHeight - touchDownSlopeU / 12.0;

        z[0] = liftOffHeight;
        z[1] = startHandle;
        const scalar_t shoulderHeight = 0.5 * (midHeight + std::max(startHandle, finalHandle));
        z[2] = shoulderHeight;
        z[4] = shoulderHeight;
        z[5] = finalHandle;
        z[6] = touchDownHeight;

        // 让 u=0.5 处穿过期望摆高且中点速度为 0，接近原两段样条的摆动峰值特性。
        z[3] = 1.5 * midHeight - 0.25 * (z[2] + z[4]);

        BSpline spline(degree);
        spline.setControlPoints(x, y, z);
        spline.setKnots(spline.generateKnots(numControlPoints, 0.0, 1.0, 0));
        return spline;
    }

    scalar_t SwingTrajectoryPlanner::getRequiredSafeHeightAtPoint(
        const Eigen::Vector2d &xy,
        const std::vector<const EdgeSphere *> &nearbySpheres) const
    {
        scalar_t requiredHeight = -std::numeric_limits<scalar_t>::infinity();

        if (qpOptimizer_ && qpOptimizer_->terrainInterface_)
        {
            requiredHeight = static_cast<scalar_t>(qpOptimizer_->terrainInterface_->getHeight(xy.x(), xy.y()));
        }

        const scalar_t inflatedMargin = std::max<scalar_t>(0.0, config_.safetyMargin);
        for (const auto *sphere : nearbySpheres)
        {
            if (sphere == nullptr)
            {
                continue;
            }

            const Eigen::Vector2d deltaXY = xy - sphere->center.head<2>();
            const scalar_t inflatedRadius = static_cast<scalar_t>(sphere->radius) + inflatedMargin;
            const scalar_t radialDistanceSquared = deltaXY.squaredNorm();
            const scalar_t radiusSquared = inflatedRadius * inflatedRadius;
            if (radialDistanceSquared >= radiusSquared)
            {
                continue;
            }

            const scalar_t topHeight = static_cast<scalar_t>(sphere->center.z()) +
                                       std::sqrt(std::max<scalar_t>(0.0, radiusSquared - radialDistanceSquared));
            requiredHeight = std::max(requiredHeight, topHeight);
        }

        if (!std::isfinite(requiredHeight))
        {
            return 0.0;
        }
        return requiredHeight;
    }




    SwingTrajectoryPlanner::TrajectoryCollisionInfo SwingTrajectoryPlanner::evaluateReferenceTrajectoryCollision(
        const BSpline &spline,
        scalar_t swingStartTime,
        scalar_t swingFinalTime,
        const Eigen::Vector3d &liftOffPos3d,
        const Eigen::Vector3d &touchDownPos3d,
        const std::vector<const EdgeSphere *> &nearbySpheres) const
    {
        TrajectoryCollisionInfo result;

        if (!qpOptimizer_ || swingFinalTime <= swingStartTime)
        {
            return result;
        }

        const int nSamples = std::max(2, collisionInfo_.nSamples);
        bool evaluatedSphereClearance = false;

        for (int sample = 0 + qpOptimizer_->config_.ignoreSamples; sample <= nSamples - qpOptimizer_->config_.ignoreSamples; ++sample) // 忽略起始和结束的几个采样点，避免过于接近地面或触地点导致的数值问题
        {
            const scalar_t alpha = static_cast<scalar_t>(sample) / static_cast<scalar_t>(nSamples);
            const Eigen::Vector3d xyPoint = liftOffPos3d + alpha * (touchDownPos3d - liftOffPos3d);
            const Eigen::Vector2d xy = xyPoint.head<2>();
            const scalar_t splineHeight = static_cast<scalar_t>(spline.getPosition(alpha).z());

            const scalar_t terrainHeight = static_cast<scalar_t>(qpOptimizer_->terrainInterface_->getHeight(xy.x(), xy.y()));
            // std::cout << "[SwingTrajectoryPlanner] 评估轨迹碰撞----Sample " << sample << "/" << nSamples 
            //           << ", Time = " << time << ", SplineHeight = " << splineHeight 
            //           << ", TerrainHeight = " << terrainHeight << std::endl;

            const scalar_t terrainClearance = splineHeight - terrainHeight;
        
            result.minTerrainClearance = std::min(result.minTerrainClearance, static_cast<double>(terrainClearance));
            result.collidesWithTerrain = (terrainClearance < 1e-9);

            // std::cout << "[SwingTrajectoryPlanner] 评估轨迹碰撞----Sample " << sample << "/" << nSamples 
            //           << ", Terrain Clearance = " << terrainClearance 
            //           << ", CollidesWithTerrain = " << result.collidesWithTerrain << std::endl;

            const scalar_t requiredHeight = getRequiredSafeHeightAtPoint(xy, nearbySpheres);
            result.maxHeightDeficit = std::max(result.maxHeightDeficit,
                                               static_cast<double>(requiredHeight - splineHeight));

            const Eigen::Vector3d samplePosition(xy.x(), xy.y(), splineHeight);
            for (const auto *sphere : nearbySpheres)
            {
                if (sphere == nullptr)
                {
                    // std::cout <<"[SwingTrajectoryPlanner] 评估轨迹碰撞----Sample " << sample << "/" << nSamples 
                    //           << ", 距离边界球过远，忽略碰撞检查" << std::endl;
                    continue;
                }

                evaluatedSphereClearance = true;
                const scalar_t inflatedRadius = static_cast<scalar_t>(sphere->radius) + config_.safetyMargin;
                const scalar_t sphereClearance =
                    static_cast<scalar_t>((samplePosition - sphere->center).norm()) - inflatedRadius;
                result.minEdgeSphereClearance =
                    std::min(result.minEdgeSphereClearance, static_cast<double>(sphereClearance));
                result.collidesWithEdgeSphere = (sphereClearance < -1e-6);
                // std::cout << "[SwingTrajectoryPlanner] 评估轨迹碰撞----Sample " << sample << "/" << nSamples 
                //           << ", Sphere Center = (" << sphere->center.x() << ", " << sphere->center.y() << ", " << sphere->center.z() 
                //           << "), Inflated Radius = " << inflatedRadius 
                //           << ", Sphere Clearance = " << sphereClearance 
                //           << ", CollidesWithEdgeSphere = " << result.collidesWithEdgeSphere << std::endl;
            }
        }

        if (!evaluatedSphereClearance)
        {
            result.minEdgeSphereClearance = std::numeric_limits<double>::infinity();
        }
        result.maxHeightDeficit = std::max(0.0, result.maxHeightDeficit);
        return result;
    }


    SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string &fileName,
                                                               const std::string &fieldName, bool verbose)
    {
        boost::property_tree::ptree pt;
        read_info(fileName, pt);

        if (verbose)
        {
            std::cerr << "\n #### Swing Trajectory Config:";
            std::cerr << "\n #### =============================================================================\n";
        }

        SwingTrajectoryPlanner::Config config;
        const std::string prefix = fieldName + ".";

        loadData::loadPtreeValue(pt, config.liftOffVelocity, prefix + "liftOffVelocity", verbose);
        loadData::loadPtreeValue(pt, config.touchDownVelocity, prefix + "touchDownVelocity", verbose);
        loadData::loadPtreeValue(pt, config.swingHeight, prefix + "swingHeight", verbose);
        loadData::loadPtreeValue(pt, config.swingTimeScale, prefix + "swingTimeScale", verbose);
        // loadData::loadPtreeValue(pt, config.safetyMargin, prefix + "safetyMargin", verbose);
        if (verbose)
        {
            std::cerr << " #### =============================================================================" << std::endl;
        }

        return config;
    }

    // ========================================================================
    // CLF-CBF QP 集成
    // ========================================================================

    void SwingTrajectoryPlanner::setQpOptimizer(
        std::shared_ptr<TerrainInterface> terrainInterface,
        const SwingTrajectoryQpConfig &qpConfig)
    {
        qpOptimizer_ = std::make_unique<SwingTrajectoryQpOptimizer>(qpConfig, std::move(terrainInterface));
    }

    void SwingTrajectoryPlanner::setEdgeSpheres(std::shared_ptr<EdgeSphereArray> edgeSpheresPtr)
    {
        if (qpOptimizer_)
        {
            qpOptimizer_->setEdgeSpheres(std::move(edgeSpheresPtr));
        }
    }

    void SwingTrajectoryPlanner::updateWithQpOptimization(
        const ModeSchedule &modeSchedule,
        scalar_t currentTime,
        const feet_array_t<scalar_array_t> &liftOffHeightSequence,
        const feet_array_t<scalar_array_t> &touchDownHeightSequence,
        const feet_array_t<std::vector<Eigen::Vector3d>> &liftOffPositions3d,
        const feet_array_t<std::vector<Eigen::Vector3d>> &touchDownPositions3d)
    {
        (void)currentTime;
        (void)liftOffPositions3d;
        (void)touchDownPositions3d;
        // In blind-walking mode we fall back to the baseline swing-height planner.
        update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
    }

} // namespace ocs2::legged_robot
