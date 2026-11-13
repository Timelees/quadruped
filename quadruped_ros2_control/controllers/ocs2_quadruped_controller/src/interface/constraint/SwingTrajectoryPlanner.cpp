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
#include <chrono>
#include <iostream>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/Lookup.h>

#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

namespace ocs2::legged_robot {
    namespace {
        constexpr int kHeightSplineDegree = 5;
        constexpr int kSwingHeightControlPoints = 7;
        constexpr int kStanceHeightControlPoints = 6;

        BSpline createHeightBSpline(const Eigen::VectorXd& zControlPoints, scalar_t startTime, scalar_t finalTime) {
            BSpline spline(kHeightSplineDegree);
            const Eigen::VectorXd zero = Eigen::VectorXd::Zero(zControlPoints.size());
            spline.setControlPoints(zero, zero, zControlPoints);
            spline.setKnots(spline.generateKnots(static_cast<int>(zControlPoints.size()), startTime, finalTime, 0));
            return spline;
        }

    } // namespace

    BSpline createSwingHeightBSpline(scalar_t swingStartTime, scalar_t swingFinalTime,
                                         scalar_t liftOffHeight, scalar_t liftOffVelocity,
                                         scalar_t midHeight,
                                         scalar_t touchDownHeight, scalar_t touchDownVelocity) {
            const scalar_t midTime = 0.5 * (swingStartTime + swingFinalTime);
            const scalar_t degree = static_cast<scalar_t>(kHeightSplineDegree);
            const scalar_t halfDuration = midTime - swingStartTime;
            const scalar_t fullDuration = swingFinalTime - swingStartTime;
            const scalar_t liftOffAcceleration = 0.0;
            const scalar_t touchDownAcceleration = 0.0;

            Eigen::VectorXd zControlPoints(kSwingHeightControlPoints);
            zControlPoints[0] = liftOffHeight;
            zControlPoints[1] = zControlPoints[0] + liftOffVelocity * halfDuration / degree;
            zControlPoints[2] = zControlPoints[1] +
                                fullDuration * (liftOffVelocity + liftOffAcceleration * halfDuration / 4.0) / degree;
            zControlPoints[3] = midHeight;
            zControlPoints[6] = touchDownHeight;
            zControlPoints[5] = zControlPoints[6] - touchDownVelocity * halfDuration / degree;
            zControlPoints[4] = zControlPoints[5] -
                                fullDuration * (touchDownVelocity - touchDownAcceleration * halfDuration / 4.0) / degree;

            BSpline spline = createHeightBSpline(zControlPoints, swingStartTime, swingFinalTime);

            Eigen::VectorXd midTimeVector(1);
            midTimeVector[0] = midTime;
            const Eigen::MatrixXd midBasis = spline.generateSplineBasis(midTimeVector);
            const int midControlPointIndex = kSwingHeightControlPoints / 2;
            const scalar_t middleBasisWeight = midBasis(0, midControlPointIndex);
            if (std::abs(middleBasisWeight) > 1e-12) {
                scalar_t constrainedMidHeight = midHeight;
                for (Eigen::Index i = 0; i < zControlPoints.size(); ++i) {
                    if (i != midControlPointIndex) {
                        constrainedMidHeight -= midBasis(0, i) * zControlPoints[i];
                    }
                }
                zControlPoints[midControlPointIndex] = constrainedMidHeight / middleBasisWeight;
                spline = createHeightBSpline(zControlPoints, swingStartTime, swingFinalTime);
            }

            return spline;
        }

    namespace {
        // Reuse the last policy's complete swing once lift-off has occurred. The
        // per-phase stock covers every swing in the horizon, so a later swing
        // cannot overwrite the executing swing's cache.
        const BSpline* executingSwing(const std::vector<BSpline>& previous,
                                     scalar_t start, scalar_t end, scalar_t initTime) {
            if (initTime < start || initTime >= end) { return nullptr; }
            for (const auto& spline : previous) {
                const auto& knots = spline.getKnots();
                if (knots.size() && std::abs(knots[0] - start) < 1e-9 &&
                    std::abs(knots[knots.size() - 1] - end) < 1e-9) {
                    return &spline;
                }
            }
            return nullptr;
        }

        BSpline createStanceHeightBSpline(scalar_t height) {
            return createHeightBSpline(Eigen::VectorXd::Constant(kStanceHeightControlPoints, height), 0.0, 1.0);
        }
    } // namespace

    SwingTrajectoryPlanner::SwingTrajectoryPlanner(Config config, size_t numFeet) : config_(config),
        numFeet_(numFeet) {
    }

    void SwingTrajectoryPlanner::setTerrainInterface(std::shared_ptr<TerrainInterface> terrainInterface) {
        auto qpConfig = config_.terrainQpConfig;
        qpConfig.footRadius = config_.footRadius;
        swingTrajectoryQpOptimizer_ = std::make_shared<SwingTrajectoryQpOptimizer>(
            std::move(qpConfig), std::move(terrainInterface));
    }

    vector3_t SwingTrajectoryPlanner::getVelocityConstraint(size_t leg, scalar_t time) const {
        const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
        return feetHeightTrajectories_[leg][index].getVelocity(time);
    }

    vector3_t SwingTrajectoryPlanner::getPositionConstraint(size_t leg, scalar_t time) const {
        const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
        vector3_t position = feetHeightTrajectories_[leg][index].getPosition(time);
        position.z() += config_.footRadius;
        return position;
    }

    scalar_t SwingTrajectoryPlanner::getZvelocityConstraint(size_t leg, scalar_t time) const {
        return getVelocityConstraint(leg, time).z();
    }

    scalar_t SwingTrajectoryPlanner::getZpositionConstraint(size_t leg, scalar_t time) const {
        const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
        return feetHeightTrajectories_[leg][index].getPosition(time).z();
    }

    void SwingTrajectoryPlanner::update(const ModeSchedule &modeSchedule, scalar_t terrainHeight) {
        const scalar_array_t terrainHeightSequence(modeSchedule.modeSequence.size(), terrainHeight);
        feet_array_t<scalar_array_t> liftOffHeightSequence;
        liftOffHeightSequence.fill(terrainHeightSequence);
        feet_array_t<scalar_array_t> touchDownHeightSequence;
        touchDownHeightSequence.fill(terrainHeightSequence);
        update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
    }


    void SwingTrajectoryPlanner::update(const ModeSchedule &modeSchedule,
                                        const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                                        const feet_array_t<scalar_array_t> &touchDownHeightSequence,
                                        scalar_t initTime) {
        scalar_array_t heightSequence(modeSchedule.modeSequence.size());
        feet_array_t<scalar_array_t> maxHeightSequence;
        for (size_t j = 0; j < numFeet_; j++) {
            for (int p = 0; p < modeSchedule.modeSequence.size(); ++p) {
                heightSequence[p] = std::max(liftOffHeightSequence[j][p], touchDownHeightSequence[j][p]);
            }
            maxHeightSequence[j] = heightSequence;
        }
        update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence, initTime);
    }


    void SwingTrajectoryPlanner::update(const ModeSchedule &modeSchedule,
                                        const feet_array_t<scalar_array_t> &liftOffHeightSequence,
                                        const feet_array_t<scalar_array_t> &touchDownHeightSequence,
                                        const feet_array_t<scalar_array_t> &maxHeightSequence,
                                        scalar_t initTime) {
        const auto &modeSequence = modeSchedule.modeSequence;
        const auto &eventTimes = modeSchedule.eventTimes;

        const auto eesContactFlagStocks = extractContactFlags(modeSequence);

        feet_array_t<std::vector<int> > startTimesIndices;
        feet_array_t<std::vector<int> > finalTimesIndices;
        for (size_t leg = 0; leg < numFeet_; leg++) {
            std::tie(startTimesIndices[leg], finalTimesIndices[leg]) =
                    updateFootSchedule(eesContactFlagStocks[leg]);
        }

        for (size_t j = 0; j < numFeet_; j++) {
            const auto previous = std::move(feetHeightTrajectories_[j]);
            feetHeightTrajectories_[j].clear();
            feetHeightTrajectories_[j].reserve(modeSequence.size());
            for (int p = 0; p < modeSequence.size(); ++p) {
                if (!eesContactFlagStocks[j][p]) {
                    // for a swing leg
                    const int swingStartIndex = startTimesIndices[j][p];
                    const int swingFinalIndex = finalTimesIndices[j][p];
                    checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence);

                    const scalar_t swingStartTime = eventTimes[swingStartIndex];
                    const scalar_t swingFinalTime = eventTimes[swingFinalIndex];
                    if (const auto* frozen = executingSwing(previous, swingStartTime, swingFinalTime, initTime)) {
                        feetHeightTrajectories_[j].push_back(*frozen);
                        continue;
                    }

                    const scalar_t scaling = swingTrajectoryScaling(swingStartTime, swingFinalTime,
                                                                    config_.swingTimeScale);

                    const scalar_t midHeight = maxHeightSequence[j][p] + scaling * config_.swingHeight;
                    feetHeightTrajectories_[j].push_back(createSwingHeightBSpline(
                        swingStartTime, swingFinalTime,
                        liftOffHeightSequence[j][p], scaling * config_.liftOffVelocity,
                        midHeight,
                        touchDownHeightSequence[j][p], scaling * config_.touchDownVelocity));
                } else {
                    // for a stance leg
                    feetHeightTrajectories_[j].push_back(createStanceHeightBSpline(liftOffHeightSequence[j][p]));
                }
            }
            feetHeightTrajectoriesEvents_[j] = eventTimes;
        }
    }

    void SwingTrajectoryPlanner::updateTerrain(
        const ModeSchedule &modeSchedule,
        const feet_array_t<scalar_array_t> &liftOffHeightSequence,
        const feet_array_t<scalar_array_t> &touchDownHeightSequence,
        const feet_array_t<std::vector<Eigen::Vector3d>> &liftOffPositionSequence,
        const feet_array_t<std::vector<Eigen::Vector3d>> &touchDownPositionSequence,
        scalar_t initTime, scalar_t finalTime) {
        scalar_array_t heightSequence(modeSchedule.modeSequence.size());
        feet_array_t<scalar_array_t> maxHeightSequence;
        for (size_t j = 0; j < numFeet_; j++) {
            for (int p = 0; p < modeSchedule.modeSequence.size(); ++p) {
                heightSequence[p] = std::max(liftOffHeightSequence[j][p], touchDownHeightSequence[j][p]);
            }
            maxHeightSequence[j] = heightSequence;
        }
        updateTerrain(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence,
                  liftOffPositionSequence, touchDownPositionSequence, initTime, finalTime);
    }

    void SwingTrajectoryPlanner::updateTerrain(
        const ModeSchedule &modeSchedule,
        const feet_array_t<scalar_array_t> &liftOffHeightSequence,
        const feet_array_t<scalar_array_t> &touchDownHeightSequence,
        const feet_array_t<scalar_array_t> &maxHeightSequence,
        const feet_array_t<std::vector<Eigen::Vector3d>> &liftOffPositionSequence,
        const feet_array_t<std::vector<Eigen::Vector3d>> &touchDownPositionSequence,
        scalar_t initTime, scalar_t finalTime) {
        if (!config_.useTerrainPlanning) {
            update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence, initTime);
            return;
        }
        if (!swingTrajectoryQpOptimizer_) {
            throw std::logic_error("Swing terrain-QP is enabled without a terrain interface.");
        }

        const auto &modeSequence = modeSchedule.modeSequence;
        const auto &eventTimes = modeSchedule.eventTimes;
        const auto eesContactFlagStocks = extractContactFlags(modeSequence);

        feet_array_t<std::vector<int> > startTimesIndices;
        feet_array_t<std::vector<int> > finalTimesIndices;
        for (size_t leg = 0; leg < numFeet_; leg++) {
            std::tie(startTimesIndices[leg], finalTimesIndices[leg]) =
                    updateFootSchedule(eesContactFlagStocks[leg]);
        }

        for (size_t j = 0; j < numFeet_; j++) {
            const auto previous = std::move(feetHeightTrajectories_[j]);
            feetHeightTrajectories_[j].clear();
            feetHeightTrajectories_[j].reserve(modeSequence.size());
            for (int p = 0; p < modeSequence.size(); ++p) {
                // 同一次更新中，连续非接触模式属于同一完整摆动段，只求解一次。
                if (p > 0 && !eesContactFlagStocks[j][p] && !eesContactFlagStocks[j][p - 1]) {
                    feetHeightTrajectories_[j].push_back(feetHeightTrajectories_[j].back());
                    continue;
                }
                if (!eesContactFlagStocks[j][p]) {
                    const int swingStartIndex = startTimesIndices[j][p];
                    const int swingFinalIndex = finalTimesIndices[j][p];
                    checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence);

                    const scalar_t swingStartTime = eventTimes[swingStartIndex];
                    const scalar_t swingFinalTime = eventTimes[swingFinalIndex];
                    if (const auto* frozen = executingSwing(previous, swingStartTime, swingFinalTime, initTime)) {
                        feetHeightTrajectories_[j].push_back(*frozen);
                        continue;
                    }
                    const scalar_t scaling = swingTrajectoryScaling(swingStartTime, swingFinalTime,
                                                                    config_.swingTimeScale);
                    const scalar_t midHeight = maxHeightSequence[j][p] + scaling * config_.swingHeight;
                    BSpline referenceSpline = createSwingHeightBSpline(
                        swingStartTime, swingFinalTime,
                        liftOffHeightSequence[j][p], scaling * config_.liftOffVelocity,
                        midHeight,
                        touchDownHeightSequence[j][p], scaling * config_.touchDownVelocity);

                    if (swingFinalTime < initTime || swingStartTime > finalTime) {
                        // OCS2 keeps padding modes outside the policy horizon. They need no QP.
                        feetHeightTrajectories_[j].push_back(referenceSpline);
                        continue;
                    }
                    Eigen::Vector3d liftOff = liftOffPositionSequence[j].at(static_cast<size_t>(p));
                    Eigen::Vector3d touchDown = touchDownPositionSequence[j].at(static_cast<size_t>(p));
                    liftOff.z() = liftOffHeightSequence[j][p];
                    touchDown.z() = touchDownHeightSequence[j][p];
                    const auto optimizationStart = std::chrono::steady_clock::now();
                    const auto optimized = swingTrajectoryQpOptimizer_->optimizeSwingTrajectory(
                        referenceSpline, swingStartTime, swingFinalTime, liftOff, touchDown,
                        workspace_ ? SwingTrajectoryQpOptimizer::HeightBounds([&, j](double t, const Eigen::Vector3d& position) {
                            return workspace_(j, t, position);
                        }) : SwingTrajectoryQpOptimizer::HeightBounds{});
                    if (config_.terrainQpConfig.verbose) {
                        std::cerr << "Swing planning ms=" << std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - optimizationStart).count() << '\n';
                    }
                    if (!optimized.valid()) {
                        // guardrail: QP infeasible or output certification failed.
                        // Fall back to the reference spline rather than aborting the
                        // controller — the reference is the pre-terrain-plan baseline
                        // and is always available.
                        if (config_.terrainQpConfig.verbose) {
                            std::cerr << "Swing terrain-QP fallback for leg " << j
                                      << " at " << std::to_string(initTime) << " in ["
                                      << std::to_string(swingStartTime) << ", "
                                      << std::to_string(swingFinalTime) << "].\n";
                        }
                        feetHeightTrajectories_[j].push_back(referenceSpline);
                    } else {
                        feetHeightTrajectories_[j].push_back(*optimized.bspline);

                    }
                } else {
                    feetHeightTrajectories_[j].push_back(createStanceHeightBSpline(liftOffHeightSequence[j][p]));
                }
            }
            feetHeightTrajectoriesEvents_[j] = eventTimes;
        }
    }


    std::pair<std::vector<int>, std::vector<int> > SwingTrajectoryPlanner::updateFootSchedule(
        const std::vector<bool> &contactFlagStock) {
        const size_t numPhases = contactFlagStock.size();

        std::vector<int> startTimeIndexStock(numPhases, 0);
        std::vector<int> finalTimeIndexStock(numPhases, 0);

        // find the startTime and finalTime indices for swing feet
        for (size_t i = 0; i < numPhases; i++) {
            if (!contactFlagStock[i]) {
                std::tie(startTimeIndexStock[i], finalTimeIndexStock[i]) = findIndex(i, contactFlagStock);
            }
        }
        return {startTimeIndexStock, finalTimeIndexStock};
    }


    feet_array_t<std::vector<bool> > SwingTrajectoryPlanner::extractContactFlags(
        const std::vector<size_t> &phaseIDsStock) const {
        const size_t numPhases = phaseIDsStock.size();

        feet_array_t<std::vector<bool> > contactFlagStock;
        std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

        for (size_t i = 0; i < numPhases; i++) {
            const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
            for (size_t j = 0; j < numFeet_; j++) {
                contactFlagStock[j][i] = contactFlag[j];
            }
        }
        return contactFlagStock;
    }


    std::pair<int, int> SwingTrajectoryPlanner::findIndex(size_t index, const std::vector<bool> &contactFlagStock) {
        const size_t numPhases = contactFlagStock.size();

        // skip if it is a stance leg
        if (contactFlagStock[index]) {
            return {0, 0};
        }

        // find the starting time
        int startTimesIndex = -1;
        for (int ip = index - 1; ip >= 0; ip--) {
            if (contactFlagStock[ip]) {
                startTimesIndex = ip;
                break;
            }
        }

        // find the final time
        int finalTimesIndex = numPhases - 1;
        for (size_t ip = index + 1; ip < numPhases; ip++) {
            if (contactFlagStock[ip]) {
                finalTimesIndex = ip - 1;
                break;
            }
        }

        return {startTimesIndex, finalTimesIndex};
    }


    void SwingTrajectoryPlanner::checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex,
                                                          const std::vector<size_t> &phaseIDsStock) {
        const size_t numSubsystems = phaseIDsStock.size();
        if (startIndex < 0) {
            std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
            for (size_t i = 0; i < numSubsystems; i++) {
                std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
            }
            std::cerr << std::endl;

            throw std::runtime_error(
                "The time of take-off for the first swing of the EE with ID " + std::to_string(leg) +
                " is not defined.");
        }
        if (finalIndex >= numSubsystems - 1) {
            std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
            for (size_t i = 0; i < numSubsystems; i++) {
                std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
            }
            std::cerr << std::endl;

            throw std::runtime_error(
                "The time of touch-down for the last swing of the EE with ID " + std::to_string(leg) +
                " is not defined.");
        }
    }


    scalar_t SwingTrajectoryPlanner::swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime,
                                                            scalar_t swingTimeScale) {
        return std::min(1.0, (finalTime - startTime) / swingTimeScale);
    }

    SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string &fileName,
                                                               const std::string &fieldName, bool verbose) {
        boost::property_tree::ptree pt;
        read_info(fileName, pt);

        if (verbose) {
            std::cerr << "\n #### Swing Trajectory Config:";
            std::cerr << "\n #### =============================================================================\n";
        }

        SwingTrajectoryPlanner::Config config;
        const std::string prefix = fieldName + ".";

        loadData::loadPtreeValue(pt, config.liftOffVelocity, prefix + "liftOffVelocity", verbose);
        loadData::loadPtreeValue(pt, config.touchDownVelocity, prefix + "touchDownVelocity", verbose);
        loadData::loadPtreeValue(pt, config.swingHeight, prefix + "swingHeight", verbose);
        loadData::loadPtreeValue(pt, config.swingTimeScale, prefix + "swingTimeScale", verbose);

        auto loadOptional = [&](auto& value, const std::string& key) {
            using ValueType = std::decay_t<decltype(value)>;
            if (const auto loaded = pt.get_optional<ValueType>(prefix + key)) {
                value = *loaded;
                if (verbose) {
                    std::cerr << prefix + key << ": " << value << std::endl;
                }
            }
        };

        loadOptional(config.useTerrainPlanning, "useTerrainPlanning");
        loadOptional(config.footRadius, "footRadius");
        loadOptional(config.trackingPositionErrorGain, "trackingPositionErrorGain");
        loadOptional(config.terrainQpConfig.numControlPoints, "terrainNumControlPoints");
        loadOptional(config.terrainQpConfig.numCurveSamples, "terrainNumCurveSamples");
        loadOptional(config.terrainQpConfig.controlPointTrackingWeight, "terrainControlPointTrackingWeight");
        loadOptional(config.terrainQpConfig.positionTrackingWeight, "terrainPositionTrackingWeight");
        loadOptional(config.terrainQpConfig.velocityTrackingWeight, "terrainVelocityTrackingWeight");
        loadOptional(config.terrainQpConfig.accelerationTrackingWeight, "terrainAccelerationTrackingWeight");
        loadOptional(config.terrainQpConfig.jerkSmoothnessWeight, "terrainJerkSmoothnessWeight");
        loadOptional(config.terrainQpConfig.groundClearance, "terrainGroundClearance");
        loadOptional(config.terrainQpConfig.qpMaxWorkingSetRecalculations, "terrainQpMaxWorkingSetRecalculations");
        loadOptional(config.terrainQpConfig.hessianRegularization, "terrainHessianRegularization");
        loadOptional(config.terrainQpConfig.elevationLayer, "terrainElevationLayer");
        loadOptional(config.terrainQpConfig.verbose, "terrainVerbose");
        int geometryMode=static_cast<int>(config.terrainQpConfig.geometryMode);
        int hullBasis=static_cast<int>(config.terrainQpConfig.collisionCheck.basis);
        loadOptional(geometryMode, "terrainGeometryMode");
        loadOptional(hullBasis, "collisionHullBasis");
        if(geometryMode<0 || geometryMode>2 || hullBasis<0 || hullBasis>1) {
            throw std::invalid_argument("Invalid terrain geometry/hull mode.");
        }
        config.terrainQpConfig.geometryMode=static_cast<SwingGeometryMode>(geometryMode);
        config.terrainQpConfig.collisionCheck.basis=static_cast<HullBasis>(hullBasis);
        loadOptional(config.terrainQpConfig.subdivisionsPerSpan, "terrainSubdivisionsPerSpan");
        loadOptional(config.terrainQpConfig.checkReference, "checkSwingReference");
        loadOptional(config.terrainQpConfig.verifyOutput, "verifySwingOutput");
        loadOptional(config.terrainQpConfig.collisionCheck.maxDepth, "collisionMaxDepth");
        loadOptional(config.terrainQpConfig.collisionCheck.maxSegments, "collisionMaxSegments");

        if (verbose) {
            std::cerr << " #### =============================================================================" <<
                    std::endl;
        }

        return config;
    }
} // namespace ocs2::legged_robot
