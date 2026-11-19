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
                if (knots.size() && knots[0] <= initTime &&
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
        numFeet_(numFeet), replanner_(config_.replanConfig) {
    }

    void SwingTrajectoryPlanner::setTerrainInterface(std::shared_ptr<TerrainInterface> terrainInterface) {
        terrainInterface_ = terrainInterface;  // Store for adaptive swing height
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
        SwingFootState empty;
        empty.fill(Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
        updateTerrain(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, liftOffPositionSequence,
                      touchDownPositionSequence, initTime, finalTime, empty, empty, empty);
    }

    void SwingTrajectoryPlanner::updateTerrain(
        const ModeSchedule &modeSchedule,
        const feet_array_t<scalar_array_t> &liftOffHeightSequence,
        const feet_array_t<scalar_array_t> &touchDownHeightSequence,
        const feet_array_t<std::vector<Eigen::Vector3d>> &liftOffPositionSequence,
        const feet_array_t<std::vector<Eigen::Vector3d>> &touchDownPositionSequence,
        scalar_t initTime, scalar_t finalTime,
        const SwingFootState &footPosition, const SwingFootState &footVelocity,
        const SwingFootState &footAcceleration) {
        scalar_array_t heightSequence(modeSchedule.modeSequence.size());
        feet_array_t<scalar_array_t> maxHeightSequence;
        for (size_t j = 0; j < numFeet_; j++) {
            for (int p = 0; p < modeSchedule.modeSequence.size(); ++p) {
                heightSequence[p] = std::max(liftOffHeightSequence[j][p], touchDownHeightSequence[j][p]);
            }
            maxHeightSequence[j] = heightSequence;
        }
        updateTerrain(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence,
                      liftOffPositionSequence, touchDownPositionSequence, initTime, finalTime, footPosition,
                      footVelocity, footAcceleration);
    }

    void SwingTrajectoryPlanner::updateTerrain(
        const ModeSchedule &modeSchedule,
        const feet_array_t<scalar_array_t> &liftOffHeightSequence,
        const feet_array_t<scalar_array_t> &touchDownHeightSequence,
        const feet_array_t<scalar_array_t> &maxHeightSequence,
        const feet_array_t<std::vector<Eigen::Vector3d>> &liftOffPositionSequence,
        const feet_array_t<std::vector<Eigen::Vector3d>> &touchDownPositionSequence,
        scalar_t initTime, scalar_t finalTime) {
        SwingFootState empty;
        empty.fill(Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN()));
        updateTerrain(modeSchedule, liftOffHeightSequence, touchDownHeightSequence, maxHeightSequence,
                      liftOffPositionSequence, touchDownPositionSequence, initTime, finalTime, empty, empty,
                      empty);
    }

    void SwingTrajectoryPlanner::updateTerrain(
        const ModeSchedule &modeSchedule,
        const feet_array_t<scalar_array_t> &liftOffHeightSequence,
        const feet_array_t<scalar_array_t> &touchDownHeightSequence,
        const feet_array_t<scalar_array_t> &maxHeightSequence,
        const feet_array_t<std::vector<Eigen::Vector3d>> &liftOffPositionSequence,
        const feet_array_t<std::vector<Eigen::Vector3d>> &touchDownPositionSequence,
        scalar_t initTime, scalar_t finalTime,
        const SwingFootState &footPosition, const SwingFootState &footVelocity,
        const SwingFootState &footAcceleration) {
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
            const auto previousCorridors = std::move(terrainCorridors_[j]);
            const auto previousStarts = std::move(terrainCorridorStarts_[j]);
            const auto previousEnds = std::move(terrainCorridorEnds_[j]);
            terrainCorridors_[j].clear();
            terrainCorridorStarts_[j].clear();
            terrainCorridorEnds_[j].clear();
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
                        for (size_t i = 0; i < previousCorridors.size(); ++i) {
                            if (std::abs(previousStarts[i] - swingStartTime) < 1e-8 &&
                                std::abs(previousEnds[i] - swingFinalTime) < 1e-8) {
                                terrainCorridors_[j].push_back(previousCorridors[i]);
                                terrainCorridorStarts_[j].push_back(swingStartTime);
                                terrainCorridorEnds_[j].push_back(swingFinalTime);
                                break;
                            }
                        }
                        // Section 4.4: the executing swing may be replaced by a
                        // state-corrected candidate when the admission check passes.
                        feetHeightTrajectories_[j].push_back(
                            replanExecutingSwing(j, *frozen, swingStartTime, swingFinalTime, initTime,
                                                 footPosition, footVelocity, footAcceleration));
                        continue;
                    }
                    const scalar_t scaling = swingTrajectoryScaling(swingStartTime, swingFinalTime,
                                                                    config_.swingTimeScale);
                    // Adaptive swing height. The reference must already clear the
                    // obstacle by more than the executor can lose: the QP's terrain
                    // row over a bar asks for crest + (groundClearance +
                    // trackingMargin) at the foot centre, and measured tracking
                    // error eats into whatever is left. On an 8 cm bar the old
                    // crest + 3 cm rule left ~1 mm of true margin (crest reads
                    // 0.0986 on the smoothed map, so 0.1286 was required against a
                    // 0.130 reference peak), which any WBC lag turns into a
                    // collision. Target the same bound the QP uses, plus an
                    // execution allowance, so the profile the QP is asked to
                    // certify is not already on the constraint surface.
                    scalar_t adaptiveSwingHeight = config_.swingHeight;
                    if (terrainInterface_ && swingFinalTime >= initTime && swingStartTime <= finalTime) {
                        const Eigen::Vector3d& liftOffPos = liftOffPositionSequence[j].at(p);
                        const Eigen::Vector3d& touchDownPos = touchDownPositionSequence[j].at(p);
                        // Sample the bar itself, not just the endpoints: a 3 cm
                        // thick obstacle under a 20 cm stride is invisible to the
                        // 0.2/0.4/0.6/0.8 midpoint probes on some phases.
                        double maxTerrainHeight = -std::numeric_limits<double>::infinity();
                        for (int sample = 0; sample <= 10; ++sample) {
                            const double s = 0.1 + 0.08 * sample;  // keep clear of contacts
                            if (s > 0.9) { break; }
                            const Eigen::Vector3d samplePos = (1 - s) * liftOffPos + s * touchDownPos;
                            try {
                                const double h = terrainInterface_->getHeight(samplePos.x(), samplePos.y());
                                maxTerrainHeight = std::max(maxTerrainHeight, h);
                            } catch (...) {
                                // Terrain query failed, skip this point
                            }
                        }
                        const double baseHeight = maxHeightSequence[j][p];
                        // Foot-centre height that clears the obstacle's crest:
                        // crest + foot sphere radius is the geometric contact
                        // point, then the QP corridor (clearance + tracking
                        // margin) and an execution allowance the WBC needs to
                        // absorb its own tracking lag.
                        const double executionAllowance = 0.02;
                        const double targetPeak = maxTerrainHeight +
                            (config_.terrainQpConfig.groundClearance + config_.terrainQpConfig.trackingMargin) +
                            executionAllowance;
                        if (targetPeak > baseHeight + adaptiveSwingHeight) {
                            adaptiveSwingHeight = targetPeak - baseHeight;
                            // Cap the lift so a corrupted map cell cannot ask for a
                            // stride the leg cannot track (~0.16 m above the
                            // support surface is a comfortable Cyberdog swing).
                            adaptiveSwingHeight = std::min(adaptiveSwingHeight, 0.16);
                        }
                    }
                    const scalar_t midHeight = maxHeightSequence[j][p] + scaling * adaptiveSwingHeight;
                    BSpline referenceSpline = createSwingHeightBSpline(
                        swingStartTime, swingFinalTime,
                        liftOffHeightSequence[j][p], scaling * config_.liftOffVelocity,
                        midHeight,
                        touchDownHeightSequence[j][p], scaling * config_.touchDownVelocity);

                    // Terrain mode tracks XYZ, including when the QP is unavailable.
                    // Insert the midpoint knot into the quintic smoothstep's Bezier
                    // controls [0,0,0,1,1,1]: [0,0,0,1/2,1,1,1]. Preserve the
                    // nominal Z controls and its endpoint derivatives exactly.
                    const Eigen::Vector3d& liftOffPosition = liftOffPositionSequence[j].at(p);
                    const Eigen::Vector3d& touchDownPosition = touchDownPositionSequence[j].at(p);
                    const Eigen::VectorXd progress =
                        (Eigen::VectorXd(kSwingHeightControlPoints) << 0, 0, 0, 0.5, 1, 1, 1).finished();
                    const Eigen::MatrixXd referenceControls = referenceSpline.getControlPoints();
                    referenceSpline.setControlPoints(
                        Eigen::VectorXd::Constant(kSwingHeightControlPoints, liftOffPosition.x()) +
                            progress * (touchDownPosition.x() - liftOffPosition.x()),
                        Eigen::VectorXd::Constant(kSwingHeightControlPoints, liftOffPosition.y()) +
                            progress * (touchDownPosition.y() - liftOffPosition.y()),
                        referenceControls.col(2));

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
                        }) : SwingTrajectoryQpOptimizer::HeightBounds{}, mapTiming_);
                    if (config_.terrainQpConfig.verbose) {
                        std::cerr << "Swing planning ms=" << std::chrono::duration<double, std::milli>(
                            std::chrono::steady_clock::now() - optimizationStart).count()
                                  << " corridor_cells=" << optimized.corridor.cells.size()
                                  << " corridor_certified=" << optimized.corridorCertified
                                  << " corridor_max_violation=" << optimized.corridorMaxViolation
                                  << " map_to_touchdown=" << optimized.corridor.mapToTouchdown
                                  << " map_height_margin=" << optimized.corridor.mapHeightMargin << '\n';
                    }
                    // Keep the corridor with the swing window it was built for. The QP may run
                    // for several future swings in one call; the execution-layer
                    // filter needs the one the foot is actually inside.
                    terrainCorridors_[j].push_back(optimized.corridor);
                    if (config_.safetyFilterConfig.enabled && optimized.valid() && optimized.corridorCertified) {
                        buildLandingCorridor(terrainCorridors_[j].back(), *terrainInterface_, *optimized.bspline,
                            config_.terrainQpConfig.corridorConfig, swingStartTime, swingFinalTime,
                            config_.safetyFilterConfig.landingPositionWindow);
                    }
                    if (!optimized.corridorCertified) terrainCorridors_[j].back().valid = false;
                    terrainCorridorStarts_[j].push_back(swingStartTime);
                    terrainCorridorEnds_[j].push_back(swingFinalTime);
                    if (!optimized.valid()) {
                        // guardrail: QP infeasible or output certification failed.
                        // Fall back to the reference spline rather than aborting the
                        // controller. The nominal XYZ reference remains available;
                        // it is a fallback, not a collision-certified trajectory.
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
        auto snapshot = std::make_shared<ExecutionSwings>();
        for (size_t leg = 0; leg < numFeet_; ++leg) {
            for (size_t i = 0; i < terrainCorridors_[leg].size(); ++i) {
                const double end = terrainCorridorEnds_[leg][i];
                for (const auto& spline : feetHeightTrajectories_[leg]) {
                    const auto& knots = spline.getKnots();
                    if (knots.size() && std::abs(knots[knots.size()-1] - end) < 1e-8) {
                        (*snapshot)[leg].push_back({spline, terrainCorridors_[leg][i],
                            terrainCorridorStarts_[leg][i], end});
                        break;
                    }
                }
            }
        }
        std::atomic_store(&executionSwings_, std::shared_ptr<const ExecutionSwings>(std::move(snapshot)));
    }


    // Section 4.4 / Algorithm 1 steps 2-4 for one executing swing. Returns the
    // reference to execute: the state-corrected candidate when every admission
    // condition holds, otherwise the unchanged previous reference.
    BSpline SwingTrajectoryPlanner::replanExecutingSwing(
        size_t leg, const BSpline &executing, scalar_t swingStartTime, scalar_t swingFinalTime,
        scalar_t initTime, const SwingFootState &footPosition, const SwingFootState &footVelocity,
        const SwingFootState &footAcceleration) {
        const auto& cfg = replanner_.config();
        const auto replanStart = std::chrono::steady_clock::now();
        if (!cfg.enabled || !config_.terrainQpConfig.safetyCorridorEnabled ||
            swingFinalTime - initTime < cfg.minRemainingTime || initTime < executing.getKnots()[0]) return executing;
        const double phase = (initTime - swingStartTime) / (swingFinalTime - swingStartTime);
        const double contactPhase = config_.terrainQpConfig.corridorConfig.contactPhase;
        if (phase < contactPhase || phase >= 1.0 - contactPhase) return executing;
        if (std::abs(pendingEnd_[leg] - swingFinalTime) > 1e-8) pendingActivation_[leg] = INFINITY;
        const bool activating = std::isfinite(pendingActivation_[leg]) && initTime >= pendingActivation_[leg];
        if (!activating && (std::isfinite(pendingActivation_[leg]) ||
                           initTime - lastReplanTime_[leg] < cfg.updateInterval)) return executing;
        lastReplanTime_[leg] = initTime;
        if (!footPosition[leg].allFinite() || !footVelocity[leg].allFinite()) {
            if (activating) {
                replanner_.recordRejection(SwingReplanOutcome::RejectedActivation);
                pendingActivation_[leg] = INFINITY;
            }
            return executing;
        }
        const Eigen::Vector3d acceleration = footAcceleration[leg].allFinite() ? footAcceleration[leg] :
                                            executing.getAcceleration(initTime);
        // Always re-read the synchronized terrain snapshot. An executing swing's
        // previous map certificate must not silently become a new-map certificate.
        const double referenceStart = executing.getKnots()[0];
        BSpline normalized = executing;
        normalized.setKnots(((executing.getKnots().array() - referenceStart) /
                             (swingFinalTime - referenceStart)).matrix());
        auto corridorConfig = config_.terrainQpConfig.corridorConfig;
        corridorConfig.contactPhase = 0.0;
        auto corridor = buildTerrainSafetyCorridor(*terrainInterface_, normalized,
            corridorConfig, config_.terrainQpConfig.subdivisionsPerSpan, HullBasis::Minvo, mapTiming_, swingFinalTime);
        for (auto& cell : corridor.cells) {
            cell.start = (referenceStart - swingStartTime + cell.start * (swingFinalTime - referenceStart)) /
                         (swingFinalTime - swingStartTime);
            cell.end = (referenceStart - swingStartTime + cell.end * (swingFinalTime - referenceStart)) /
                       (swingFinalTime - swingStartTime);
        }
        std::vector<CorridorPolytope> noncontact;
        for (auto& cell : corridor.cells) {
            if (cell.end <= contactPhase || cell.start >= 1.0-contactPhase) continue;
            cell.start = std::max(cell.start, contactPhase);
            cell.end = std::min(cell.end, 1.0-contactPhase);
            noncontact.push_back(std::move(cell));
        }
        corridor.cells = std::move(noncontact);
        corridor.start = contactPhase; corridor.end = 1.0-contactPhase;
        corridor.valid = corridor.valid && !corridor.cells.empty();
        // Current map must still support the ORIGINAL touchdown target. Do not
        // silently redirect it to the latest foothold selector's target.
        if (corridor.valid) {
            try {
                const auto terminal = executing.getPosition(swingFinalTime);
                if (std::abs(terrainInterface_->getHeight(terminal.x(), terminal.y())-terminal.z()) > 0.025) {
                    replanner_.recordRejection(SwingReplanOutcome::RejectedTerminal);
                    pendingActivation_[leg] = INFINITY;
                    return executing;
                }
            } catch (const TerrainUnavailable&) {
                replanner_.recordRejection(SwingReplanOutcome::RejectedTerminal);
                pendingActivation_[leg] = INFINITY;
                return executing;
            }
        }
        const double delay = activating ? 0.0 : cfg.activationDelay;
        const double activation = initTime + delay;
        auto predictionConfig = cfg;
        // Eq. (34), including input/model mismatch in disturbanceBound.
        predictionConfig.positionUncertainty += delay * cfg.velocityUncertainty +
                                                0.5 * cfg.disturbanceBound * delay * delay;
        predictionConfig.velocityUncertainty += cfg.disturbanceBound * delay;
        SwingReplanDecision decision;
        if (delay > 0.0) {
            SwingOnlineReplanner prediction(predictionConfig);
            decision = prediction.admit(executing, corridor, swingStartTime, activation, swingFinalTime,
                footPosition[leg] + delay * footVelocity[leg] + 0.5 * delay * delay * acceleration,
                footVelocity[leg] + delay * acceleration, acceleration);
            if (decision.admitted()) {
                pendingActivation_[leg] = activation;
                pendingEnd_[leg] = swingFinalTime;
                return executing;
            }
            pendingActivation_[leg] = INFINITY;
            replanner_.recordRejection(decision.outcome);
        } else {
            pendingActivation_[leg] = INFINITY;
            decision = replanner_.admit(executing, corridor, swingStartTime, initTime, swingFinalTime,
                                       footPosition[leg], footVelocity[leg], acceleration);
        }
        if (decision.admitted() && config_.safetyFilterConfig.enabled) {
            const auto& filterConfig = config_.safetyFilterConfig;
            SwingSafetyFilter::FootState state;
            state.position = footPosition[leg] + config_.footRadius * Eigen::Vector3d::UnitZ();
            state.velocity = footVelocity[leg];
            state.referenceAcceleration = decision.candidate.getAcceleration(initTime);
            // x_d matches the measured activation state by construction. Check
            // initial HOCBF and combined rows using the configured input box.
            const auto control = SwingSafetyFilter(filterConfig).filter(corridor, state,
                state.referenceAcceleration, phase, filterConfig.maxPeriod);
            if (!control.active() || (filterConfig.tubeEnabled && control.tubeStatus != SafetyFilterStatus::Solved)) {
                decision.outcome = SwingReplanOutcome::RejectedControl;
                decision.reason = control.status == SafetyFilterStatus::InvalidState ?
                    "HOCBF or tube initial state invalid" : "initial joint filter rows infeasible";
                replanner_.revokeAdmission(decision.outcome);
            }
        }
        if (decision.admitted() && workspace_) {
            // The existing workspace API is sampled IK; do not label it a hull certificate.
            for (int i = 0; i <= 20; ++i) {
                const double t = initTime + (swingFinalTime - initTime) * i / 20.0;
                const auto point = decision.candidate.getPosition(t);
                const auto bounds = workspace_(leg, t, point);
                if (!bounds.allFinite() || point.z() < bounds[0] - 1e-8 || point.z() > bounds[1] + 1e-8) {
                    decision.outcome = SwingReplanOutcome::RejectedCandidate;
                    decision.reason = "candidate outside IK workspace";
                    replanner_.revokeAdmission(decision.outcome);
                    break;
                }
            }
        }
        if (config_.terrainQpConfig.verbose) {
            std::cerr << "Swing replan leg=" << leg << " activation=" << initTime
                      << " remaining=" << swingFinalTime - initTime
                      << " position_delta=" << (footPosition[leg] - executing.getPosition(initTime)).norm()
                      << " outcome=" << static_cast<int>(decision.outcome) << " reason=" << decision.reason
                      << " replan_ms=" << std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - replanStart).count()
                      << " stats=" << replanner_.stats().json() << '\n';
        }
        if (!decision.admitted()) return executing;
        // Store only a certificate whose maps belong to the accepted knot space.
        if (!terrainCorridorEnds_[leg].empty() && terrainCorridorEnds_[leg].back() == swingFinalTime) {
            auto& committed = terrainCorridors_[leg].back();
            committed = SwingOnlineReplanner::remapCorridor(
                decision.candidate, corridor, swingStartTime, swingFinalTime);
            if (config_.safetyFilterConfig.enabled) {
                buildLandingCorridor(committed, *terrainInterface_, decision.candidate,
                    config_.terrainQpConfig.corridorConfig, swingStartTime, swingFinalTime,
                    config_.safetyFilterConfig.landingPositionWindow);
            }
        }
        return decision.candidate;
    }

    feet_array_t<TerrainSafetyCorridor> SwingTrajectoryPlanner::getTerrainCorridors() const {
        feet_array_t<TerrainSafetyCorridor> result;
        for (size_t leg = 0; leg < numFeet_; ++leg) {
            // The last swing of the horizon is the freshest plan; publish that one.
            if (!terrainCorridors_[leg].empty()) {
                result[leg] = terrainCorridors_[leg].back();
            }
        }
        return result;
    }

    const TerrainSafetyCorridor& SwingTrajectoryPlanner::getTerrainCorridor(size_t leg, scalar_t time) const {
        static const TerrainSafetyCorridor none;
        const auto& corridors = terrainCorridors_[leg];
        for (size_t i = 0; i < corridors.size(); ++i) {
            if (time >= terrainCorridorStarts_[leg][i] && time < terrainCorridorEnds_[leg][i] &&
                corridors[i].valid) {
                return corridors[i];
            }
        }
        return none;
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
        loadOptional(config.trackingVerticalErrorGain, "trackingVerticalErrorGain");
        loadOptional(config.trackingVerticalHardConstraint, "trackingVerticalHardConstraint");
        loadOptional(config.terrainQpConfig.numControlPoints, "terrainNumControlPoints");
        loadOptional(config.terrainQpConfig.numCurveSamples, "terrainNumCurveSamples");
        loadOptional(config.terrainQpConfig.controlPointTrackingWeight, "terrainControlPointTrackingWeight");
        loadOptional(config.terrainQpConfig.positionTrackingWeight, "terrainPositionTrackingWeight");
        loadOptional(config.terrainQpConfig.velocityTrackingWeight, "terrainVelocityTrackingWeight");
        loadOptional(config.terrainQpConfig.accelerationTrackingWeight, "terrainAccelerationTrackingWeight");
        loadOptional(config.terrainQpConfig.jerkSmoothnessWeight, "terrainJerkSmoothnessWeight");
        loadOptional(config.terrainQpConfig.groundClearance, "terrainGroundClearance");
        loadOptional(config.terrainQpConfig.trackingMargin, "terrainTrackingMargin");
        loadOptional(config.terrainQpConfig.calfLength, "terrainCalfLength");
        loadOptional(config.terrainQpConfig.calfSlope, "terrainCalfSlope");
        loadOptional(config.terrainQpConfig.calfSamples, "terrainCalfSamples");
        loadOptional(config.terrainQpConfig.maxCorrection, "terrainMaxCorrection");
        loadOptional(config.terrainQpConfig.maxVelocity, "terrainMaxVelocity");
        loadOptional(config.terrainQpConfig.maxAcceleration, "terrainMaxAcceleration");
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

        // Section 4.4 online replanning: off by default, it changes the executing
        // reference mid-swing rather than only future swings.
        loadOptional(config.replanConfig.enabled, "swingOnlineReplanEnabled");
        loadOptional(config.replanConfig.correctionDuration, "swingReplanCorrectionDuration");
        loadOptional(config.replanConfig.activationDelay, "swingReplanActivationDelay");
        loadOptional(config.replanConfig.maxPositionDeviation, "swingReplanMaxPositionDeviation");
        loadOptional(config.replanConfig.maxVelocityDeviation, "swingReplanMaxVelocityDeviation");
        loadOptional(config.replanConfig.maxMarginFraction, "swingReplanMaxMarginFraction");
        loadOptional(config.replanConfig.minRemainingTime, "swingReplanMinRemainingTime");
        loadOptional(config.replanConfig.updateInterval, "swingReplanUpdateInterval");
        loadOptional(config.replanConfig.maxVelocity, "swingReplanMaxVelocity");
        loadOptional(config.replanConfig.maxAcceleration, "swingReplanMaxAcceleration");
        loadOptional(config.replanConfig.maxJerk, "swingReplanMaxJerk");
        loadOptional(config.replanConfig.positionUncertainty, "swingReplanPositionUncertainty");
        loadOptional(config.replanConfig.velocityUncertainty, "swingReplanVelocityUncertainty");
        loadOptional(config.replanConfig.disturbanceBound, "swingReplanDisturbanceBound");
        loadOptional(config.replanConfig.nominalTrackingWeight, "swingReplanNominalTrackingWeight");
        loadOptional(config.replanConfig.previousTrackingWeight, "swingReplanPreviousTrackingWeight");
        auto& filter = config.safetyFilterConfig;
        loadOptional(filter.enabled, "safetyFilterEnabled");
        loadOptional(filter.tubeEnabled, "safetyFilterTubeEnabled");
        loadOptional(filter.hocbfGain1, "safetyFilterHocbfGain1");
        loadOptional(filter.hocbfGain2, "safetyFilterHocbfGain2");
        loadOptional(filter.disturbanceBound, "safetyFilterDisturbanceBound");
        loadOptional(filter.positionBound, "safetyFilterPositionBound");
        loadOptional(filter.velocityBound, "safetyFilterVelocityBound");
        loadOptional(filter.couplingGain, "safetyFilterCouplingGain");
        loadOptional(filter.couplingBound, "safetyFilterCouplingBound");
        loadOptional(filter.tubeRate, "safetyFilterTubeRate");
        loadOptional(filter.maxCorrection, "safetyFilterMaxCorrection");
        loadOptional(filter.maxAcceleration, "safetyFilterMaxAcceleration");
        loadOptional(filter.hocbfSampleRate, "safetyFilterHocbfSampleRate");
        loadOptional(filter.tubeSampleRate, "safetyFilterTubeSampleRate");
        loadOptional(filter.maxPeriod, "safetyFilterMaxPeriod");
        loadOptional(filter.landingPositionWindow, "safetyFilterLandingPositionWindow");
        loadOptional(filter.landingMaxDownSpeed, "safetyFilterLandingMaxDownSpeed");
        loadOptional(filter.landingMaxUpSpeed, "safetyFilterLandingMaxUpSpeed");
        config.replanConfig.landingPositionWindow = config.terrainQpConfig.landingPositionWindow =
            filter.landingPositionWindow;
        config.replanConfig.landingMaxDownSpeed = config.terrainQpConfig.landingMaxDownSpeed =
            filter.landingMaxDownSpeed;
        config.replanConfig.landingMaxUpSpeed = config.terrainQpConfig.landingMaxUpSpeed =
            filter.landingMaxUpSpeed;

        // Section 3.1 terrain safety corridor: optional, off by default so the
        // existing height-based planning path is untouched unless switched on.
        loadOptional(config.terrainQpConfig.safetyCorridorEnabled, "terrainSafetyCorridorEnabled");
        auto& corridor = config.terrainQpConfig.corridorConfig;
        loadOptional(corridor.lateralExtent, "corridorLateralExtent");
        loadOptional(corridor.longitudinalExtent, "corridorLongitudinalExtent");
        loadOptional(corridor.verticalExtent, "corridorVerticalExtent");
        loadOptional(corridor.mapMargin, "corridorMapMargin");
        loadOptional(corridor.terrainRiseRate, "corridorTerrainRiseRate");
        loadOptional(corridor.geometricClearance, "corridorGeometricClearance");
        loadOptional(corridor.numericalMargin, "corridorNumericalMargin");
        loadOptional(corridor.contactPhase, "corridorContactPhase");
        loadOptional(corridor.footRadius, "corridorFootRadius");
        loadOptional(corridor.trackingMargin, "corridorTrackingMargin");
        // Default the corridor's geometry to the values the QP already uses, so
        // enabling it does not silently change the safety margins.
        if (config.terrainQpConfig.safetyCorridorEnabled) {
            if (!pt.get_optional<scalar_t>(prefix + "corridorFootRadius")) {
                corridor.footRadius = config.footRadius;
            }
            if (!pt.get_optional<scalar_t>(prefix + "corridorTrackingMargin")) {
                corridor.trackingMargin = config.terrainQpConfig.trackingMargin;
            }
        }

        if (verbose) {
            std::cerr << " #### =============================================================================" <<
                    std::endl;
        }

        return config;
    }
} // namespace ocs2::legged_robot
