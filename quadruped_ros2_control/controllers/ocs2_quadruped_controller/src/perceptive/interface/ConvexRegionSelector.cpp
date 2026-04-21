//
// Created by biao on 3/21/25.
//

#include <ocs2_quadruped_controller/perceptive/interface/ConvexRegionSelector.h>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>

#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition/GeometryUtils.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>

namespace ocs2::legged_robot
{
    namespace
    {
        scalar_t wrapAngle(scalar_t angle)
        {
            return std::atan2(std::sin(angle), std::cos(angle));
        }

        scalar_t estimateDesiredYawRate(const TargetTrajectories& targetTrajectories,
                                        const CentroidalModelInfo& info,
                                        scalar_t time)
        {
            if (targetTrajectories.size() < 2)
            {
                return 0.0;
            }

            constexpr scalar_t yawRateSamplingWindow = 0.05;
            const auto stateBefore = targetTrajectories.getDesiredState(time - yawRateSamplingWindow);
            const auto stateAfter = targetTrajectories.getDesiredState(time + yawRateSamplingWindow);

            const scalar_t yawBefore = centroidal_model::getBasePose(stateBefore, info)(3);
            const scalar_t yawAfter = centroidal_model::getBasePose(stateAfter, info)(3);
            const scalar_t deltaTime = 2.0 * yawRateSamplingWindow;
            return wrapAngle(yawAfter - yawBefore) / deltaTime;
        }

        scalar_t computePrecedingSwingDuration(size_t stancePhaseIndex,
                                               const std::vector<bool>& contactFlags,
                                               const scalar_array_t& eventTimes)
        {
            if (stancePhaseIndex == 0 || stancePhaseIndex >= contactFlags.size())
            {
                return 0.0;
            }

            if (!contactFlags[stancePhaseIndex] || contactFlags[stancePhaseIndex - 1])
            {
                return 0.0;
            }

            int firstSwingPhaseIndex = static_cast<int>(stancePhaseIndex) - 1;
            while (firstSwingPhaseIndex > 0 && !contactFlags[firstSwingPhaseIndex - 1])
            {
                --firstSwingPhaseIndex;
            }

            if (firstSwingPhaseIndex == 0)
            {
                return 0.0;
            }

            const int liftOffEventIndex = firstSwingPhaseIndex - 1;
            const int touchDownEventIndex = static_cast<int>(stancePhaseIndex) - 1;
            if (liftOffEventIndex < 0 || touchDownEventIndex < 0 ||
                touchDownEventIndex >= static_cast<int>(eventTimes.size()))
            {
                return 0.0;
            }

            return std::max(eventTimes[touchDownEventIndex] - eventTimes[liftOffEventIndex], scalar_t(0.0));
        }
    } // namespace

    ConvexRegionSelector::ConvexRegionSelector(CentroidalModelInfo info,
                                               std::shared_ptr<convex_plane_decomposition::PlanarTerrain>
                                               planarTerrainPtr,
                                               const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                               size_t numVertices)
        : info_(std::move(info)),
          numVertices_(numVertices),
          planarTerrainPtr_(std::move(planarTerrainPtr)),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone())
    {
    }
    // 指定腿在指定时间的足部投影
    convex_plane_decomposition::PlanarTerrainProjection ConvexRegionSelector::getProjection(
        size_t leg, scalar_t time) const
    {
        const auto index = lookup::findIndexInTimeArray(timeEvents_[leg], time);
        return feetProjections_[leg][index];
    }
    // 返回凸多边形,确保安全的足端放置区域
    convex_plane_decomposition::CgalPolygon2d ConvexRegionSelector::getConvexPolygon(size_t leg, scalar_t time) const
    {
        const auto index = lookup::findIndexInTimeArray(timeEvents_[leg], time);
        return convexPolygons_[leg][index];
    }
    // 返回名义足端位置,理想位置
    vector3_t ConvexRegionSelector::getNominalFootholds(size_t leg, scalar_t time) const
    {
        const auto index = lookup::findIndexInTimeArray(timeEvents_[leg], time);
        return nominalFootholds_[leg][index];
    }

    void ConvexRegionSelector::update(const ModeSchedule& modeSchedule, scalar_t initTime, const vector_t& initState,
                                      TargetTrajectories& targetTrajectories)
    {
        planarTerrain_ = *planarTerrainPtr_;
        // Need copy storage it since PlanarTerrainProjection.regionPtr is a pointer
        const auto& modeSequence = modeSchedule.modeSequence;
        const auto& eventTimes = modeSchedule.eventTimes;
        const auto contactFlagStocks = extractContactFlags(modeSequence);
        const size_t numPhases = modeSequence.size();

        // Find start and final index of time for legs
        feet_array_t<std::vector<int>> startIndices;
        feet_array_t<std::vector<int>> finalIndices;
        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++)
        {
            startIndices[leg] = std::vector<int>(numPhases, 0);
            finalIndices[leg] = std::vector<int>(numPhases, 0);
            // find the startTime and finalTime indices for swing feet  确定每个接触阶段的起止索引
            for (size_t i = 0; i < numPhases; i++)
            {
                // skip if it is a stance leg
                if (contactFlagStocks[leg][i])
                {
                    std::tie(startIndices[leg][i], finalIndices[leg][i]) = findIndex(i, contactFlagStocks[leg]);
                }
            }
        }

        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++)
        {
            feetProjections_[leg].clear();
            convexPolygons_[leg].clear();
            nominalFootholds_[leg].clear();
            feetProjections_[leg].resize(numPhases);
            convexPolygons_[leg].resize(numPhases);
            nominalFootholds_[leg].resize(numPhases);
            middleTimes_[leg].clear();
            initStandFinalTime_[leg] = 0;

            scalar_t lastStandMiddleTime = NAN;
            // Stand leg foot
            for (size_t i = 0; i < numPhases; ++i)
            {
                if (contactFlagStocks[leg][i])
                {
                    const int standStartIndex = startIndices[leg][i];
                    const int standFinalIndex = finalIndices[leg][i];
                    const scalar_t standStartTime = eventTimes[standStartIndex];
                    const scalar_t standFinalTime = eventTimes[standFinalIndex];
                    const scalar_t standMiddleTime = standStartTime + (standFinalTime - standStartTime) / 2;
                    
                    // 如果该中间时间与前一个接触块不同（避免重复处理连续的接触阶段），则生成新的落足点信息
                    if (!numerics::almost_eq(standMiddleTime, lastStandMiddleTime))
                    {
                        scalar_t footholdReferenceTime = standMiddleTime;
                        if (i > 0 && !contactFlagStocks[leg][i - 1] && i - 1 < eventTimes.size())
                        {
                            footholdReferenceTime = std::max(eventTimes[i - 1], initTime);
                        }

                        const vector_t desiredState = targetTrajectories.getDesiredState(footholdReferenceTime);
                        const vector3_t desiredFootPosition = endEffectorKinematicsPtr_->getPosition(desiredState)[leg];
                        const scalar_t swingDuration = computePrecedingSwingDuration(i, contactFlagStocks[leg], eventTimes);
                        const scalar_t desiredYawRate = estimateDesiredYawRate(targetTrajectories, info_, footholdReferenceTime);
                        const vector3_t nominalFoothold =
                            getNominalFoothold(leg, desiredState, initState, desiredFootPosition, swingDuration,
                                               desiredYawRate);

                        // 边缘距离惩罚函数，鼓励落足点远离边界，提升安全性
                        auto penaltyFunction = [this](const vector3_t& projectedPointInWorld) -> double {
                            // if (footholdScorer_ != nullptr) {
                            //     return footholdScorer_->computePenalty(projectedPointInWorld);
                            // }
                            return 0.0;
                        };
                        auto safeProjection = getBestPlanarRegionAtPositionInWorld(
                            nominalFoothold, planarTerrain_.planarRegions, penaltyFunction);

                        // 凸多边形生长 (numVertices=8, growthFactor=1.1 加速收敛)
                        constexpr double growthFactor = 1.05;
                        auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
                            safeProjection.regionPtr->boundaryWithInset.boundary,
                            safeProjection.positionInTerrainFrame,
                            static_cast<int>(numVertices_), growthFactor);

                        feetProjections_[leg][i] = safeProjection;
                        convexPolygons_[leg][i] = convexRegion;
                        nominalFootholds_[leg][i] = nominalFoothold;
                        middleTimes_[leg].push_back(standMiddleTime);
                        lastStandMiddleTime = standMiddleTime;
                    }
                    else
                    {
                        feetProjections_[leg][i] = feetProjections_[leg][i - 1];
                        convexPolygons_[leg][i] = convexPolygons_[leg][i - 1];
                        nominalFootholds_[leg][i] = nominalFootholds_[leg][i - 1];
                    }

                    if (standStartTime < initTime && initTime < standFinalTime)
                    {
                        initStandFinalTime_[leg] = standFinalTime;
                    }
                }
            }
        }

        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++)
        {
            timeEvents_[leg] = eventTimes;
        }
    }

    feet_array_t<std::vector<bool>> ConvexRegionSelector::extractContactFlags(
        const std::vector<size_t>& phaseIDsStock) const
    {
        const size_t numPhases = phaseIDsStock.size();

        feet_array_t<std::vector<bool>> contactFlagStock;
        std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

        for (size_t i = 0; i < numPhases; i++)
        {
            const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
            for (size_t j = 0; j < info_.numThreeDofContacts; j++)
            {
                contactFlagStock[j][i] = contactFlag[j];
            }
        }
        return contactFlagStock;
    }

    std::pair<int, int> ConvexRegionSelector::findIndex(size_t index, const std::vector<bool>& contactFlagStock)
    {
        const size_t numPhases = contactFlagStock.size();

        if (!contactFlagStock[index])
        {
            return {0, 0};
        }

        // find the starting time
        int startTimesIndex = 0;
        for (int ip = index - 1; ip >= 0; ip--)
        {
            if (!contactFlagStock[ip])
            {
                startTimesIndex = ip;
                break;
            }
        }
        // find the final time
        int finalTimesIndex = numPhases - 2;
        for (size_t ip = index + 1; ip < numPhases; ip++)
        {
            if (!contactFlagStock[ip])
            {
                finalTimesIndex = ip - 1;
                break;
            }
        }
        return {startTimesIndex, finalTimesIndex};
    }

    vector3_t ConvexRegionSelector::getNominalFoothold(size_t leg, const vector_t& desiredState,
                                                       const vector_t& initState,
                                                       const vector3_t& desiredFootPosition,
                                                       scalar_t swingDuration,
                                                       scalar_t desiredYawRate) const
    {
        constexpr scalar_t comHeight = 0.26;  // nominal standing height (comHeight from reference.info)
        constexpr scalar_t gravity = 9.81;
        constexpr scalar_t maxPreviewTime = 0.35;
        constexpr scalar_t maxHorizontalCorrection = 0.12;

        vector3_t desiredVel = centroidal_model::getNormalizedMomentum(desiredState, info_).head(3);
        vector3_t measuredVel = centroidal_model::getNormalizedMomentum(initState, info_).head(3);

        const scalar_t previewTime = std::min(std::max(swingDuration, scalar_t(0.0)), maxPreviewTime);
        const vector3_t basePosition = centroidal_model::getBasePose(desiredState, info_).head(3);

        vector3_t feedforward = 0.5 * previewTime * desiredVel;
        feedforward.z() = 0.0;

        vector3_t feedback = std::sqrt(comHeight / gravity) * (measuredVel - desiredVel);
        feedback.z() = 0.0;

        vector3_t baseToFoot = desiredFootPosition - basePosition;
        baseToFoot.z() = 0.0;

        vector3_t turningCompensation;
        turningCompensation << -desiredYawRate * baseToFoot.y(), desiredYawRate * baseToFoot.x(), 0.0;
        turningCompensation *= 0.5 * previewTime;

        vector3_t horizontalCorrection = feedforward + feedback + turningCompensation;
        const scalar_t horizontalCorrectionNorm = horizontalCorrection.head(2).norm();
        if (horizontalCorrectionNorm > maxHorizontalCorrection)
        {
            horizontalCorrection.head(2) *= maxHorizontalCorrection / horizontalCorrectionNorm;
        }

        vector3_t nominalFoothold = desiredFootPosition + horizontalCorrection;
        nominalFoothold.z() = desiredFootPosition.z();
        return nominalFoothold;
    }
} // namespace legged