//
// Created by biao on 3/21/25.
//

#include <ocs2_quadruped_controller/perceptive/interface/ConvexRegionSelector.h>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_core/misc/Lookup.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_quadruped_controller/perceptive/common/FootPlacementSafetySet.h>

#include <convex_plane_decomposition/ConvexRegionGrowing.h>
#include <convex_plane_decomposition/GeometryUtils.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>

namespace ocs2::legged_robot
{
    namespace
    {
        constexpr scalar_t kFootholdCandidateDuplicateTolerance = 1e-4;
        constexpr scalar_t kJointCentroidWeight = 1.0;
        constexpr scalar_t kJointGeometryWeight = 0.35;
        constexpr scalar_t kJointAreaWeight = 6.0;
        constexpr scalar_t kJointAreaRetentionRatio = 0.65;
        constexpr scalar_t kJointSupportMarginWeight = 10.0;
        constexpr scalar_t kJointSupportSpanWeight = 3.5;
        constexpr scalar_t kSupportMarginTarget = 0.03;
        constexpr scalar_t kLongitudinalSpanRetentionRatio = 0.75;
        constexpr scalar_t kLateralSpanRetentionRatio = 0.75;
        constexpr size_t kJointOptimizationIterations = 3;
        constexpr scalar_t kSynchronousEventTimeTolerance = 0.3;
        constexpr scalar_t kSynchronousOverlapSlack = 0.08;
        constexpr scalar_t kLockHardRadius = 0.025;
        constexpr scalar_t kLockSoftRadius = 0.08;
        constexpr scalar_t kLockDeviationWeight = 16.0;
        constexpr scalar_t kLockedCandidatePreferenceMargin = 0.002;
        constexpr scalar_t kLockedCandidateEdgeThreshold = 0.03;
        constexpr scalar_t kLockedCandidateBoundaryThreshold = 0.01;

        struct JointSupportCosts
        {
            scalar_t supportCentroidCost = 0.0;
            scalar_t supportGeometryCost = 0.0;
            scalar_t supportAreaCost = 0.0;
            scalar_t supportMarginCost = 0.0;
            scalar_t supportSpanCost = 0.0;

            scalar_t total() const
            {
                return supportCentroidCost + supportGeometryCost + supportAreaCost + supportMarginCost +
                       supportSpanCost;
            }
        };

        struct SynchronousEventGroup
        {
            std::vector<size_t> memberIndices;
            scalar_t anchorTime = 0.0;
            scalar_t overlapStart = 0.0;
            scalar_t overlapEnd = 0.0;
        };

        struct JointFootholdEvent
        {
            size_t leg = 0;
            size_t phaseIndex = 0;
            scalar_t standMiddleTime = 0.0;
            scalar_t standStartTime = 0.0;
            scalar_t standFinalTime = 0.0;
            bool hasValidRegion = false;
            int supportGroup = -1;
            convex_plane_decomposition::PlanarTerrainProjection projection;
            convex_plane_decomposition::CgalPolygon2d safetyPolygon;
            vector3_t nominalFoothold = vector3_t::Zero();
            vector3_t projectedFoothold = vector3_t::Zero();
            vector3_t predictedBasePosition = vector3_t::Zero();
            bool hasLockedFoothold = false;
            vector3_t lockedFoothold = vector3_t::Zero();
            bool hasLockedCandidate = false;
            size_t lockedCandidateIndex = 0;
            std::vector<convex_plane_decomposition::CgalPoint2d> candidatesInTerrain;
            std::vector<ConvexRegionSelector::FootholdScoreBreakdown> candidateScores;
            size_t selectedIndex = 0;
        };

        scalar_t computeLocalCost(const ConvexRegionSelector::FootholdScoreBreakdown& breakdown)
        {
            return breakdown.nominalTrackingCost + breakdown.projectedTrackingCost +
                   breakdown.previewVelocityCost + breakdown.reachConsistencyCost +
                   breakdown.boundaryMarginCost + breakdown.continuityCost +
                   breakdown.edgePenaltyCost + breakdown.lockDeviationCost;
        }

        scalar_t computeCross2d(const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs)
        {
            return lhs.x() * rhs.y() - lhs.y() * rhs.x();
        }

        Eigen::Vector2d toVector2(const vector3_t& position)
        {
            return {position.x(), position.y()};
        }

        convex_plane_decomposition::CgalPoint2d positionInPlaneFrameFromPositionInWorld(
            const vector3_t& positionInWorld,
            const Eigen::Isometry3d& transformPlaneToWorld)
        {
            const Eigen::Vector3d positionInPlane = transformPlaneToWorld.inverse() * positionInWorld;
            return {positionInPlane.x(), positionInPlane.y()};
        }

        Eigen::Vector2d computeCentroid2d(const std::vector<Eigen::Vector2d>& points)
        {
            if (points.empty())
            {
                return Eigen::Vector2d::Zero();
            }

            Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
            for (const auto& point : points)
            {
                centroid += point;
            }
            return centroid / static_cast<scalar_t>(points.size());
        }

        scalar_t computeSortedPolygonArea(std::vector<Eigen::Vector2d> points)
        {
            if (points.size() < 3)
            {
                return 0.0;
            }

            const auto centroid = computeCentroid2d(points);
            std::sort(points.begin(), points.end(), [&centroid](const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs) {
                return std::atan2(lhs.y() - centroid.y(), lhs.x() - centroid.x()) <
                       std::atan2(rhs.y() - centroid.y(), rhs.x() - centroid.x());
            });

            scalar_t doubleArea = 0.0;
            for (size_t i = 0; i < points.size(); ++i)
            {
                const auto& current = points[i];
                const auto& next = points[(i + 1) % points.size()];
                doubleArea += current.x() * next.y() - next.x() * current.y();
            }
            return 0.5 * std::abs(doubleArea);
        }

        scalar_t computeSignedSupportMargin(const std::vector<Eigen::Vector2d>& points,
                                            const Eigen::Vector2d& query)
        {
            if (points.size() < 3)
            {
                return 0.0;
            }

            std::vector<Eigen::Vector2d> sortedPoints = points;
            const auto centroid = computeCentroid2d(sortedPoints);
            std::sort(sortedPoints.begin(), sortedPoints.end(), [&centroid](const Eigen::Vector2d& lhs, const Eigen::Vector2d& rhs) {
                return std::atan2(lhs.y() - centroid.y(), lhs.x() - centroid.x()) <
                       std::atan2(rhs.y() - centroid.y(), rhs.x() - centroid.x());
            });

            scalar_t signedArea = 0.0;
            for (size_t i = 0; i < sortedPoints.size(); ++i)
            {
                const auto& current = sortedPoints[i];
                const auto& next = sortedPoints[(i + 1) % sortedPoints.size()];
                signedArea += computeCross2d(current, next);
            }
            const scalar_t orientation = signedArea >= 0.0 ? 1.0 : -1.0;

            scalar_t minMargin = std::numeric_limits<scalar_t>::infinity();
            for (size_t i = 0; i < sortedPoints.size(); ++i)
            {
                const auto& current = sortedPoints[i];
                const auto& next = sortedPoints[(i + 1) % sortedPoints.size()];
                const Eigen::Vector2d edge = next - current;
                const scalar_t edgeNorm = std::max(edge.norm(), scalar_t(1e-6));
                const scalar_t signedDistance = orientation * computeCross2d(edge, query - current) / edgeNorm;
                minMargin = std::min(minMargin, signedDistance);
            }
            return std::isfinite(minMargin) ? minMargin : 0.0;
        }

        scalar_t computeSpan(const std::vector<Eigen::Vector2d>& points, int axis)
        {
            if (points.empty())
            {
                return 0.0;
            }

            scalar_t minValue = std::numeric_limits<scalar_t>::infinity();
            scalar_t maxValue = -std::numeric_limits<scalar_t>::infinity();
            for (const auto& point : points)
            {
                minValue = std::min(minValue, point(axis));
                maxValue = std::max(maxValue, point(axis));
            }
            return maxValue - minValue;
        }

        bool groupContainsLeg(const SynchronousEventGroup& group,
                              const std::vector<JointFootholdEvent>& events,
                              size_t leg)
        {
            return std::any_of(group.memberIndices.begin(), group.memberIndices.end(),
                               [&events, leg](size_t index) { return events[index].leg == leg; });
        }

        std::vector<SynchronousEventGroup> buildSynchronousEventGroups(std::vector<JointFootholdEvent>& events)
        {
            std::vector<size_t> sortedIndices(events.size());
            std::iota(sortedIndices.begin(), sortedIndices.end(), 0);
            std::sort(sortedIndices.begin(), sortedIndices.end(), [&events](size_t lhs, size_t rhs) {
                return events[lhs].standMiddleTime < events[rhs].standMiddleTime;
            });

            std::vector<SynchronousEventGroup> groups;
            for (const auto eventIndex : sortedIndices)
            {
                auto& event = events[eventIndex];
                int assignedGroup = -1;
                scalar_t bestTimeResidual = std::numeric_limits<scalar_t>::infinity();
                for (size_t groupIndex = 0; groupIndex < groups.size(); ++groupIndex)
                {
                    const auto& group = groups[groupIndex];
                    if (groupContainsLeg(group, events, event.leg))
                    {
                        continue;
                    }

                    const scalar_t nextOverlapStart = std::max(group.overlapStart, event.standStartTime);
                    const scalar_t nextOverlapEnd = std::min(group.overlapEnd, event.standFinalTime);
                    const bool overlaps = nextOverlapStart <= nextOverlapEnd + kSynchronousOverlapSlack;
                    const scalar_t timeResidual = std::abs(group.anchorTime - event.standMiddleTime);
                    if (!overlaps || timeResidual > kSynchronousEventTimeTolerance)
                    {
                        continue;
                    }

                    if (timeResidual < bestTimeResidual)
                    {
                        bestTimeResidual = timeResidual;
                        assignedGroup = static_cast<int>(groupIndex);
                    }
                }

                if (assignedGroup < 0)
                {
                    SynchronousEventGroup group;
                    group.memberIndices.push_back(eventIndex);
                    group.anchorTime = event.standMiddleTime;
                    group.overlapStart = event.standStartTime;
                    group.overlapEnd = event.standFinalTime;
                    groups.push_back(group);
                    event.supportGroup = static_cast<int>(groups.size() - 1);
                    continue;
                }

                auto& selectedGroup = groups[assignedGroup];
                selectedGroup.memberIndices.push_back(eventIndex);
                selectedGroup.anchorTime =
                    (selectedGroup.anchorTime * static_cast<scalar_t>(selectedGroup.memberIndices.size() - 1) +
                     event.standMiddleTime) /
                    static_cast<scalar_t>(selectedGroup.memberIndices.size());
                selectedGroup.overlapStart = std::max(selectedGroup.overlapStart, event.standStartTime);
                selectedGroup.overlapEnd = std::min(selectedGroup.overlapEnd, event.standFinalTime);
                event.supportGroup = assignedGroup;
            }

            return groups;
        }

        JointSupportCosts computeJointCostsForCandidate(const std::vector<JointFootholdEvent>& events,
                                                        const std::vector<size_t>& group,
                                                        size_t eventIndexInGroup,
                                                        size_t candidateIndex)
        {
            JointSupportCosts costs;
            if (group.size() < 2)
            {
                return costs;
            }

            std::vector<Eigen::Vector2d> selectedPoints;
            std::vector<Eigen::Vector2d> nominalPoints;
            std::vector<Eigen::Vector2d> predictedBasePoints;
            selectedPoints.reserve(group.size());
            nominalPoints.reserve(group.size());
            predictedBasePoints.reserve(group.size());

            for (size_t groupIndex = 0; groupIndex < group.size(); ++groupIndex)
            {
                const auto& event = events[group[groupIndex]];
                nominalPoints.push_back(toVector2(event.nominalFoothold));
                predictedBasePoints.push_back(toVector2(event.predictedBasePosition));

                if (!event.hasValidRegion || event.candidateScores.empty())
                {
                    selectedPoints.push_back(toVector2(event.projection.positionInWorld));
                    continue;
                }

                const size_t resolvedCandidateIndex =
                    (groupIndex == eventIndexInGroup) ? candidateIndex : event.selectedIndex;
                selectedPoints.push_back(toVector2(event.candidateScores[resolvedCandidateIndex].candidateInWorld));
            }

            const auto selectedCentroid = computeCentroid2d(selectedPoints);
            const auto nominalCentroid = computeCentroid2d(nominalPoints);
            costs.supportCentroidCost = kJointCentroidWeight * (selectedCentroid - nominalCentroid).squaredNorm();

            scalar_t geometryResidual = 0.0;
            size_t pairCount = 0;
            for (size_t i = 0; i < selectedPoints.size(); ++i)
            {
                for (size_t j = i + 1; j < selectedPoints.size(); ++j)
                {
                    const auto selectedDelta = selectedPoints[i] - selectedPoints[j];
                    const auto nominalDelta = nominalPoints[i] - nominalPoints[j];
                    geometryResidual += (selectedDelta - nominalDelta).squaredNorm();
                    ++pairCount;
                }
            }
            costs.supportGeometryCost = pairCount > 0
                                            ? kJointGeometryWeight * geometryResidual /
                                                  static_cast<scalar_t>(pairCount)
                                            : 0.0;

            const scalar_t nominalArea = computeSortedPolygonArea(nominalPoints);
            const scalar_t selectedArea = computeSortedPolygonArea(selectedPoints);
            const scalar_t minArea = kJointAreaRetentionRatio * nominalArea;
            const scalar_t areaShortfall = std::max(minArea - selectedArea, scalar_t(0.0));
            costs.supportAreaCost = kJointAreaWeight * areaShortfall * areaShortfall;

            const scalar_t nominalLongitudinalSpan = computeSpan(nominalPoints, 0);
            const scalar_t nominalLateralSpan = computeSpan(nominalPoints, 1);
            const scalar_t selectedLongitudinalSpan = computeSpan(selectedPoints, 0);
            const scalar_t selectedLateralSpan = computeSpan(selectedPoints, 1);
            const scalar_t longitudinalShortfall =
                std::max(kLongitudinalSpanRetentionRatio * nominalLongitudinalSpan - selectedLongitudinalSpan,
                         scalar_t(0.0));
            const scalar_t lateralShortfall =
                std::max(kLateralSpanRetentionRatio * nominalLateralSpan - selectedLateralSpan, scalar_t(0.0));
            costs.supportSpanCost = kJointSupportSpanWeight *
                                    (longitudinalShortfall * longitudinalShortfall +
                                     lateralShortfall * lateralShortfall);

            if (selectedPoints.size() >= 3)
            {
                const auto predictedBase = computeCentroid2d(predictedBasePoints);
                const scalar_t supportMargin = computeSignedSupportMargin(selectedPoints, predictedBase);
                const scalar_t marginShortfall = std::max(kSupportMarginTarget - supportMargin, scalar_t(0.0));
                costs.supportMarginCost = kJointSupportMarginWeight * marginShortfall * marginShortfall;
            }

            return costs;
        }

        void optimizeJointFootholdSelection(std::vector<JointFootholdEvent>& events,
                                            const std::vector<SynchronousEventGroup>& groups)
        {
            if (events.empty() || groups.empty())
            {
                return;
            }

            for (auto& event : events)
            {
                if (event.hasValidRegion && !event.candidateScores.empty())
                {
                    scalar_t bestScore = std::numeric_limits<scalar_t>::infinity();
                    size_t bestIndex = 0;
                    for (size_t candidateIndex = 0; candidateIndex < event.candidateScores.size(); ++candidateIndex)
                    {
                        const auto score = computeLocalCost(event.candidateScores[candidateIndex]);
                        if (score < bestScore)
                        {
                            bestScore = score;
                            bestIndex = candidateIndex;
                        }
                    }
                    event.selectedIndex = bestIndex;
                }
            }

            for (const auto& group : groups)
            {
                if (group.memberIndices.size() < 2)
                {
                    continue;
                }

                for (size_t iteration = 0; iteration < kJointOptimizationIterations; ++iteration)
                {
                    bool changed = false;
                    for (size_t eventIndexInGroup = 0; eventIndexInGroup < group.memberIndices.size(); ++eventIndexInGroup)
                    {
                        auto& event = events[group.memberIndices[eventIndexInGroup]];
                        if (!event.hasValidRegion || event.candidateScores.empty())
                        {
                            continue;
                        }

                        scalar_t bestScore = std::numeric_limits<scalar_t>::infinity();
                        size_t bestIndex = event.selectedIndex;
                        for (size_t candidateIndex = 0; candidateIndex < event.candidateScores.size(); ++candidateIndex)
                        {
                            const auto jointCosts =
                                computeJointCostsForCandidate(events, group.memberIndices, eventIndexInGroup,
                                                              candidateIndex);
                            const scalar_t jointCost = jointCosts.total();
                            const scalar_t totalCost = computeLocalCost(event.candidateScores[candidateIndex]) + jointCost;
                            if (totalCost < bestScore)
                            {
                                bestScore = totalCost;
                                bestIndex = candidateIndex;
                            }
                        }

                        if (bestIndex != event.selectedIndex)
                        {
                            event.selectedIndex = bestIndex;
                            changed = true;
                        }
                    }

                    if (!changed)
                    {
                        break;
                    }
                }

                for (size_t eventIndexInGroup = 0; eventIndexInGroup < group.memberIndices.size(); ++eventIndexInGroup)
                {
                    auto& event = events[group.memberIndices[eventIndexInGroup]];
                    if (!event.hasValidRegion || event.candidateScores.empty())
                    {
                        continue;
                    }

                    for (size_t candidateIndex = 0; candidateIndex < event.candidateScores.size(); ++candidateIndex)
                    {
                        const auto jointCosts = computeJointCostsForCandidate(events, group.memberIndices,
                                                                              eventIndexInGroup, candidateIndex);
                        auto& score = event.candidateScores[candidateIndex];
                        score.supportCentroidCost = jointCosts.supportCentroidCost;
                        score.supportGeometryCost = jointCosts.supportGeometryCost;
                        score.supportAreaCost = jointCosts.supportAreaCost;
                        score.supportMarginCost = jointCosts.supportMarginCost;
                        score.supportSpanCost = jointCosts.supportSpanCost;
                        score.jointCost = jointCosts.total();
                        score.totalCost = computeLocalCost(score) + score.jointCost;
                    }
                }
            }
        }

        scalar_t wrapAngle(scalar_t angle)
        {
            return std::atan2(std::sin(angle), std::cos(angle));
        }

        convex_plane_decomposition::CgalPoint2d computePolygonCentroid(
            const convex_plane_decomposition::CgalPolygon2d& polygon)
        {
            if (polygon.size() == 0)
            {
                return {0.0, 0.0};
            }

            double sumX = 0.0;
            double sumY = 0.0;
            for (size_t i = 0; i < polygon.size(); ++i)
            {
                sumX += polygon.vertex(i).x();
                sumY += polygon.vertex(i).y();
            }
            return {sumX / static_cast<double>(polygon.size()), sumY / static_cast<double>(polygon.size())};
        }

        bool arePointsClose(const convex_plane_decomposition::CgalPoint2d& lhs,
                            const convex_plane_decomposition::CgalPoint2d& rhs)
        {
            const double dx = lhs.x() - rhs.x();
            const double dy = lhs.y() - rhs.y();
            return dx * dx + dy * dy <=
                   kFootholdCandidateDuplicateTolerance * kFootholdCandidateDuplicateTolerance;
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
        logger_ = std::make_unique<quadruped::logs::LogSave>();
        logger_->setEnabled(debugLoggingEnabled_);
    }
    // 指定腿在指定时间的足部投影
    convex_plane_decomposition::PlanarTerrainProjection ConvexRegionSelector::getProjection(
        size_t leg, scalar_t time) const
    {
        const auto index = lookup::findIndexInTimeArray(timeEvents_[leg], time);
        return feetProjections_[leg][index];
    }
    // 返回统一安全集 polygon: 已完成 inward shrink，评分、MPC 和 WBC 都使用同一集合。
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
        feet_array_t<std::vector<LockedFootholdRecord>> currentSelectedFootholdsByTime;
        feet_array_t<std::vector<int>> phaseEventIndices;
        std::vector<JointFootholdEvent> jointEvents;
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
            phaseEventIndices[leg] = std::vector<int>(numPhases, -1);

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
                        JointFootholdEvent jointEvent;
                        jointEvent.leg = leg;
                        jointEvent.phaseIndex = i;
                        jointEvent.standMiddleTime = standMiddleTime;
                        jointEvent.standStartTime = standStartTime;
                        jointEvent.standFinalTime = standFinalTime;

                        scalar_t footholdReferenceTime = standMiddleTime;
                        if (i > 0 && !contactFlagStocks[leg][i - 1] && i - 1 < eventTimes.size())
                        {
                            footholdReferenceTime = std::max(eventTimes[i - 1], initTime);
                        }

                        const vector_t desiredState = targetTrajectories.getDesiredState(footholdReferenceTime);
                        const vector3_t desiredFootPosition = endEffectorKinematicsPtr_->getPosition(desiredState)[leg];
                        const scalar_t swingDuration = computePrecedingSwingDuration(i, contactFlagStocks[leg], eventTimes);
                        const scalar_t desiredYawRate = estimateDesiredYawRate(targetTrajectories, info_, footholdReferenceTime);
                        jointEvent.predictedBasePosition =
                            centroidal_model::getBasePose(desiredState, info_).head(3) +
                            std::max(swingDuration, scalar_t(0.1)) *
                                centroidal_model::getNormalizedMomentum(desiredState, info_).head(3);
                        jointEvent.predictedBasePosition.z() = centroidal_model::getBasePose(desiredState, info_)(2);
                        const vector3_t nominalFoothold =
                            getNominalFoothold(leg, desiredState, initState, desiredFootPosition, swingDuration,
                                               desiredYawRate);

                        // 边缘距离惩罚函数，鼓励落足点远离边界，提升安全性
                        auto penaltyFunction = [this](const vector3_t& projectedPointInWorld) -> double {
                            // if (footholdScorer_ != nullptr)
                            // {
                            //     return footholdScorer_->computePenalty(projectedPointInWorld);
                            // }
                            return 0.0;
                        };
                        auto safeProjection = getBestPlanarRegionAtPositionInWorld(
                            nominalFoothold, planarTerrain_.planarRegions, penaltyFunction);

                        jointEvent.projection = safeProjection;
                        jointEvent.nominalFoothold = nominalFoothold;
                        jointEvent.hasLockedFoothold =
                            getPreviousSelectedFoothold(leg, standMiddleTime, jointEvent.lockedFoothold);

                        if (safeProjection.regionPtr == nullptr)
                        {
                            jointEvent.hasValidRegion = false;
                            phaseEventIndices[leg][i] = static_cast<int>(jointEvents.size());
                            jointEvents.push_back(std::move(jointEvent));
                            lastStandMiddleTime = standMiddleTime;
                            continue;
                        }

                        // 凸多边形生长 (numVertices=8, growthFactor=1.1 加速收敛)
                        constexpr double growthFactor = 1.05;
                        auto convexRegion = convex_plane_decomposition::growConvexPolygonInsideShape(
                            safeProjection.regionPtr->boundaryWithInset.boundary,
                            safeProjection.positionInTerrainFrame,
                            static_cast<int>(numVertices_), growthFactor);
                        const auto safetyPolygon = foot_placement::shrinkConvexPolygon(convexRegion);

                        jointEvent.hasValidRegion = safetyPolygon.size() >= 3;
                        jointEvent.safetyPolygon = safetyPolygon;
                        jointEvent.projectedFoothold = safeProjection.positionInWorld;

                        if (jointEvent.hasValidRegion)
                        {
                            jointEvent.candidatesInTerrain = sampleFootholdCandidates(
                                safetyPolygon, safeProjection.positionInTerrainFrame);
                            if (jointEvent.hasLockedFoothold)
                            {
                                const auto lockedCandidateInTerrain = positionInPlaneFrameFromPositionInWorld(
                                    jointEvent.lockedFoothold, safeProjection.regionPtr->transformPlaneToWorld);
                                if (convex_plane_decomposition::isInside(lockedCandidateInTerrain, safetyPolygon))
                                {
                                    bool duplicate = false;
                                    for (const auto& existing : jointEvent.candidatesInTerrain)
                                    {
                                        if (arePointsClose(existing, lockedCandidateInTerrain))
                                        {
                                            duplicate = true;
                                            break;
                                        }
                                    }
                                    if (!duplicate)
                                    {
                                        jointEvent.candidatesInTerrain.push_back(lockedCandidateInTerrain);
                                        jointEvent.hasLockedCandidate = true;
                                        jointEvent.lockedCandidateIndex = jointEvent.candidatesInTerrain.size() - 1;
                                    }
                                }
                            }
                            jointEvent.candidateScores.reserve(jointEvent.candidatesInTerrain.size());
                            scalar_t bestScore = std::numeric_limits<scalar_t>::infinity();
                            for (size_t candidateIndex = 0; candidateIndex < jointEvent.candidatesInTerrain.size(); ++candidateIndex)
                            {
                                auto score = evaluateFootholdCandidate(
                                    leg, standMiddleTime, *safeProjection.regionPtr, safetyPolygon,
                                    jointEvent.candidatesInTerrain[candidateIndex], safeProjection.positionInTerrainFrame,
                                    desiredState, initState, desiredFootPosition, nominalFoothold, swingDuration,
                                    desiredYawRate);
                                jointEvent.candidateScores.push_back(score);
                                if (jointEvent.hasLockedFoothold &&
                                    (jointEvent.candidateScores.back().candidateInWorld.head<2>() -
                                     jointEvent.lockedFoothold.head<2>()).norm() <= kLockHardRadius)
                                {
                                    jointEvent.hasLockedCandidate = true;
                                    jointEvent.lockedCandidateIndex = candidateIndex;
                                }
                                if (score.totalCost < bestScore)
                                {
                                    bestScore = score.totalCost;
                                    jointEvent.selectedIndex = candidateIndex;
                                }
                            }
                        }

                        phaseEventIndices[leg][i] = static_cast<int>(jointEvents.size());
                        jointEvents.push_back(std::move(jointEvent));
                        lastStandMiddleTime = standMiddleTime;
                    }
                    else
                    {
                        phaseEventIndices[leg][i] = phaseEventIndices[leg][i - 1];
                    }

                    if (standStartTime < initTime && initTime < standFinalTime)
                    {
                        initStandFinalTime_[leg] = standFinalTime;
                    }
                }
            }
        }

        const auto synchronousGroups = buildSynchronousEventGroups(jointEvents);
        optimizeJointFootholdSelection(jointEvents, synchronousGroups);

        for (size_t leg = 0; leg < info_.numThreeDofContacts; ++leg)
        {
            for (size_t i = 0; i < numPhases; ++i)
            {
                if (!contactFlagStocks[leg][i])
                {
                    continue;
                }

                const int eventIndex = phaseEventIndices[leg][i];
                if (eventIndex < 0)
                {
                    continue;
                }

                auto& jointEvent = jointEvents[eventIndex];
                auto selectedProjection = jointEvent.projection;
                vector3_t selectedFoothold = selectedProjection.positionInWorld;
                if (jointEvent.hasValidRegion && !jointEvent.candidateScores.empty())
                {
                    if (jointEvent.hasLockedCandidate && jointEvent.lockedCandidateIndex < jointEvent.candidateScores.size())
                    {
                        const auto& selectedScore = jointEvent.candidateScores[jointEvent.selectedIndex];
                        const auto& lockedScore = jointEvent.candidateScores[jointEvent.lockedCandidateIndex];
                        const scalar_t selectedDrift = jointEvent.hasLockedFoothold
                                                          ? (selectedScore.candidateInWorld.head<2>() -
                                                             jointEvent.lockedFoothold.head<2>()).norm()
                                                          : 0.0;
                        const bool keepLockedCandidate = jointEvent.hasLockedFoothold &&
                                                         lockedScore.edgePenaltyRaw <= kLockedCandidateEdgeThreshold &&
                                                         lockedScore.boundaryDistance >= kLockedCandidateBoundaryThreshold &&
                                                         (selectedDrift > kLockSoftRadius ||
                                                          lockedScore.totalCost <= selectedScore.totalCost +
                                                              kLockedCandidatePreferenceMargin);
                        if (keepLockedCandidate)
                        {
                            jointEvent.selectedIndex = jointEvent.lockedCandidateIndex;
                        }
                    }

                    selectedProjection.positionInTerrainFrame = jointEvent.candidatesInTerrain[jointEvent.selectedIndex];
                    selectedProjection.positionInWorld =
                        jointEvent.candidateScores[jointEvent.selectedIndex].candidateInWorld;
                    selectedFoothold = selectedProjection.positionInWorld;
                }

                feetProjections_[leg][i] = selectedProjection;
                convexPolygons_[leg][i] = jointEvent.safetyPolygon;
                nominalFootholds_[leg][i] = jointEvent.nominalFoothold;

                if (i == 0 || phaseEventIndices[leg][i - 1] != eventIndex)
                {
                    middleTimes_[leg].push_back(jointEvent.standMiddleTime);
                    currentSelectedFootholdsByTime[leg].push_back({jointEvent.standMiddleTime, selectedFoothold});
                    // logFootholdSelection(leg, jointEvent.standMiddleTime, jointEvent.nominalFoothold,
                                        //  jointEvent.projectedFoothold, jointEvent.candidateScores,
                                        //  jointEvent.selectedIndex);
                }
            }

            // During swing, reuse the next touchdown polygon so MPC can enforce a time-varying
            // convergence toward the target foothold region before contact is established.
            for (int i = static_cast<int>(numPhases) - 2; i >= 0; --i)
            {
                if (contactFlagStocks[leg][static_cast<size_t>(i)])
                {
                    continue;
                }

                int nextContactIndex = i + 1;
                while (nextContactIndex < static_cast<int>(numPhases) &&
                       !contactFlagStocks[leg][static_cast<size_t>(nextContactIndex)])
                {
                    ++nextContactIndex;
                }

                if (nextContactIndex >= static_cast<int>(numPhases))
                {
                    continue;
                }

                feetProjections_[leg][static_cast<size_t>(i)] = feetProjections_[leg][static_cast<size_t>(nextContactIndex)];
                convexPolygons_[leg][static_cast<size_t>(i)] = convexPolygons_[leg][static_cast<size_t>(nextContactIndex)];
                nominalFootholds_[leg][static_cast<size_t>(i)] = nominalFootholds_[leg][static_cast<size_t>(nextContactIndex)];
            }
        }

        for (size_t leg = 0; leg < info_.numThreeDofContacts; leg++)
        {
            timeEvents_[leg] = eventTimes;
        }

        previousSelectedFootholdsByTime_ = currentSelectedFootholdsByTime;
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
        constexpr scalar_t maxHorizontalCorrection = 0.12;

        vector3_t desiredVel = centroidal_model::getNormalizedMomentum(desiredState, info_).head(3);
        vector3_t measuredVel = centroidal_model::getNormalizedMomentum(initState, info_).head(3);

        const scalar_t previewTime = std::max(swingDuration, scalar_t(0.0));
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

    std::vector<convex_plane_decomposition::CgalPoint2d> ConvexRegionSelector::sampleFootholdCandidates(
        const convex_plane_decomposition::CgalPolygon2d& convexPolygon,
        const convex_plane_decomposition::CgalPoint2d& projectedPointInTerrain) const
    {
        std::vector<convex_plane_decomposition::CgalPoint2d> candidates;
        if (convexPolygon.size() < 3)
        {
            candidates.push_back(projectedPointInTerrain);
            return candidates;
        }

        const auto centroid = computePolygonCentroid(convexPolygon);
        const auto tryAddCandidate = [&candidates, &convexPolygon](
                                         const convex_plane_decomposition::CgalPoint2d& candidate) {
            if (!convex_plane_decomposition::isInside(candidate, convexPolygon))
            {
                return;
            }
            for (const auto& existing : candidates)
            {
                if (arePointsClose(existing, candidate))
                {
                    return;
                }
            }
            candidates.push_back(candidate);
        };

        tryAddCandidate(projectedPointInTerrain);
        tryAddCandidate(centroid);
        tryAddCandidate(convex_plane_decomposition::getPointOnLine(centroid, projectedPointInTerrain, 0.2));
        tryAddCandidate(convex_plane_decomposition::getPointOnLine(centroid, projectedPointInTerrain, 0.5));
        tryAddCandidate(convex_plane_decomposition::getPointOnLine(centroid, projectedPointInTerrain, 0.8));

        const size_t radialSamples = std::min<size_t>(8, convexPolygon.size());
        for (size_t sampleIndex = 0; sampleIndex < radialSamples; ++sampleIndex)
        {
            const size_t vertexIndex = sampleIndex * convexPolygon.size() / radialSamples;
            const auto vertex = convexPolygon.vertex(vertexIndex);
            const size_t nextIndex = (vertexIndex + 1) % convexPolygon.size();
            const auto nextVertex = convexPolygon.vertex(nextIndex);
            const convex_plane_decomposition::CgalPoint2d edgeMidpoint(
                0.5 * (vertex.x() + nextVertex.x()), 0.5 * (vertex.y() + nextVertex.y()));

            tryAddCandidate(convex_plane_decomposition::getPointOnLine(centroid, vertex, 0.45));
            tryAddCandidate(convex_plane_decomposition::getPointOnLine(centroid, vertex, 0.7));
            tryAddCandidate(convex_plane_decomposition::getPointOnLine(centroid, edgeMidpoint, 0.6));
        }

        if (candidates.empty())
        {
            candidates.push_back(projectedPointInTerrain);
        }
        return candidates;
    }

    convex_plane_decomposition::CgalPoint2d ConvexRegionSelector::selectBestFootholdCandidate(
        size_t leg,
        scalar_t standMiddleTime,
        const convex_plane_decomposition::PlanarRegion& region,
        const convex_plane_decomposition::CgalPolygon2d& convexPolygon,
        const convex_plane_decomposition::CgalPoint2d& projectedPointInTerrain,
        const vector_t& desiredState,
        const vector_t& initState,
        const vector3_t& desiredFootPosition,
        const vector3_t& nominalFoothold,
        scalar_t swingDuration,
        scalar_t desiredYawRate)
    {
        const auto candidates = sampleFootholdCandidates(convexPolygon, projectedPointInTerrain);

        scalar_t bestScore = std::numeric_limits<scalar_t>::infinity();
        auto bestCandidate = projectedPointInTerrain;
        std::vector<FootholdScoreBreakdown> candidateScores;
        candidateScores.reserve(candidates.size());
        size_t bestIndex = 0;
        for (const auto& candidate : candidates)
        {
            auto score = evaluateFootholdCandidate(leg, standMiddleTime, region, convexPolygon, candidate,
                                                   projectedPointInTerrain, desiredState, initState,
                                                   desiredFootPosition, nominalFoothold, swingDuration,
                                                   desiredYawRate);
            candidateScores.push_back(score);
            if (score.totalCost < bestScore)
            {
                bestScore = score.totalCost;
                bestCandidate = candidate;
                bestIndex = candidateScores.size() - 1;
            }
        }

        const vector3_t projectedFoothold = convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
            projectedPointInTerrain, region.transformPlaneToWorld);
        // logFootholdSelection(leg, standMiddleTime, nominalFoothold, projectedFoothold, candidateScores, bestIndex);

        return bestCandidate;
    }

    ConvexRegionSelector::FootholdScoreBreakdown ConvexRegionSelector::evaluateFootholdCandidate(
        size_t leg,
        scalar_t standMiddleTime,
        const convex_plane_decomposition::PlanarRegion& region,
        const convex_plane_decomposition::CgalPolygon2d& convexPolygon,
        const convex_plane_decomposition::CgalPoint2d& candidatePointInTerrain,
        const convex_plane_decomposition::CgalPoint2d& projectedPointInTerrain,
        const vector_t& desiredState,
        const vector_t& initState,
        const vector3_t& desiredFootPosition,
        const vector3_t& nominalFoothold,
        scalar_t swingDuration,
        scalar_t desiredYawRate) const
    {
        constexpr scalar_t comHeight = 0.26;
        constexpr scalar_t gravity = 9.81;
        constexpr scalar_t minBoundaryMargin = 0.025;
        constexpr scalar_t nominalTrackingWeight = 1.0;
        constexpr scalar_t projectedTrackingWeight = 0.6;
        constexpr scalar_t previewVelocityWeight = 0.25;
        constexpr scalar_t reachConsistencyWeight = 0.12;
        constexpr scalar_t boundaryMarginWeight = 0.8;
        constexpr scalar_t continuityWeight = 0.4;
        constexpr scalar_t edgePenaltyWeight = 0.6;

        FootholdScoreBreakdown breakdown;

        const vector3_t candidateInWorld = convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
            candidatePointInTerrain, region.transformPlaneToWorld);
        breakdown.candidateInWorld = candidateInWorld;
        const vector3_t projectedPointInWorld = convex_plane_decomposition::positionInWorldFrameFromPosition2dInPlane(
            projectedPointInTerrain, region.transformPlaneToWorld);

        vector3_t desiredVel = centroidal_model::getNormalizedMomentum(desiredState, info_).head(3);
        vector3_t measuredVel = centroidal_model::getNormalizedMomentum(initState, info_).head(3);
        desiredVel.z() = 0.0;
        measuredVel.z() = 0.0;

        const scalar_t previewTime = std::max(swingDuration, scalar_t(0.1));
        const scalar_t timeConstant = std::max(std::sqrt(comHeight / gravity), scalar_t(1e-3));

        const vector3_t basePosition = centroidal_model::getBasePose(desiredState, info_).head(3);

        vector3_t feedforward = 0.5 * previewTime * desiredVel;
        feedforward.z() = 0.0;

        vector3_t baseToDesiredFoot = desiredFootPosition - basePosition;
        baseToDesiredFoot.z() = 0.0;

        vector3_t turningCompensation;
        turningCompensation << -desiredYawRate * baseToDesiredFoot.y(),
            desiredYawRate * baseToDesiredFoot.x(), 0.0;
        turningCompensation *= 0.5 * previewTime;

        vector3_t candidateOffset = candidateInWorld - desiredFootPosition;
        candidateOffset.z() = 0.0;

        vector3_t velocityResidual = measuredVel - desiredVel -
                                     (candidateOffset - feedforward - turningCompensation) / timeConstant;
        velocityResidual.z() = 0.0;

        vector3_t predictedBaseAtTouchdown = basePosition + previewTime * desiredVel;
        predictedBaseAtTouchdown.z() = basePosition.z();

        vector3_t candidateReach = candidateInWorld - predictedBaseAtTouchdown;
        candidateReach.z() = 0.0;
        vector3_t desiredReach = desiredFootPosition - basePosition;
        desiredReach.z() = 0.0;

        const scalar_t nominalTrackingCost =
            nominalTrackingWeight * (candidateInWorld.head<2>() - nominalFoothold.head<2>()).squaredNorm();
        const scalar_t projectedTrackingCost =
            projectedTrackingWeight * (candidateInWorld.head<2>() - projectedPointInWorld.head<2>()).squaredNorm();
        const scalar_t previewVelocityCost =
            previewVelocityWeight * velocityResidual.head<2>().squaredNorm();
        const scalar_t reachConsistencyCost =
            reachConsistencyWeight * (candidateReach.head<2>() - desiredReach.head<2>()).squaredNorm();

        const scalar_t boundaryDistance = std::sqrt(std::max(
            convex_plane_decomposition::squaredDistance(candidatePointInTerrain, convexPolygon), 0.0));
        const scalar_t boundaryMarginViolation = std::max(minBoundaryMargin - boundaryDistance, scalar_t(0.0));
        const scalar_t boundaryMarginCost = boundaryMarginWeight * boundaryMarginViolation * boundaryMarginViolation;

        scalar_t continuityCost = 0.0;
        vector3_t previousSelectedFoothold = vector3_t::Zero();
        if (getPreviousSelectedFoothold(leg, standMiddleTime, previousSelectedFoothold))
        {
            breakdown.continuityDistance = (candidateInWorld.head<2>() - previousSelectedFoothold.head<2>()).norm();
            continuityCost = continuityWeight *
                             (candidateInWorld.head<2>() - previousSelectedFoothold.head<2>()).squaredNorm();
            const scalar_t lockDistance = breakdown.continuityDistance;
            breakdown.lockDistance = lockDistance;
            const scalar_t lockViolation = std::max(lockDistance - kLockHardRadius, scalar_t(0.0));
            breakdown.lockDeviationCost = kLockDeviationWeight * lockViolation * lockViolation;
        }

        const scalar_t edgePenaltyRaw = footholdScorer_ != nullptr ? footholdScorer_->computePenalty(candidateInWorld) : 0.0;
        const scalar_t edgePenaltyCost = edgePenaltyWeight * edgePenaltyRaw;

        breakdown.nominalTrackingCost = nominalTrackingCost;
        breakdown.projectedTrackingCost = projectedTrackingCost;
        breakdown.previewVelocityCost = previewVelocityCost;
        breakdown.reachConsistencyCost = reachConsistencyCost;
        breakdown.boundaryMarginCost = boundaryMarginCost;
        breakdown.continuityCost = continuityCost;
        breakdown.edgePenaltyRaw = edgePenaltyRaw;
        breakdown.edgePenaltyCost = edgePenaltyCost;
        breakdown.boundaryDistance = boundaryDistance;
        breakdown.totalCost = nominalTrackingCost + projectedTrackingCost + previewVelocityCost +
                              reachConsistencyCost + boundaryMarginCost + continuityCost + edgePenaltyCost +
                              breakdown.lockDeviationCost;

        return breakdown;
    }

    bool ConvexRegionSelector::getPreviousSelectedFoothold(size_t leg, scalar_t standMiddleTime,
                                                           vector3_t& footholdInWorld) const
    {
        constexpr scalar_t kFootholdTimeMatchTolerance = 0.05;
        if (leg >= previousSelectedFootholdsByTime_.size())
        {
            return false;
        }

        for (const auto& [cachedTime, cachedFoothold] : previousSelectedFootholdsByTime_[leg])
        {
            if (std::abs(cachedTime - standMiddleTime) <= kFootholdTimeMatchTolerance)
            {
                footholdInWorld = cachedFoothold;
                return true;
            }
        }

        return false;
    }

    // void ConvexRegionSelector::logFootholdSelection(size_t leg,
    //                                                 scalar_t standMiddleTime,
    //                                                 const vector3_t& nominalFoothold,
    //                                                 const vector3_t& projectedFoothold,
    //                                                 const std::vector<FootholdScoreBreakdown>& candidates,
    //                                                 size_t selectedIndex)
    // {
    //     if (!debugLoggingEnabled_ || logger_ == nullptr)
    //     {
    //         return;
    //     }

    //     ++debugLogCounter_;
    //     if (debugLogCounter_ % std::max<size_t>(debugLogDecimation_, 1) != 0)
    //     {
    //         return;
    //     }

    //     const std::string detailHeader =
    //         "leg,stand_middle_time,candidate_index,is_selected,candidate_x,candidate_y,candidate_z,"
    //         "nominal_x,nominal_y,nominal_z,projected_x,projected_y,projected_z,"
    //         "total_cost,nominal_cost,projected_cost,preview_vel_cost,reach_cost,boundary_cost,continuity_cost,"
    //         "edge_penalty_raw,edge_penalty_cost,boundary_distance,continuity_distance,"
    //         "support_centroid_cost,support_geometry_cost,support_area_cost,support_margin_cost,support_span_cost,"
    //         "joint_cost,lock_cost,lock_distance";
    //     const std::string summaryHeader =
    //         "leg,stand_middle_time,selected_x,selected_y,selected_z,nominal_x,nominal_y,nominal_z,"
    //         "projected_x,projected_y,projected_z,selected_total_cost,selected_edge_penalty_raw,selected_boundary_distance,"
    //         "selected_support_centroid_cost,selected_support_geometry_cost,selected_support_area_cost,"
    //         "selected_support_margin_cost,selected_support_span_cost,selected_joint_cost,selected_lock_cost,selected_lock_distance";

    //     for (size_t candidateIndex = 0; candidateIndex < candidates.size(); ++candidateIndex)
    //     {
    //         const auto& candidate = candidates[candidateIndex];
    //         logger_->saveDataToCSV(
    //             "foothold_candidate_scores.csv", detailHeader,
    //             {static_cast<double>(leg),
    //              static_cast<double>(standMiddleTime),
    //              static_cast<double>(candidateIndex),
    //              candidateIndex == selectedIndex ? 1.0 : 0.0,
    //              candidate.candidateInWorld.x(), candidate.candidateInWorld.y(), candidate.candidateInWorld.z(),
    //              nominalFoothold.x(), nominalFoothold.y(), nominalFoothold.z(),
    //              projectedFoothold.x(), projectedFoothold.y(), projectedFoothold.z(),
    //              static_cast<double>(candidate.totalCost),
    //              static_cast<double>(candidate.nominalTrackingCost),
    //              static_cast<double>(candidate.projectedTrackingCost),
    //              static_cast<double>(candidate.previewVelocityCost),
    //              static_cast<double>(candidate.reachConsistencyCost),
    //              static_cast<double>(candidate.boundaryMarginCost),
    //              static_cast<double>(candidate.continuityCost),
    //              static_cast<double>(candidate.edgePenaltyRaw),
    //              static_cast<double>(candidate.edgePenaltyCost),
    //              static_cast<double>(candidate.boundaryDistance),
    //              static_cast<double>(candidate.continuityDistance),
    //              static_cast<double>(candidate.supportCentroidCost),
    //              static_cast<double>(candidate.supportGeometryCost),
    //              static_cast<double>(candidate.supportAreaCost),
    //              static_cast<double>(candidate.supportMarginCost),
    //              static_cast<double>(candidate.supportSpanCost),
    //              static_cast<double>(candidate.jointCost),
    //              static_cast<double>(candidate.lockDeviationCost),
    //              static_cast<double>(candidate.lockDistance)});
    //     }

    //     if (selectedIndex < candidates.size())
    //     {
    //         const auto& selected = candidates[selectedIndex];
    //         logger_->saveDataToCSV(
    //             "foothold_selection_summary.csv", summaryHeader,
    //             {static_cast<double>(leg),
    //              static_cast<double>(standMiddleTime),
    //              selected.candidateInWorld.x(), selected.candidateInWorld.y(), selected.candidateInWorld.z(),
    //              nominalFoothold.x(), nominalFoothold.y(), nominalFoothold.z(),
    //              projectedFoothold.x(), projectedFoothold.y(), projectedFoothold.z(),
    //              static_cast<double>(selected.totalCost),
    //              static_cast<double>(selected.edgePenaltyRaw),
    //              static_cast<double>(selected.boundaryDistance),
    //              static_cast<double>(selected.supportCentroidCost),
    //              static_cast<double>(selected.supportGeometryCost),
    //              static_cast<double>(selected.supportAreaCost),
    //              static_cast<double>(selected.supportMarginCost),
    //              static_cast<double>(selected.supportSpanCost),
    //              static_cast<double>(selected.jointCost),
    //              static_cast<double>(selected.lockDeviationCost),
    //              static_cast<double>(selected.lockDistance)});
    //     }
    // }
} // namespace legged
