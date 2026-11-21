//
// Created by biao on 3/21/25.
//

#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ocs2_core/reference/ModeSchedule.h>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

namespace quadruped::logs { class LogSave; }

namespace ocs2::legged_robot
{
    class ConvexRegionSelector
    {
    public:
        ~ConvexRegionSelector();
        struct FootholdScoreBreakdown
        {
            vector3_t candidateInWorld = vector3_t::Zero();
            scalar_t totalCost = 0.0;
            scalar_t nominalTrackingCost = 0.0;
            scalar_t projectedTrackingCost = 0.0;
            scalar_t previewVelocityCost = 0.0;
            scalar_t reachConsistencyCost = 0.0;
            scalar_t boundaryMarginCost = 0.0;
            scalar_t continuityCost = 0.0;
            scalar_t edgePenaltyRaw = 0.0;
            scalar_t edgePenaltyCost = 0.0;
            scalar_t boundaryDistance = 0.0;
            scalar_t continuityDistance = 0.0;
            scalar_t supportCentroidCost = 0.0;
            scalar_t supportGeometryCost = 0.0;
            scalar_t supportAreaCost = 0.0;
            scalar_t supportMarginCost = 0.0;
            scalar_t supportSpanCost = 0.0;
            scalar_t jointCost = 0.0;
            scalar_t lockDeviationCost = 0.0;
            scalar_t lockDistance = 0.0;
        };

        ConvexRegionSelector(CentroidalModelInfo info,
                             std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr,
                             const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                             size_t numVertices);

        void update(const ModeSchedule& modeSchedule, scalar_t initTime, const vector_t& initState,
                    TargetTrajectories& targetTrajectories,
                    const feet_array_t<vector3_t>* acceptedTouchdowns = nullptr);

        convex_plane_decomposition::PlanarTerrainProjection getProjection(size_t leg, scalar_t time) const;

        convex_plane_decomposition::CgalPolygon2d getConvexPolygon(size_t leg, scalar_t time) const;

        vector3_t getNominalFootholds(size_t leg, scalar_t time) const;

        std::vector<scalar_t> getMiddleTimes(size_t leg) const { return middleTimes_[leg]; }

        std::vector<convex_plane_decomposition::PlanarTerrainProjection> getProjections(size_t leg)
        {
            return feetProjections_[leg];
        }

        std::shared_ptr<convex_plane_decomposition::PlanarTerrain> getPlanarTerrainPtr() { return planarTerrainPtr_; }

        feet_array_t<scalar_t> getInitStandFinalTimes() { return initStandFinalTime_; }

        feet_array_t<std::vector<bool>> extractContactFlags(const std::vector<size_t>& phaseIDsStock) const;

    private:
        class FootholdScorer
        {
        public:
            explicit FootholdScorer(const convex_plane_decomposition::PlanarTerrain* planarTerrainPtr);

            double computePenalty(const vector3_t& projectedPointInWorld) const;

        private:
            const convex_plane_decomposition::PlanarTerrain* planarTerrainPtr_;
        };

        // Own the plane: projections otherwise point into the map replaced each solve.
        struct CommittedFoothold
        {
            scalar_t touchdownTime = 0.0;
            scalar_t stanceEndTime = 0.0;
            convex_plane_decomposition::PlanarRegion region;
            convex_plane_decomposition::PlanarTerrainProjection projection;
            convex_plane_decomposition::CgalPolygon2d polygon;
            vector3_t nominal = vector3_t::Zero();
        };
        feet_array_t<std::unique_ptr<CommittedFoothold>> committedFootholds_;

        struct LockedFootholdRecord
        {
            scalar_t standMiddleTime = 0.0;
            vector3_t footholdInWorld = vector3_t::Zero();
        };

        static std::pair<int, int> findIndex(size_t index, const std::vector<bool>& contactFlagStock);

        vector3_t getNominalFoothold(size_t leg, const vector_t& desiredState, const vector_t& initState,
                                     const vector3_t& desiredFootPosition, scalar_t swingDuration,
                                     scalar_t desiredYawRate) const;

        std::vector<convex_plane_decomposition::CgalPoint2d> sampleFootholdCandidates(
            const convex_plane_decomposition::CgalPolygon2d& convexPolygon,
            const convex_plane_decomposition::CgalPoint2d& projectedPointInTerrain) const;

        convex_plane_decomposition::CgalPoint2d selectBestFootholdCandidate(
            size_t leg, scalar_t standMiddleTime, const convex_plane_decomposition::PlanarRegion& region,
            const convex_plane_decomposition::CgalPolygon2d& convexPolygon,
            const convex_plane_decomposition::CgalPoint2d& projectedPointInTerrain, const vector_t& desiredState,
            const vector_t& initState, const vector3_t& desiredFootPosition, const vector3_t& nominalFoothold,
            scalar_t swingDuration, scalar_t desiredYawRate);

        FootholdScoreBreakdown evaluateFootholdCandidate(
            size_t leg, scalar_t standMiddleTime, const convex_plane_decomposition::PlanarRegion& region,
            const convex_plane_decomposition::CgalPolygon2d& convexPolygon,
            const convex_plane_decomposition::CgalPoint2d& candidatePointInTerrain,
            const convex_plane_decomposition::CgalPoint2d& projectedPointInTerrain, const vector_t& desiredState,
            const vector_t& initState, const vector3_t& desiredFootPosition, const vector3_t& nominalFoothold,
            scalar_t swingDuration, scalar_t desiredYawRate) const;

        bool getPreviousSelectedFoothold(size_t leg, scalar_t standMiddleTime, vector3_t& footholdInWorld) const;

        void logFootholdSelection(size_t leg, scalar_t standMiddleTime, const vector3_t& nominalFoothold,
                                  const vector3_t& projectedFoothold,
                                  const std::vector<FootholdScoreBreakdown>& candidates, size_t selectedIndex);

        feet_array_t<std::vector<convex_plane_decomposition::PlanarTerrainProjection>> feetProjections_;
        feet_array_t<std::vector<convex_plane_decomposition::CgalPolygon2d>> convexPolygons_;
        feet_array_t<std::vector<vector3_t>> nominalFootholds_;
        feet_array_t<std::vector<scalar_t>> middleTimes_;
        feet_array_t<scalar_t> initStandFinalTime_;
        feet_array_t<std::vector<scalar_t>> timeEvents_;
        feet_array_t<std::vector<LockedFootholdRecord>> previousSelectedFootholdsByTime_;

        vector3_t previousCommandPreview_ = vector3_t::Zero();
        bool commandPreviewInitialized_ = false;

        const CentroidalModelInfo info_;
        size_t numVertices_;

        convex_plane_decomposition::PlanarTerrain planarTerrain_;
        std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
        std::unique_ptr<FootholdScorer> footholdScorer_;
        std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;

        std::unique_ptr<quadruped::logs::LogSave> logger_;
        bool debugLoggingEnabled_ = true;
        size_t debugLogDecimation_ = 1;
        size_t debugLogCounter_ = 0;
    };
} // namespace ocs2::legged_robot
