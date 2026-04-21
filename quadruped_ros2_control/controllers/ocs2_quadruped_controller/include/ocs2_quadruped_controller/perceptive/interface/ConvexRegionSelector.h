//
// Created by biao on 3/21/25.
//

#pragma once

#include <ocs2_core/reference/ModeSchedule.h>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <convex_plane_decomposition/PolygonTypes.h>
#include <convex_plane_decomposition/SegmentedPlaneProjection.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_legged_robot/common/Types.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include "ocs2_quadruped_controller/perceptive/constraint/EdgeAwareFootholdScorer.h"

namespace ocs2::legged_robot
{
    class ConvexRegionSelector
    {
    public:
        ConvexRegionSelector(CentroidalModelInfo info,
                             std::shared_ptr<convex_plane_decomposition::PlanarTerrain> PlanarTerrainPtr,
                             const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t numVertices);

        /** 设置边缘感知落足点评分器，传入后 update() 会在选择落足点时对靠近边缘的候选点施加惩罚 */
        // void setFootholdScorer(EdgeAwareFootholdScorer* scorer) { footholdScorer_ = scorer; }

        void update(const ModeSchedule& modeSchedule, scalar_t initTime, const vector_t& initState,
                    TargetTrajectories& targetTrajectories);

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
        static std::pair<int, int> findIndex(size_t index, const std::vector<bool>& contactFlagStock);

        vector3_t getNominalFoothold(size_t leg, const vector_t& desiredState, const vector_t& initState,
                         const vector3_t& desiredFootPosition, scalar_t swingDuration,
                         scalar_t desiredYawRate) const;

        feet_array_t<std::vector<convex_plane_decomposition::PlanarTerrainProjection>> feetProjections_;
        feet_array_t<std::vector<convex_plane_decomposition::CgalPolygon2d>> convexPolygons_;

        feet_array_t<std::vector<vector3_t>> nominalFootholds_;
        feet_array_t<std::vector<scalar_t>> middleTimes_;

        feet_array_t<scalar_t> initStandFinalTime_;

        feet_array_t<std::vector<scalar_t>> timeEvents_;

        const CentroidalModelInfo info_;
        size_t numVertices_;

        convex_plane_decomposition::PlanarTerrain planarTerrain_;
        std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
        std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;

        // EdgeAwareFootholdScorer* footholdScorer_ = nullptr;  // 非拥有指针，生命周期由调用方管理
    };
} // namespace ocs2::legged_robot
