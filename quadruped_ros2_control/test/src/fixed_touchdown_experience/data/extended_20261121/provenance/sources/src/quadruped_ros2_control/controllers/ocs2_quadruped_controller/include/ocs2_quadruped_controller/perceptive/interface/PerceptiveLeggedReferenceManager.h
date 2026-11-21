//
// Created by biao on 3/21/25.
//

#pragma once

#include <memory>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include "ocs2_quadruped_controller/perceptive/interface/ConvexRegionSelector.h"

#include <ocs2_quadruped_controller/interface/SwitchedModelReferenceManager.h>

namespace ocs2::legged_robot
{
    class PlanarTerrainReceiver;

    class PerceptiveLeggedReferenceManager : public SwitchedModelReferenceManager
    {
    public:
        struct FootPlacementRelaxationConfig
        {
            FootPlacementRelaxationConfig(scalar_t maxRelaxationValue = 0.04,
                                          scalar_t relaxationPowerValue = 2.0,
                                          scalar_t swingActivationProgressValue = 0.7)
                : maxRelaxation(maxRelaxationValue),
                  relaxationPower(relaxationPowerValue),
                  swingActivationProgress(swingActivationProgressValue)
            {
            }

            scalar_t maxRelaxation = 0.04;
            scalar_t relaxationPower = 2.0;
            scalar_t swingActivationProgress = 0.7;
        };

        PerceptiveLeggedReferenceManager(CentroidalModelInfo info, std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                         std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                         std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr,
                                         const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                         scalar_t comHeight,
                                         FootPlacementRelaxationConfig footPlacementRelaxationConfig =
                                             FootPlacementRelaxationConfig());

        void setJointLimits(vector_t lower, vector_t upper) {
            jointLower_ = std::move(lower);
            jointUpper_ = std::move(upper);
        }

        void setTerrainReceiver(std::shared_ptr<PlanarTerrainReceiver> receiver) {
            terrainReceiver_ = std::move(receiver);
        }

        void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState) override;

        const std::shared_ptr<ConvexRegionSelector>& getConvexRegionSelectorPtr() const { return convexRegionSelectorPtr_; }

        contact_flag_t getFootPlacementFlags(scalar_t time) const;

        scalar_t getFootPlacementRelaxation(size_t leg, scalar_t time) const;

        const FootPlacementRelaxationConfig& getFootPlacementRelaxationConfig() const
        {
            return footPlacementRelaxationConfig_;
        }


    protected:
        void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                              TargetTrajectories& targetTrajectories,
                              ModeSchedule& modeSchedule) override;

        virtual void updateSwingTrajectoryPlanner(scalar_t initTime, const vector_t& initState,
                                                  ModeSchedule& modeSchedule, const TargetTrajectories& targets);

        void modifyProjections(scalar_t initTime, const vector_t& initState, size_t leg, size_t initIndex,
                               const std::vector<bool>& contactFlagStocks,
                               std::vector<convex_plane_decomposition::PlanarTerrainProjection>& projections);

        std::pair<scalar_array_t, scalar_array_t> getHeights(const std::vector<bool>& contactFlagStocks,
                                                             const std::vector<
                                                                 convex_plane_decomposition::PlanarTerrainProjection>&
                                                             projections);

        /**
         * 获取摆动相位中每条腿的抬脚和落脚3D位置。
         */
        std::pair<feet_array_t<std::vector<Eigen::Vector3d>>, feet_array_t<std::vector<Eigen::Vector3d>>>
            getSwingPositions3d(const feet_array_t<std::vector<bool>>& contactFlagStocks,
                               const feet_array_t<std::vector<convex_plane_decomposition::PlanarTerrainProjection>>& allProjections) const;

        Eigen::Vector2d swingHeightBounds(size_t leg, const vector_t& state, const Eigen::Vector3d& position) const;
        bool solveSwingIk(size_t leg, const vector3_t& desired, vector_t& state) const;
        vector_t swingNominalState(scalar_t time, scalar_t initTime, const vector_t& initState,
                                  const TargetTrajectories& targets) const;
        vector_t jointLower_, jointUpper_;
        const CentroidalModelInfo info_;
        feet_array_t<vector3_t> lastLiftoffPos_;

        std::shared_ptr<ConvexRegionSelector> convexRegionSelectorPtr_;
        std::shared_ptr<PlanarTerrainReceiver> terrainReceiver_;
        std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;

        scalar_t comHeight_;
        FootPlacementRelaxationConfig footPlacementRelaxationConfig_;
    };
} // namespace legged
