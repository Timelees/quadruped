//
// Created by biao on 3/21/25.
//

#pragma once

#include <convex_plane_decomposition/PlanarRegion.h>
#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_quadruped_controller/interface/LeggedInterface.h>
#include <ocs2_sphere_approximation/PinocchioSphereInterface.h>
#include <grid_map_sdf/SignedDistanceField.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ocs2_quadruped_controller/perceptive/interface/PerceptiveLeggedReferenceManager.h"

namespace ocs2::legged_robot
{
    class PerceptiveLeggedInterface final : public LeggedInterface
    {
    public:
        using LeggedInterface::LeggedInterface;


        void setupOptimalControlProblem(const std::string& taskFile,
                                        const std::string& urdfFile,
                                        const std::string& referenceFile,
                                        bool verbose) override;

        void setupReferenceManager(const std::string& taskFile, const std::string& urdfFile,
                                   const std::string& referenceFile,
                                   bool verbose) override;

        void setupPreComputation(const std::string& taskFile, const std::string& urdfFile,
                                 const std::string& referenceFile,
                                 bool verbose);

        std::shared_ptr<grid_map::SignedDistanceField> getSignedDistanceFieldPtr() const
        {
            return signedDistanceFieldPtr_;
        }

        std::shared_ptr<convex_plane_decomposition::PlanarTerrain> getPlanarTerrainPtr() const
        {
            return planarTerrainPtr_;
        }

        std::shared_ptr<PinocchioSphereInterface> getPinocchioSphereInterfacePtr() const
        {
            return pinocchioSphereInterfacePtr_;
        }

        size_t getNumVertices() const { return numVertices_; }

    protected:
        PerceptiveLeggedReferenceManager::FootPlacementRelaxationConfig loadFootPlacementRelaxationConfig(
            const std::string& taskFile, const std::string& prefix, bool verbose) const;

        size_t numVertices_ = 10;

        std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
        std::shared_ptr<grid_map::SignedDistanceField> signedDistanceFieldPtr_;
        std::shared_ptr<PinocchioSphereInterface> pinocchioSphereInterfacePtr_;

        std::unique_ptr<StateCost> getFootPlacementConstraint(size_t contactPointIndex,
                                                              const std::string& taskFile,
                                                              const std::string& prefix,
                                                              bool verbose);

        std::unique_ptr<StateInputCost> getFootPlacementCbfConstraint(size_t contactPointIndex,
                                                                   const std::string& taskFile,
                                                                   const std::string& prefix,
                                                                   bool verbose);
    };
} // namespace ocs2::legged_robot
