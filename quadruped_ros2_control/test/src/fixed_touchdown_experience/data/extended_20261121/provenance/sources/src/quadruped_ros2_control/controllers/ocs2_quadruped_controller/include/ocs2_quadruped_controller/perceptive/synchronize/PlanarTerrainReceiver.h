//
// Created by biao on 3/21/25.
//

#ifndef PLANARTERRAINRECEIVER_H
#define PLANARTERRAINRECEIVER_H

#include "ocs2_quadruped_controller/interface/constraint/TerrainSafetyCorridor.h"

#include <atomic>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <convex_plane_decomposition_msgs/msg/planar_terrain.hpp>

#include <convex_plane_decomposition/PlanarRegion.h>
#include <grid_map_sdf/SignedDistanceField.hpp>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

namespace ocs2::legged_robot
{
    class PlanarTerrainReceiver : public SolverSynchronizedModule
    {
    public:
        PlanarTerrainReceiver(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                              const std::shared_ptr<convex_plane_decomposition::PlanarTerrain>& planarTerrainPtr,
                              const std::shared_ptr<grid_map::SignedDistanceField>& signedDistanceFieldPtr,
                              const std::string& mapTopic,
                              const std::string& sdfElevationLayer);

        void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& currentState,
                          const ReferenceManagerInterface& referenceManager) override;

        void postSolverRun(const PrimalSolution& /*primalSolution*/) override
        {
        }

        TerrainMapTiming mapTiming(double controllerTime) const;

        // Fuse only corresponding world cells; rolling-buffer indices are not coordinates.
        static void filterHeightLayer(grid_map::GridMap& current, const grid_map::GridMap& previous,
                                      const std::string& layer, double alpha);
        static void preserveObservationLayer(grid_map::GridMap& map, const std::string& layer);

    private:
        bool sanitizeElevationLayer(convex_plane_decomposition::PlanarTerrain& planarTerrain) const;
        bool acceptTimestamp(grid_map::Time timestamp) const;
        bool acceptMapCenter(const convex_plane_decomposition::PlanarTerrain& planarTerrain,
                             double& centerJump) const;
        bool hasFilterCompatibleGeometry(const grid_map::GridMap& lhs, const grid_map::GridMap& rhs) const;
        convex_plane_decomposition::PlanarTerrain filterTerrain(
            const convex_plane_decomposition::PlanarTerrain& candidate,
            double centerJump) const;
        double getOrDeclareDoubleParameter(const std::string& name, double defaultValue) const;

        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        rclcpp::Subscription<convex_plane_decomposition_msgs::msg::PlanarTerrain>::SharedPtr subscription_;

        std::string sdfElevationLayer_;

        std::mutex mutex_;
        std::atomic_bool updated_ = false;

        convex_plane_decomposition::PlanarTerrain pendingTerrain_;
        std::unique_ptr<grid_map::SignedDistanceField> pendingSdf_;
        convex_plane_decomposition::PlanarTerrain filteredTerrain_;
        bool hasFilteredTerrain_ = false;
        grid_map::Time lastAcceptedTimestamp_ = 0;

        double maxMapAge_ = 1.0;
        double certificateAge_ = 2.0;
        double futureTimestampTolerance_ = 0.2;
        double maxCenterJump_ = 0.6;
        double centerFilterThreshold_ = 0.05;
        double heightFilterAlpha_ = 0.5;

        std::shared_ptr<convex_plane_decomposition::PlanarTerrain> planarTerrainPtr_;
        std::shared_ptr<grid_map::SignedDistanceField> sdfPtr_;
    };
}


#endif //PLANARTERRAINRECEIVER_H
