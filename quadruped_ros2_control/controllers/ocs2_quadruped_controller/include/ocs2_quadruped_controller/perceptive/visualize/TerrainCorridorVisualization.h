#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <ocs2_core/Types.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "ocs2_quadruped_controller/interface/constraint/TerrainSafetyCorridor.h"

namespace ocs2::legged_robot {
// Shared by the controller and the independent ROS scene. All vertices are
// sphere-centre coordinates; wireframes show geom/plan, fill shows plan.
visualization_msgs::msg::MarkerArray terrainCorridorMarkers(
    const std::vector<TerrainSafetyCorridor>& corridors, const std_msgs::msg::Header& header);

class TerrainCorridorVisualization {
   public:
    TerrainCorridorVisualization(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                 size_t numFeet, scalar_t maxUpdateFrequency = 20.0);
    void update(const std::vector<TerrainSafetyCorridor>& corridors, const std_msgs::msg::Header& header);
    static constexpr const char* kTopic = "terrain_safety_corridor";

   private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};
}  // namespace ocs2::legged_robot
