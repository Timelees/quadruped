//
// Created by GitHub Copilot on 3/24/26.
//

#pragma once

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_mpc/SystemObservation.h>

#include "ocs2_quadruped_controller/perceptive/synchronize/EdgeSphere.h"

namespace ocs2::legged_robot {

class EdgeVisualization {
 public:
  EdgeVisualization(std::shared_ptr<EdgeSphereArray> edgeSpheresPtr,
                    const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                    scalar_t maxUpdateFrequency = 20.0);

  void update(const SystemObservation& observation);

 private:
  std::shared_ptr<EdgeSphereArray> edgeSpheresPtr_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPublisher_;
  scalar_t lastTime_;
  scalar_t minPublishTimeDifference_;
};

}  // namespace ocs2::legged_robot