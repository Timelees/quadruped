//
// Created by GitHub Copilot on 3/24/26.
//

#include "ocs2_quadruped_controller/perceptive/visualize/EdgeVisualization.h"

#include <limits>

#include <ocs2_ros_interfaces/common/RosMsgHelpers.h>
#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>

namespace ocs2::legged_robot {

EdgeVisualization::EdgeVisualization(std::shared_ptr<EdgeSphereArray> edgeSpheresPtr,
                                     const rclcpp_lifecycle::LifecycleNode::SharedPtr& node,
                                     scalar_t maxUpdateFrequency)
    : edgeSpheresPtr_(std::move(edgeSpheresPtr)),
      lastTime_(std::numeric_limits<scalar_t>::lowest()),
      minPublishTimeDifference_(1.0 / maxUpdateFrequency) {
  markerPublisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("edge_spheres", 1);
}

void EdgeVisualization::update(const SystemObservation& observation) {
  if (observation.time - lastTime_ <= minPublishTimeDifference_) {
    return;
  }
  lastTime_ = observation.time;

  visualization_msgs::msg::MarkerArray markers;

  visualization_msgs::msg::Marker deleteAll;
  deleteAll.action = visualization_msgs::msg::Marker::DELETEALL;
  deleteAll.header.frame_id = "odom";
  markers.markers.push_back(deleteAll);

  if (!edgeSpheresPtr_) {
    markerPublisher_->publish(markers);
    return;
  }

  for (size_t i = 0; i < edgeSpheresPtr_->size(); ++i) {
    const auto& sphere = edgeSpheresPtr_->at(i);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";
    marker.ns = "edge_spheres";
    marker.id = static_cast<int>(i);
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = ros_msg_helpers::getPointMsg(sphere.center);
    marker.pose.orientation.w = 1.0;

    const double d = 2.0 * sphere.radius;
    marker.scale.x = d;
    marker.scale.y = d;
    marker.scale.z = d;
    marker.color = ros_msg_helpers::getColor(getRGB(Color::orange), 0.55);
    marker.frame_locked = false;
    markers.markers.push_back(marker);
  }

  markerPublisher_->publish(markers);
}

}  // namespace ocs2::legged_robot